using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using UnityStandardAssets.Vehicles.Car;
using Assets.WayPointSystem2;
using WayPointManager = Assets.WayPointSystem2.WayPointManager;
using Assets.OtherDriver;
using Assets.Traffic_Lights_System.Scripts;

public class MoveAlongCircuitWithCarControl : MonoBehaviour
{
    #region Public Settings and Parameters

    [Header("Circuit Settings")]
    public WayPointManager wayPointManager;
    public WayPointCircuit Circuit;
    [Tooltip("Base speed (in addition to the track-dependent speed)")]
    public float Speed = 5.0f;
    [Tooltip("Layer for the road surface (used e.g. for raycasts)")]
    public LayerMask RoadLayer;
    public string CarName;

    [Header("Indicators & Offsets")]
    public IndicatorManager indicatorManager;
    [Tooltip("Default rotation (Euler angles) to align the vehicle correctly")]
    public Vector3 StandardRotationEulerAngles;
    [Tooltip("Offset (X: lateral, Y: vertical, Z: longitudinal) relative to the path")]
    public Vector3 Offset = Vector3.zero;

    [Header("Driving Behavior & Wait Points")]
    [Tooltip("Calculated distance traveled along the circuit")]
    public float DistanceTravelled;
    [Range(0, 100)]
    public float HumanBehavior = 0f;
    public List<WaitPoint> WaitPoints = new List<WaitPoint>();

    [Header("Driving Dynamics")]
    [SerializeField] private float accelForce = 3f;
    [SerializeField] private float brakeForce = 10f;

    [Header("Distance and Braking Logic")]
    [Tooltip("Maximum 'lookahead' distance at which to brake")]
    public float brakingDistance = 20f;
    [Tooltip("If a vehicle is closer than this distance, stop")]
    public float safeDistance = 8f;
    [Tooltip("Speed is reduced by this factor if a vehicle is ahead")]
    public float brakingMultiplier = 0.25f;
    [Tooltip("LayerMask for vehicles (e.g. 'CarLayer')")]
    public LayerMask carLayerMask;
    public bool UseStandardRotationForDetection = false;

    [Header("Cross-Track & Swerve Correction")]
    [Tooltip("Factor to correct lateral error – higher values correct more strongly")]
    public float CorrectionFactor = 0.05f;
    private const float SwerveAmplitude = 0.5f;
    private const float SwerveFrequency = 0f;

    [Header("Off-Track Correction")]
    [Tooltip("If the car is more than this distance (units) off the circuit, it will be reset")]
    public float offTrackDistanceThreshold = 15f;
    [Tooltip("If the car is off the track for longer than this time (seconds), it will be reset")]
    public float offTrackTimeThreshold = 5f;
    private float offTrackTimer = 0f;

    #endregion

    #region Private Fields and Internal Variables

    private Quaternion _standardRotation;
    private float _currentVelocity = 0f;
    private float _originalSpeed;

    private bool _isWaiting = false;
    private float _waitTimer = 0f;

    private float[] _waypointDistances;

    private int turnCooldownCounter = 0;
    private int lastWaypointIndex = -1;
    private bool _pendingTurn = false;
    private int _pendingTurnWaypointIndex = -1;
    private PossibleTurn _pendingTurnCandidate;
    private int _passedWaypointsSinceTurn = 0;
    private int _lastWaypointIndexAfterTurn = -1;
    private int _collectedTurnCandidateIndex = -1;
    private PossibleTurn _collectedTurnCandidate = null;

    public float steeringBoostMultiplier = 0.3f;
    public float speedReductionMultiplier = 0.5f;

    private bool useFallback = false;

    public CarController carController;

    #endregion

    #region Unity Methods

    /// <summary>
    /// Initializes components and checks for CarController.
    /// </summary>
    private void Awake()
    {
        carController = GetComponent<CarController>();
        indicatorManager = GetComponent<IndicatorManager>();
        if (carController == null)
        {
            useFallback = true;
        }
        
    }

    /// <summary>
    /// Sets initial rotation, speed, and precomputes waypoint distances.
    /// </summary>
    private void Start()
    {
        _standardRotation = Quaternion.Euler(StandardRotationEulerAngles);
        _originalSpeed = Speed;
        if (Circuit != null)
        {
            PrecomputeWaypointDistances();
        }
        else
        {
            Debug.LogWarning("No circuit set - please assign a valid circuit in the Inspector.");
        }
     
    }

    /// <summary>
    /// Updates the car's movement and handles speed, turning, waiting, and path projection.
    /// </summary>
    private void FixedUpdate()
    {
        if (Circuit == null || Circuit.WayPoints == null || !Circuit.WayPoints.Any())
            return;

        if (!useFallback)
        {
            // Original-Logik mit CarController
            _currentVelocity = carController.CurrentSpeed;
            if (carController.CurrentSpeed < 0.1f)
            {
                _currentVelocity = 0f;
            }

            UpdateDistanceTravelledBasedOnPosition();

            int currentWaypointIndex = GetCurrentCircuitWaypointIndex();
            if (currentWaypointIndex != lastWaypointIndex)
            {
                _passedWaypointsSinceTurn++;
                lastWaypointIndex = currentWaypointIndex;
            }

            bool redLightFound = CheckForRedLight(out float stopDist);
            bool carAhead = CheckForCarInFront(out float distanceToCar);
            float segSpeed = GetSegmentSpeed(DistanceTravelled);
            float baseSpeed = segSpeed + (Speed * (HumanBehavior / 100f));
            float targetSpeed = baseSpeed;

            if (redLightFound)
                targetSpeed = 0f;
            else if (carAhead)
            {
                if (distanceToCar <= safeDistance)
                    targetSpeed = 0f;
                else
                    targetSpeed *= brakingMultiplier;
            }

            if (_isWaiting)
                targetSpeed = 0f;
            if (_pendingTurn)
                targetSpeed *= 0.5f;

            if (_currentVelocity < targetSpeed)
                _currentVelocity = Mathf.MoveTowards(_currentVelocity, targetSpeed, accelForce * Time.deltaTime);
            else
                _currentVelocity = Mathf.MoveTowards(_currentVelocity, targetSpeed, brakeForce * Time.deltaTime);

            if (redLightFound && stopDist >= 0f)
            {
                _currentVelocity = 0f;
            }

            EvaluatePossibleTurn();
            HandleWaiting();

            if (_pendingTurn)
            {
                float currentDistanceMod = DistanceTravelled % Circuit.Length;
                float targetWaypointDistance = _waypointDistances[_pendingTurnWaypointIndex];
                float diff = Mathf.Abs(currentDistanceMod - targetWaypointDistance);
                float distanceDiff = Mathf.Min(diff, Circuit.Length - diff);
                float threshold = 5f;
                if (distanceDiff < threshold)
                {
                    WayPoint turnWaypoint = _pendingTurnCandidate.wayPoint;
                    WayPointCircuit newCircuit = FindCircuitForWaypoint(turnWaypoint);
                    if (newCircuit != null)
                    {
                        Circuit = newCircuit;
                        PrecomputeWaypointDistances();
                        int index = newCircuit.WayPoints.IndexOf(turnWaypoint);
                        if (index != -1 && _waypointDistances != null && index < _waypointDistances.Length)
                        {
                            DistanceTravelled = _waypointDistances[index];
                        }
                        else
                        {
                            DistanceTravelled = 0f;
                        }

                        Debug.Log("Turn executed at waypoint " + _pendingTurnWaypointIndex + " - new circuit set.");
                        turnCooldownCounter = 10;
                        _passedWaypointsSinceTurn = 0;
                        lastWaypointIndex = GetCurrentCircuitWaypointIndex();
                    }
                    else
                    {
                        Debug.LogWarning("No valid circuit found for the turn waypoint!");
                    }

                    _pendingTurn = false;
                    _pendingTurnWaypointIndex = -1;
                    _pendingTurnCandidate = null;
                }
            }
            else
            {
                indicatorManager.DeactivateAll();
            }

            int lookAheadOffset = _pendingTurn ? 3 : 2;
            float futureDistance = DistanceTravelled;
            if (_waypointDistances != null && _waypointDistances.Length > 0)
            {
                int currentIndex = GetCurrentCircuitWaypointIndex();
                int lookAheadIndex = currentIndex + lookAheadOffset;
                if (Circuit.closedLoop)
                    lookAheadIndex %= Circuit.WayPoints.Count;
                else
                    lookAheadIndex = Mathf.Min(lookAheadIndex, Circuit.WayPoints.Count - 1);
                futureDistance = _waypointDistances[lookAheadIndex];
            }

            RoutePoint curveRP = GetRoutePoint(futureDistance);
            float curveAngleDiff = Vector3.Angle(transform.forward, curveRP.Direction);
            float curveThreshold = 15f; // Schwellenwert ab dem abgebremst wird
            if (curveAngleDiff > curveThreshold)
            {
                float factor = Mathf.InverseLerp(curveThreshold, 45f, curveAngleDiff);
                targetSpeed *= Mathf.Lerp(1f, 0.3f, factor);
            }

            RoutePoint futureRoutePoint = GetRoutePoint(futureDistance);
            Vector3 desiredDirection = futureRoutePoint.Direction;
            Vector3 basePosition = futureRoutePoint.Position;
            Vector3 tangentRight = Vector3.Cross(Vector3.up, desiredDirection).normalized;

            float swerveAmount = (HumanBehavior / 100f) * SwerveAmplitude;
            float swerve = Mathf.Sin(Time.time * SwerveFrequency) * swerveAmount;
            Vector3 swervedPosition = basePosition + tangentRight * swerve + Vector3.up * Offset.y;

            RaycastHit hit;
            Vector3 rayOrigin = swervedPosition + Vector3.up * 10f;
            if (Physics.Raycast(rayOrigin, Vector3.down, out hit, 20f, RoadLayer))
                swervedPosition = hit.point + Vector3.up * Offset.y;

            Quaternion desiredRotation = _standardRotation * Quaternion.LookRotation(desiredDirection);

            Vector3 deviation = (basePosition + Offset) - transform.position;
            float lateralError = Vector3.Dot(deviation, tangentRight);
            float correctiveSteering = lateralError * CorrectionFactor;
            float angleDiff = Vector3.SignedAngle(transform.forward, desiredDirection, Vector3.up);
            float baseSteering = Mathf.Clamp(angleDiff / 45f, -1f, 1f);

            float finalSteeringInput;
            if (Mathf.Abs(lateralError) > offTrackDistanceThreshold)
            {
                int closestIndex = GetClosestWaypointIndex();
                Vector3 recoveryDirection = (Circuit.WayPoints[closestIndex].Position - transform.position).normalized;
                finalSteeringInput =
                    Mathf.Clamp(Vector3.SignedAngle(transform.forward, recoveryDirection, Vector3.up) / 45f, -1f, 1f);
            }
            else
            {
                finalSteeringInput = Mathf.Clamp(baseSteering + correctiveSteering, -1f, 1f);
            }

            float currentSpeed = carController.CurrentSpeed;
            float throttle = 0f;
            float brake = 0f;
            if (Mathf.Approximately(targetSpeed, 0f))
            {
                throttle = 0f;
                brake = -1f;
            }
            else
            {
                if (currentSpeed < targetSpeed)
                    throttle = Mathf.Clamp01((targetSpeed - currentSpeed) / targetSpeed);
                else
                    brake = Mathf.Clamp01((currentSpeed - targetSpeed) / targetSpeed) * -1;
            }

            carController.Move(finalSteeringInput, throttle, brake, 0f);
        }
        else
        {
            // Fallback-Logik (ohne CarController): direkte Positions- und Rotationsanpassung
            FallbackFixedUpdate();
        }
    }
    private void FallbackFixedUpdate()
    {
        bool redLightFound = CheckForRedLight(out float stopDist);
        bool carAhead = CheckForCarInFront(out float distanceToCar);
        float segSpeed = GetSegmentSpeed(DistanceTravelled);
        float baseSpeed = segSpeed + (Speed * (HumanBehavior / 100f));
        float targetSpeed = baseSpeed;

        if (redLightFound)
            targetSpeed = 0f;
        else if (carAhead)
        {
            if (distanceToCar <= safeDistance)
                targetSpeed = 0f;
            else
                targetSpeed *= brakingMultiplier;
        }
        if (_isWaiting)
            targetSpeed = 0f;
        if (_pendingTurn)
            targetSpeed *= 0.5f;

        if (_currentVelocity < targetSpeed)
            _currentVelocity = Mathf.MoveTowards(_currentVelocity, targetSpeed, accelForce * Time.deltaTime);
        else
            _currentVelocity = Mathf.MoveTowards(_currentVelocity, targetSpeed, brakeForce * Time.deltaTime);

        if (redLightFound && stopDist >= 0f)
            _currentVelocity = 0f;

        DistanceTravelled += _currentVelocity * Time.deltaTime;

        EvaluatePossibleTurn();
        HandleWaiting();

        if (_pendingTurn)
        {
            float currentDistanceMod = DistanceTravelled % Circuit.Length;
            float targetWaypointDistance = _waypointDistances[_pendingTurnWaypointIndex];
            float diff = Mathf.Abs(currentDistanceMod - targetWaypointDistance);
            float distanceDiff = Mathf.Min(diff, Circuit.Length - diff);
            float threshold = 5f;
            if (distanceDiff < threshold)
            {
                WayPoint turnWaypoint = _pendingTurnCandidate.wayPoint;
                WayPointCircuit newCircuit = FindCircuitForWaypoint(turnWaypoint);
                if (newCircuit != null)
                {
                    Circuit = newCircuit;
                    PrecomputeWaypointDistances();
                    int index = newCircuit.WayPoints.IndexOf(turnWaypoint);
                    if (index != -1 && _waypointDistances != null && index < _waypointDistances.Length)
                    {
                        DistanceTravelled = _waypointDistances[index];
                    }
                    else
                    {
                        DistanceTravelled = 0f;
                    }
                    Debug.Log("Turn executed (fallback) at waypoint " + _pendingTurnWaypointIndex + " - new circuit set.");
                    turnCooldownCounter = 10;
                    _passedWaypointsSinceTurn = 0;
                    lastWaypointIndex = GetCurrentCircuitWaypointIndex();
                }
                else
                {
                    Debug.LogWarning("No valid circuit found for turn waypoint (fallback)!");
                }
                _pendingTurn = false;
                _pendingTurnWaypointIndex = -1;
                _pendingTurnCandidate = null;
            }
        }
        else
        {
            indicatorManager.DeactivateAll();
        }


        UpdateVehiclePositionAndRotation_Fallback();
    }

    private void UpdateVehiclePositionAndRotation_Fallback()
    {
        RoutePoint routePoint = GetRoutePoint(DistanceTravelled);
        Vector3 direction = (Speed >= 0) ? routePoint.Direction : -routePoint.Direction;
        Vector3 right = Vector3.Cross(Vector3.up, direction).normalized;
        Vector3 horizontalOffset = right * Offset.x + direction * Offset.z;
        Vector3 basePosition = routePoint.Position + horizontalOffset;
        float swerveAmount = (HumanBehavior / 100f) * SwerveAmplitude;
        float swerve = Mathf.Sin(Time.time * 2f) * swerveAmount;  // Hier wird z. B. eine Frequenz von 2 verwendet
        Vector3 swervedPosition = basePosition + right * swerve;
        Vector3 rayOrigin = swervedPosition + Vector3.up * 10f;
        if (Physics.Raycast(rayOrigin, Vector3.down, out RaycastHit hit, 20f, RoadLayer))
        {
            swervedPosition = hit.point;
        }
        swervedPosition += Vector3.up * Offset.y;
        transform.position = swervedPosition;
        transform.rotation = _standardRotation * Quaternion.LookRotation(direction);
    }



    /// <summary>
    /// Draws a blue line from the car to the nearest waypoint in the Editor.
    /// </summary>
    private void OnDrawGizmos()
    {
        if (Circuit != null && Circuit.WayPoints != null && Circuit.WayPoints.Count > 0)
        {
            WayPoint closestWP = GetClosestWaypointOnCircuit();
            if (closestWP != null)
            {
                Gizmos.color = Color.blue;
                Gizmos.DrawLine(transform.position, closestWP.Position);
            }
        }
    }

    #endregion

    #region Path Calculation and Helper Methods

    /// <summary>
    /// Returns the route point (position and direction) at the given distance along the circuit using a Catmull-Rom spline.
    /// </summary>
    /// <param name="dist">Distance along the circuit.</param>
    /// <returns>A RoutePoint with position and direction.</returns>
    private RoutePoint GetRoutePoint(float dist)
    {
        RoutePoint rp = new RoutePoint();
        if (Circuit.closedLoop)
        {
            dist %= Circuit.Length;
            if (dist < 0)
                dist += Circuit.Length;
        }
        else
        {
            dist = Mathf.Clamp(dist, 0f, Circuit.Length);
        }
        int segIndex = 0;
        while (segIndex < _waypointDistances.Length - 1 && _waypointDistances[segIndex + 1] < dist)
            segIndex++;
        float segDist = dist - _waypointDistances[segIndex];
        float segLength = 0f;
        if (segIndex == _waypointDistances.Length - 1)
        {
            if (Circuit.closedLoop)
                segLength = Vector3.Distance(Circuit.WayPoints[segIndex].Position, Circuit.WayPoints[0].Position);
        }
        else
        {
            segLength = Vector3.Distance(Circuit.WayPoints[segIndex].Position, Circuit.WayPoints[segIndex + 1].Position);
        }
        float t = (segLength > 0.0001f) ? (segDist / segLength) : 0f;
        int p0Index = segIndex - 1;
        int p1Index = segIndex;
        int p2Index = segIndex + 1;
        int p3Index = segIndex + 2;
        Vector3 p0 = GetWaypointPosition(p0Index);
        Vector3 p1 = GetWaypointPosition(p1Index);
        Vector3 p2 = GetWaypointPosition(p2Index);
        Vector3 p3 = GetWaypointPosition(p3Index);
        Vector3 catmullPos = CatmullRomPosition(t, p0, p1, p2, p3);
        float delta = 0.001f;
        float tAhead = Mathf.Clamp01(t + delta);
        float tBehind = Mathf.Clamp01(t - delta);
        Vector3 posAhead = CatmullRomPosition(tAhead, p0, p1, p2, p3);
        Vector3 posBehind = CatmullRomPosition(tBehind, p0, p1, p2, p3);
        Vector3 dir = (posAhead - posBehind).normalized;
        rp.Position = catmullPos;
        rp.Direction = dir;
        return rp;
    }

    /// <summary>
    /// Calculates a position on a Catmull-Rom spline.
    /// </summary>
    private Vector3 CatmullRomPosition(float t, Vector3 p0, Vector3 p1, Vector3 p2, Vector3 p3)
    {
        float t2 = t * t;
        float t3 = t2 * t;
        float b0 = 0.5f * (-t3 + 2f * t2 - t);
        float b1 = 0.5f * (3f * t3 - 5f * t2 + 2f);
        float b2 = 0.5f * (-3f * t3 + 4f * t2 + t);
        float b3 = 0.5f * (t3 - t2);
        return p0 * b0 + p1 * b1 + p2 * b2 + p3 * b3;
    }

    /// <summary>
    /// Returns the position of a waypoint, accounting for closed circuits.
    /// </summary>
    private Vector3 GetWaypointPosition(int index)
    {
        int count = Circuit.WayPoints.Count;
        if (Circuit.closedLoop)
            index = (index % count + count) % count;
        else
        {
            if (index < 0)
                index = 0;
            if (index > count - 1)
                index = count - 1;
        }
        return Circuit.WayPoints[index].Position;
    }

    /// <summary>
    /// Precomputes the distances between waypoints and sets the total circuit length.
    /// </summary>
    public void PrecomputeWaypointDistances()
    {
        if (Circuit == null || Circuit.WayPoints.Count < 2)
        {
            _waypointDistances = null;
            if (Circuit != null)
                Circuit.Length = 0f;
            return;
        }
        _waypointDistances = new float[Circuit.WayPoints.Count];
        float total = 0f;
        for (int i = 0; i < Circuit.WayPoints.Count; i++)
        {
            if (i == 0)
                _waypointDistances[i] = 0f;
            else
            {
                Vector3 prev = Circuit.WayPoints[i - 1].Position;
                Vector3 curr = Circuit.WayPoints[i].Position;
                total += Vector3.Distance(prev, curr);
                _waypointDistances[i] = total;
            }
        }
        Circuit.Length = total;
    }

    /// <summary>
    /// Returns the interpolated speed for the current segment based on waypoint speeds.
    /// </summary>
    private float GetSegmentSpeed(float dist)
    {
        if (Circuit.closedLoop)
        {
            dist %= Circuit.Length;
            if (dist < 0)
                dist += Circuit.Length;
        }
        else
            dist = Mathf.Clamp(dist, 0, Circuit.Length);
        int segIndex = 0;
        while (segIndex < _waypointDistances.Length - 1 && _waypointDistances[segIndex + 1] < dist)
            segIndex++;
        float segDist = dist - _waypointDistances[segIndex];
        float segLength = 0f;
        if (segIndex == _waypointDistances.Length - 1)
        {
            if (Circuit.closedLoop)
                segLength = Vector3.Distance(Circuit.WayPoints[segIndex].Position, Circuit.WayPoints[0].Position);
            else
                segLength = 0f;
        }
        else
            segLength = Vector3.Distance(Circuit.WayPoints[segIndex].Position, Circuit.WayPoints[segIndex + 1].Position);
        float t = (segLength > 0.0001f) ? (segDist / segLength) : 0f;
        float speedA = Circuit.WayPoints[segIndex].Speed;
        int nextIndex = (segIndex + 1) % Circuit.WayPoints.Count;
        float speedB = Circuit.WayPoints[nextIndex].Speed;
        return Mathf.Lerp(speedA, speedB, t);
    }

    /// <summary>
    /// Returns the index of the circuit waypoint closest to the current vehicle position.
    /// </summary>
    private int GetCurrentCircuitWaypointIndex()
    {
        if (Circuit == null || Circuit.WayPoints == null || Circuit.WayPoints.Count == 0)
            return -1;
        int index = 0;
        float closestDistance = float.MaxValue;
        for (int i = 0; i < Circuit.WayPoints.Count; i++)
        {
            float dist = Vector3.Distance(transform.position, Circuit.WayPoints[i].Position);
            if (dist < closestDistance)
            {
                closestDistance = dist;
                index = i;
            }
        }
        return index;
    }

    /// <summary>
    /// Returns the index of the closest waypoint on the circuit.
    /// </summary>
    private int GetClosestWaypointIndex()
    {
        int index = 0;
        float closestDistance = float.MaxValue;
        for (int i = 0; i < Circuit.WayPoints.Count; i++)
        {
            float dist = Vector3.Distance(transform.position, Circuit.WayPoints[i].Position);
            if (dist < closestDistance)
            {
                closestDistance = dist;
                index = i;
            }
        }
        return index;
    }

    /// <summary>
    /// Finds and returns the circuit associated with the given waypoint.
    /// </summary>
    private WayPointCircuit FindCircuitForWaypoint(WayPoint wp)
    {
        foreach (var circuit in wayPointManager.Circuits)
        {
            if (circuit.WayPoints.Contains(wp))
                return circuit;
        }
        return null;
    }

    /// <summary>
    /// Returns the waypoint on the circuit that is closest to the current vehicle position.
    /// </summary>
    private WayPoint GetClosestWaypointOnCircuit()
    {
        if (Circuit == null || Circuit.WayPoints == null || Circuit.WayPoints.Count == 0)
            return null;
        WayPoint closestWaypoint = Circuit.WayPoints[0];
        float closestDistance = Vector3.Distance(transform.position, closestWaypoint.Position);
        foreach (WayPoint wp in Circuit.WayPoints)
        {
            float d = Vector3.Distance(transform.position, wp.Position);
            if (d < closestDistance)
            {
                closestDistance = d;
                closestWaypoint = wp;
            }
        }
        return closestWaypoint;
    }

    /// <summary>
    /// Sets the start point of the car based on the provided route index.
    /// </summary>
    public void SetStartPoint(float routeIndex)
    {
        if (Circuit == null || !Circuit.WayPoints.Any())
        {
            wayPointManager = FindObjectsOfType<WayPointManager>().FirstOrDefault();
            if (wayPointManager != null && wayPointManager.Circuits != null && wayPointManager.Circuits.Count > 0)
            {
                int randomCircuitIndex = Random.Range(0, wayPointManager.Circuits.Count);
                Circuit = wayPointManager.Circuits[randomCircuitIndex];
                Debug.Log("No circuit provided, randomly selected circuit '" + Circuit.name + "'.");
                PrecomputeWaypointDistances();
            }
            else
            {
                Debug.LogWarning("No circuit provided and WayPointManager contains no circuits. Start point set to 0.");
                DistanceTravelled = 0f;
                return;
            }
        }
        if (_waypointDistances != null && _waypointDistances.Length > 0)
        {
            int index = (int)routeIndex;
            if (index < 0 || index >= _waypointDistances.Length)
            {
                index = Random.Range(0, _waypointDistances.Length);
                Debug.Log("Invalid start index. Randomly selected index: " + index);
            }
            DistanceTravelled = _waypointDistances[index];
            RoutePoint rp = GetRoutePoint(DistanceTravelled);
            transform.position = rp.Position;
            transform.rotation = Quaternion.LookRotation(rp.Direction);
            if(!useFallback)
                carController.SetCarPositionAndRotation(rp.Position, Quaternion.LookRotation(rp.Direction));
            Debug.Log("Start point set at waypoint index " + index + ". Distance = " + DistanceTravelled);
        }
        else
        {
            Debug.LogWarning("No waypoint distances available, start point set to 0.");
            DistanceTravelled = 0f;
        }
    }

    /// <summary>
    /// Updates the DistanceTravelled based on projecting the vehicle's position onto the circuit.
    /// </summary>
    private void UpdateDistanceTravelledBasedOnPosition()
    {
        if (Circuit == null || Circuit.WayPoints == null || Circuit.WayPoints.Count < 2)
            return;

        int count = Circuit.WayPoints.Count;
        int closestIndex = GetClosestWaypointIndex();
        int windowSize = 10;
        float bestDistance = float.MaxValue;
        float bestProjectedDistance = _waypointDistances[closestIndex];
        for (int offset = -windowSize; offset <= windowSize; offset++)
        {
            int i = (closestIndex + offset + count) % count;
            if (!Circuit.closedLoop && i >= count - 1)
                continue;

            int next = (i + 1) % count;
            Vector3 A = Circuit.WayPoints[i].Position;
            Vector3 B = Circuit.WayPoints[next].Position;
            Vector3 AB = B - A;
            Vector3 AP = transform.position - A;
            float t = Mathf.Clamp01(Vector3.Dot(AP, AB) / Vector3.Dot(AB, AB));
            Vector3 projection = A + t * AB;
            float dist = Vector3.Distance(transform.position, projection);
            if (dist < bestDistance)
            {
                bestDistance = dist;
                float segLength = Vector3.Distance(A, B);
                bestProjectedDistance = _waypointDistances[i] + t * segLength;
                if (Circuit.closedLoop)
                    bestProjectedDistance %= Circuit.Length;
            }
        }
        DistanceTravelled = Mathf.Lerp(DistanceTravelled, bestProjectedDistance, 0.2f);
    }

    /// <summary>
    /// Sets the RoadLayer based on the provided layer name.
    /// </summary>
    public void SetRoadLayerByName(string layerName)
    {
        int layerIndex = LayerMask.NameToLayer(layerName);
        if (layerIndex == -1)
        {
            Debug.LogError($"Layer '{layerName}' does not exist. Please create it in the project settings.");
            return;
        }
        RoadLayer = 1 << layerIndex;
        Debug.Log($"RoadLayer set to '{layerName}' (Index: {layerIndex}).");
    }

    #endregion

    /// <summary>
    /// Evaluates possible turns and activates the appropriate indicator if a valid turn candidate is found.
    /// </summary>
    public void EvaluatePossibleTurn()
    {
        if (_pendingTurn)
            return;
        if (_passedWaypointsSinceTurn < 5)
        {
            indicatorManager.DeactivateAll();
            return;
        }
        if (turnCooldownCounter > 0)
        {
            turnCooldownCounter--;
            return;
        }

        int currentIndex = GetCurrentCircuitWaypointIndex();
        if (currentIndex == -1)
            return;

        float currentDistance = DistanceTravelled % Circuit.Length;
        int totalWaypoints = Circuit.WayPoints.Count;
        float dynamicTurnDetectionDistance = brakingDistance + 0.5f * _currentVelocity;
        int maxOffset = Mathf.Min(totalWaypoints, Mathf.CeilToInt(dynamicTurnDetectionDistance * 2));
        List<(int candidateIndex, float candidateDiff, PossibleTurn candidateTurn)> candidates = new List<(int, float, PossibleTurn)>();

        for (int offset = 1; offset <= maxOffset; offset++)
        {
            int candidateIndex = (currentIndex + offset) % totalWaypoints;
            WayPoint candidateWaypoint = Circuit.WayPoints[candidateIndex];

            if (candidateWaypoint.PossibleTurns != null && candidateWaypoint.PossibleTurns.Count > 0)
            {
                float waypointDistance = _waypointDistances[candidateIndex] % Circuit.Length;
                float diff = (waypointDistance - currentDistance + Circuit.Length) % Circuit.Length;

                if (diff > dynamicTurnDetectionDistance)
                    continue;

                PossibleTurn candidateTurn = candidateWaypoint.PossibleTurns[Random.Range(0, candidateWaypoint.PossibleTurns.Count)];
                WayPointCircuit candidateCircuit = FindCircuitForWaypoint(candidateTurn.wayPoint);
                if (candidateCircuit == null || candidateCircuit == Circuit)
                    continue;

                candidates.Add((candidateIndex, diff, candidateTurn));
            }
        }

        if (candidates.Count > 0)
        {
            var bestCandidate = candidates.OrderBy(c => c.candidateDiff).First();
            _collectedTurnCandidateIndex = bestCandidate.candidateIndex;
            _collectedTurnCandidate = bestCandidate.candidateTurn;
            Debug.Log("Collected turn candidate updated to waypoint " + bestCandidate.candidateIndex + " with diff " + bestCandidate.candidateDiff);

            if (bestCandidate.candidateDiff < dynamicTurnDetectionDistance)
            {
                _pendingTurn = true;
                _pendingTurnWaypointIndex = bestCandidate.candidateIndex;
                _pendingTurnCandidate = bestCandidate.candidateTurn;

                if (_pendingTurnCandidate.indicatorDirection == IndicatorDirection.Left)
                    indicatorManager.ActivateLeft();
                else if (_pendingTurnCandidate.indicatorDirection == IndicatorDirection.Right)
                    indicatorManager.ActivateRight();
                else
                    indicatorManager.DeactivateAll();

                Debug.Log("Turn candidate locked at waypoint " + bestCandidate.candidateIndex);
                _collectedTurnCandidate = null;
                _collectedTurnCandidateIndex = -1;
            }
        }
        else
        {
            _collectedTurnCandidate = null;
            _collectedTurnCandidateIndex = -1;
            indicatorManager.DeactivateAll();
        }
    }

    /// <summary>
    /// Manages waiting at specific points along the route.
    /// </summary>
    private void HandleWaiting()
    {
        if (_isWaiting)
        {
            _waitTimer -= Time.deltaTime;
            if (_waitTimer <= 0f)
            {
                _isWaiting = false;
                Speed = _originalSpeed;
            }
        }
        else
        {
            foreach (WaitPoint wp in WaitPoints)
            {
                float distance = wp._repeatPerRount ? DistanceTravelled % Circuit.Length : DistanceTravelled;
                if (Mathf.Abs(distance - wp._routeDistance) < 0.1f)
                {
                    StartWaiting(wp._waitTime);
                    break;
                }
            }
        }
    }

    /// <summary>
    /// Starts waiting for a specified time.
    /// </summary>
    private void StartWaiting(float waitTime)
    {
        _isWaiting = true;
        _waitTimer = waitTime;
        Debug.Log($"Waiting {waitTime} seconds at distance {DistanceTravelled}.");
    }

    /// <summary>
    /// Checks for a car in front and returns the distance to the closest car.
    /// </summary>
    private bool CheckForCarInFront(out float distanceToCar)
    {
        distanceToCar = Mathf.Infinity;
        float dynamicBrakingDistance = brakingDistance + 0.5f * _currentVelocity;
        Vector3 sphereCenter = transform.position + transform.up * 0.5f;
        Vector3 forwardDirection = UseStandardRotationForDetection ? _standardRotation * Vector3.forward : transform.forward;
        Vector3 rayStart = transform.position + transform.up * 0.5f;
        Debug.DrawRay(rayStart, forwardDirection * dynamicBrakingDistance, Color.red);
        float coneHalfAngle = 30f - _currentVelocity;

        if(wayPointManager == null)
            wayPointManager = FindObjectsOfType<WayPointManager>().FirstOrDefault();
#if UNITY_EDITOR
        if (wayPointManager.ShowBrakingSphere)
        {
            Vector3 apex = rayStart;
            Vector3 baseCenter = apex + forwardDirection * dynamicBrakingDistance;
            float halfAngleRad = coneHalfAngle * Mathf.Deg2Rad;
            float baseRadius = dynamicBrakingDistance * Mathf.Tan(halfAngleRad);
            Vector3 rightDir = Vector3.Cross(forwardDirection, Vector3.up);
            if (rightDir.sqrMagnitude < 0.0001f)
            {
                rightDir = Vector3.Cross(forwardDirection, Vector3.forward);
            }
            rightDir.Normalize();
            Vector3 upDir = Vector3.Cross(forwardDirection, rightDir).normalized;
            int segments = 20;
            for (int i = 0; i < segments; i++)
            {
                float theta = (float)i / segments * 2f * Mathf.PI;
                float nextTheta = (float)(i + 1) / segments * 2f * Mathf.PI;
                Vector3 point1 = baseCenter + (rightDir * Mathf.Cos(theta) + upDir * Mathf.Sin(theta)) * baseRadius;
                Vector3 point2 = baseCenter + (rightDir * Mathf.Cos(nextTheta) + upDir * Mathf.Sin(nextTheta)) * baseRadius;
                Debug.DrawLine(point1, point2, Color.yellow);
                Debug.DrawLine(apex, point1, Color.yellow);
                Debug.DrawLine(apex, point2, Color.yellow);
            }
        }
#endif

        Collider[] hits = Physics.OverlapSphere(sphereCenter, dynamicBrakingDistance, carLayerMask);
        bool carFound = false;
        foreach (Collider hitCollider in hits)
        {
            if (hitCollider.gameObject == this.gameObject)
                continue;
            Vector3 dirToCollider = (hitCollider.bounds.center - transform.position).normalized;
            float angle = Vector3.Angle(forwardDirection, dirToCollider);
            if (angle <= coneHalfAngle)
            {
                float dist = Vector3.Distance(transform.position, hitCollider.bounds.center);
                if (dist < distanceToCar)
                {
                    distanceToCar = dist;
                    carFound = true;
                }
            }
        }
        return carFound;
    }

    /// <summary>
    /// Checks for a red light within braking distance.
    /// </summary>
    private bool CheckForRedLight(out float stopDist)
    {
        stopDist = -1f;
        if (Circuit == null || Circuit.WayPoints.Count == 0)
            return false;
        float currentDistance = DistanceTravelled % Circuit.Length;
        float dynamicBrakingDistance = brakingDistance + 0.5f * _currentVelocity;
        for (int i = 0; i < Circuit.WayPoints.Count; i++)
        {
            WayPoint wp = Circuit.WayPoints[i];
            float waypointDistance = _waypointDistances[i] % Circuit.Length;
            float distDiff = (waypointDistance - currentDistance + Circuit.Length) % Circuit.Length;
            if (distDiff >= 0f && distDiff < dynamicBrakingDistance)
            {
                if (wp.Stop || (wp.TrafficLight != null && wp.TrafficLight.LightState == LightState.Red))
                {
                    stopDist = waypointDistance;
                    return true;
                }
            }
        }
        return false;
    }

    /// <summary>
    /// Returns the current position on the route.
    /// </summary>
    public Vector3 Position
    {
        get { return GetRoutePoint(DistanceTravelled).Position; }
    }
}
