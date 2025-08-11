using System.Collections.Generic;
using System.Linq;
using Assets.OtherDriver;
using Assets.Traffic_Lights_System.Scripts;
using UnityEngine;
using UnityEngine.Analytics;

namespace Assets.WayPointSystem2
{
    [System.Serializable]
    public struct RoutePoint
    {
        public Vector3 Position;
        public Vector3 Direction;
    }

    public class MoveAlongCircuit : MonoBehaviour
    {
        public WayPointManager wayPointManager;

        [Header("Allgemeine Einstellungen")]
        public WayPointCircuit Circuit;
        public float Speed = 5.0f;
        public LayerMask RoadLayer;
        public Vector3 Position;
        public string CarName;

        [Header("Blinker Einstellungen")]
        public IndicatorManager indicatorManager;

        [Header("Start-Einstellungen")]
        public float DistanceTravelled;
        public Vector3 StandardRotationEulerAngles;
        private Quaternion _standardRotation;
        public Vector3 Offset = Vector3.zero;

        [Header("Verhalten / Swerving")]
        [Range(0, 100)]
        public float HumanBehavior = 50f;
        private const float SwerveAmplitude = 0.5f;
        private const float SwerveFrequency = 2f;

        [Header("Wartepunkte / Stops")]
        public List<WaitPoint> WaitPoints = new List<WaitPoint>();
        private bool _isWaiting;
        private float _waitTimer;
        private float _originalSpeed;

        private float[] _waypointDistances;
        private float _currentVelocity = 0f;

        [SerializeField] private float brakeForce = 10f;
        [SerializeField] private float accelForce = 3f;

        [Header("Abstands-Kontrolle (Fahrzeugerkennung)")]
        [Tooltip("Maximaler 'Blick' nach vorne, ab wann wir bremsen.")]
        public float brakingDistance = 20f;
        [Tooltip("Wenn das vordere Auto näher als diese Distanz ist, halten wir an.")]
        public float safeDistance = 8f;
        [Tooltip("Geschwindigkeit wird um diesen Faktor verringert, wenn ein Auto vor uns ist.")]
        public float brakingMultiplier = 0.3f;
        public bool UseStandardRotationForDetection = false;
        [Tooltip("LayerMask für Autos (z. B. 'CarLayer').")]
        public LayerMask carLayerMask;

        [Header("Abbiege-Einstellungen")]
        [Tooltip("Wahrscheinlichkeit (0 bis 1), mit der an einem Wegpunkt mit PossibleTurn abgebogen wird.")]
        public float turnChance = 0.1f;
        private int turnCooldownCounter = 0;
        private int lastWaypointIndex = -1;
        private bool _turnIndicatorActive = false;
        private bool _pendingTurn = false;
        private int _pendingTurnWaypointIndex = -1;
        private PossibleTurn _pendingTurnCandidate;

        private int _passedWaypointsSinceTurn = 0;
        private int _lastWaypointIndexAfterTurn = -1;
        public int lookaheadWaypointCount = 40;        
        public float turnDecisionThreshold = 20f; 
        private int _collectedTurnCandidateIndex = -1;
        private PossibleTurn _collectedTurnCandidate = null;

        private void FixedUpdate()
        {
            if (Circuit == null || !Circuit.WayPoints.Any())
            {
                return;
            }

            Position = transform.position;

            int currentWaypointIndex = GetCurrentCircuitWaypointIndex();
            if (currentWaypointIndex != _lastWaypointIndexAfterTurn)
            {
                _passedWaypointsSinceTurn++;
                _lastWaypointIndexAfterTurn = currentWaypointIndex;
            }

            float segSpeed = GetSegmentSpeed(DistanceTravelled);
            float baseSpeed = segSpeed + (Speed * (HumanBehavior / 100f));

            bool redLightFound = CheckForRedLight(out float stopDist);

            bool carAhead = CheckForCarInFront(out float distanceToCar);

            float targetSpeed = baseSpeed;
            if (carAhead || redLightFound)
            {
                if (distanceToCar <= safeDistance)
                    targetSpeed = 0f;
                else
                    targetSpeed *= brakingMultiplier;
            }
            if (_isWaiting)
            {
                targetSpeed = 0f;
            }

            if (_pendingTurn)
            {
                targetSpeed *= 0.5f;
            }

            if (_currentVelocity < targetSpeed)
            {
                _currentVelocity = Mathf.MoveTowards(_currentVelocity, targetSpeed, accelForce * Time.deltaTime);
            }
            else
            {
                _currentVelocity = Mathf.MoveTowards(_currentVelocity, targetSpeed, brakeForce * Time.deltaTime);
            }

            float oldDist = DistanceTravelled;
            float newDist = oldDist + _currentVelocity * Time.deltaTime;
            if (redLightFound && stopDist >= 0f)
            {
                if (oldDist <= stopDist && newDist >= stopDist)
                {
                    newDist = stopDist;
                    _currentVelocity = 0f;
                }
            }
            DistanceTravelled = newDist;

            UpdateVehiclePositionAndRotation();

            EvaluatePossibleTurn();

            HandleWaiting();

            if (_pendingTurn)
            {
                float currentDistanceMod = DistanceTravelled % Circuit.Length;
                float targetWaypointDistance = _waypointDistances[_pendingTurnWaypointIndex];

                float diff = Mathf.Abs(currentDistanceMod - targetWaypointDistance);
                float distanceDiff = Mathf.Min(diff, Circuit.Length - diff);
                float threshold = 1f;

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
                        Debug.Log("Turn ausgeführt bei Wegpunkt " + _pendingTurnWaypointIndex + " – neuer Circuit gesetzt.");
                        turnCooldownCounter = 10;
                        _passedWaypointsSinceTurn = 0;
                    }
                    else
                    {
                        Debug.LogWarning("Kein passender Circuit für den Abbiege-Weg gefunden!");
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
        }

        private bool CheckForCarInFront(out float distanceToCar)
        {
            distanceToCar = Mathf.Infinity;
            float dynamicBrakingDistance = brakingDistance + 0.5f * _currentVelocity;
            Vector3 sphereCenter = transform.position + transform.up * 0.5f;
            Vector3 forwardDirection = UseStandardRotationForDetection ? _standardRotation * Vector3.forward : transform.forward;
            Vector3 rayStart = transform.position + transform.up * 0.5f;
            Debug.DrawRay(rayStart, forwardDirection * dynamicBrakingDistance, Color.red);
            float coneHalfAngle = 30f - _currentVelocity;

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

        private void UpdateVehiclePositionAndRotation()
        {
            RoutePoint routePoint = GetRoutePoint(DistanceTravelled);
            Vector3 direction = (Speed >= 0) ? routePoint.Direction : -routePoint.Direction;
            Vector3 right = Vector3.Cross(Vector3.up, direction).normalized;
            Vector3 horizontalOffset = right * Offset.x + direction * Offset.z;
            Vector3 basePosition = routePoint.Position + horizontalOffset;
            float swerveAmount = (HumanBehavior / 100f) * SwerveAmplitude;
            float swerve = Mathf.Sin(Time.time * SwerveFrequency) * swerveAmount;
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

        private RoutePoint GetRoutePoint(float dist)
        {
            RoutePoint rp = new RoutePoint();
            if (Circuit.closedLoop)
            {
                dist %= Circuit.Length;
                if (dist < 0) dist += Circuit.Length;
            }
            else
            {
                dist = Mathf.Clamp(dist, 0f, Circuit.Length);
            }
            int segIndex = 0;
            while (segIndex < _waypointDistances.Length - 1 && _waypointDistances[segIndex + 1] < dist)
            {
                segIndex++;
            }
            float segDist = dist - _waypointDistances[segIndex];
            float segLength = 0f;
            if (segIndex == _waypointDistances.Length - 1)
            {
                if (Circuit.closedLoop)
                {
                    segLength = Vector3.Distance(Circuit.WayPoints[segIndex].Position, Circuit.WayPoints[0].Position);
                }
            }
            else
            {
                segLength = Vector3.Distance(Circuit.WayPoints[segIndex].Position, Circuit.WayPoints[segIndex + 1].Position);
            }
            float t = (segLength > 0.0001f) ? segDist / segLength : 0f;
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

        private Vector3 GetWaypointPosition(int index)
        {
            int count = Circuit.WayPoints.Count;
            if (Circuit.closedLoop)
            {
                index = (index % count + count) % count;
            }
            else
            {
                if (index < 0) index = 0;
                if (index > count - 1) index = count - 1;
            }
            return Circuit.WayPoints[index].Position;
        }

        private void PrecomputeWaypointDistances()
        {
            if (Circuit == null || Circuit.WayPoints.Count < 2)
            {
                _waypointDistances = null;
                if (Circuit != null) Circuit.Length = 0f;
                return;
            }
            _waypointDistances = new float[Circuit.WayPoints.Count];
            float total = 0f;
            for (int i = 0; i < Circuit.WayPoints.Count; i++)
            {
                if (i == 0)
                {
                    _waypointDistances[i] = 0f;
                }
                else
                {
                    Vector3 prev = Circuit.WayPoints[i - 1].Position;
                    Vector3 current = Circuit.WayPoints[i].Position;
                    total += Vector3.Distance(prev, current);
                    _waypointDistances[i] = total;
                }
            }
            Circuit.Length = total;
        }

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

        private bool CheckForTurnSignalAhead(out IndicatorDirection upcomingTurn)
        {
            upcomingTurn = IndicatorDirection.Straight;
            if (Circuit == null || Circuit.WayPoints.Count == 0)
                return false;
            int currentIndex = GetCurrentCircuitWaypointIndex();
            for (int offset = 1; offset <= 3; offset++)
            {
                int index = Circuit.closedLoop
                    ? (currentIndex + offset) % Circuit.WayPoints.Count
                    : Mathf.Min(currentIndex + offset, Circuit.WayPoints.Count - 1);
                WayPoint wp = Circuit.WayPoints[index];
                if (wp.IndicatorOrientation != IndicatorDirection.Straight)
                {
                    upcomingTurn = wp.IndicatorOrientation;
                    return true;
                }
            }
            return false;
        }

        public WayPoint getClosestWayPoint()
        {
            var allWayPoints = new List<WayPoint>();
            WayPoint closestWayPoint = null;
            float closestDistance = 0;
            allWayPoints = wayPointManager.WayPoints;
            Position = transform.position;
            foreach (var wayPoint in allWayPoints)
            {
                if (closestWayPoint == null)
                {
                    closestWayPoint = wayPoint;
                    closestDistance = Vector3.Distance(wayPoint.Position, Position);
                }
                else if (closestDistance > Vector3.Distance(wayPoint.Position, Position))
                {
                    closestWayPoint = wayPoint;
                    closestDistance = Vector3.Distance(wayPoint.Position, Position);
                }
            }
            return closestWayPoint;
        }

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

        private void StartWaiting(float waitTime)
        {
            _isWaiting = true;
            _waitTimer = waitTime;
            Debug.Log($"Warte {waitTime} Sekunden an Distanz {DistanceTravelled}.");
        }

        public void SetStartPoint(float routeIndex)
        {
            if (Circuit == null)
            {
                wayPointManager = FindObjectsOfType<WayPointManager>().FirstOrDefault();
                if (wayPointManager != null && wayPointManager.Circuits != null && wayPointManager.Circuits.Count > 0)
                {
                    int randomCircuitIndex = Random.Range(0, wayPointManager.Circuits.Count);
                    Circuit = wayPointManager.Circuits[randomCircuitIndex];
                    Debug.Log("Kein Circuit übergeben, zufällig Circuit '" + Circuit.name + "' gewählt.");
                    PrecomputeWaypointDistances();
                }
                else
                {
                    Debug.LogWarning("Kein Circuit übergeben und WayPointManager enthält keine Circuits. Startpunkt wird auf 0 gesetzt.");
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
                    Debug.Log("Ungültiger Startindex. Zufällig ausgewählter Index: " + index);
                }
                DistanceTravelled = _waypointDistances[index];
                Debug.Log("Startpunkt auf Waypoint-Index " + index + " gesetzt. Distanz = " + DistanceTravelled);
            }
            else
            {
                Debug.LogWarning("Keine Wegpunkt-Distanzen vorhanden, Startpunkt wird auf 0 gesetzt.");
                DistanceTravelled = 0f;
            }
        }

        public void SetSpeed(float speed)
        {
            Speed = speed;
            _originalSpeed = speed;
        }

        public void SetHumanBehavior(float humanBehavior)
        {
            HumanBehavior = humanBehavior;
        }

        public void SetCircuit(WayPointCircuit circuit)
        {
            Circuit = circuit;
            PrecomputeWaypointDistances();
        }

        public void SetWaitPoints(List<WaitPoint> waitPoints)
        {
            WaitPoints = waitPoints;
        }

        public void SetOrientation(Quaternion eulerAngles)
        {
            StandardRotationEulerAngles = eulerAngles.eulerAngles;
            _standardRotation = eulerAngles;
        }

        public void SetOffset(Vector3 offset)
        {
            Offset = offset;
        }

        private float GetSegmentSpeed(float dist)
        {
            if (Circuit.closedLoop)
            {
                dist %= Circuit.Length;
                if (dist < 0) dist += Circuit.Length;
            }
            else
            {
                dist = Mathf.Clamp(dist, 0, Circuit.Length);
            }
            int segIndex = 0;
            while (segIndex < _waypointDistances.Length - 1 && _waypointDistances[segIndex + 1] < dist)
            {
                segIndex++;
            }
            float segDist = dist - _waypointDistances[segIndex];
            float segLength = 0f;
            if (segIndex == _waypointDistances.Length - 1)
            {
                if (Circuit.closedLoop)
                {
                    segLength = Vector3.Distance(Circuit.WayPoints[segIndex].Position, Circuit.WayPoints[0].Position);
                }
                else
                {
                    segLength = 0f;
                }
            }
            else
            {
                segLength = Vector3.Distance(Circuit.WayPoints[segIndex].Position, Circuit.WayPoints[segIndex + 1].Position);
            }
            float t = (segLength > 0.0001f) ? (segDist / segLength) : 0f;
            float speedA = Circuit.WayPoints[segIndex].Speed;
            int nextIndex = (segIndex + 1) % Circuit.WayPoints.Count;
            float speedB = Circuit.WayPoints[nextIndex].Speed;
            float segmentSpeed = Mathf.Lerp(speedA, speedB, t);
            return segmentSpeed;
        }

        public void SetRoadLayerByName(string layerName)
        {
            int layerIndex = LayerMask.NameToLayer(layerName);
            if (layerIndex == -1)
            {
                Debug.LogError($"Layer '{layerName}' existiert nicht. Bitte in Projektsettings anlegen.");
                return;
            }
            RoadLayer = 1 << layerIndex;
            Debug.Log($"RoadLayer auf '{layerName}' gesetzt (Index: {layerIndex}).");
        }

        public void EvaluatePossibleTurn()
        {

            if (_pendingTurn)
                return;


            if (_passedWaypointsSinceTurn < 5)
                return;

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

            List<(int candidateIndex, float candidateDiff, PossibleTurn candidateTurn)> candidates = new List<(int, float, PossibleTurn)>();

            for (int offset = 1; offset <= dynamicTurnDetectionDistance * 2; offset++)
            {
                int candidateIndex = (currentIndex + offset) % totalWaypoints;
                WayPoint candidateWaypoint = Circuit.WayPoints[candidateIndex];

                if (candidateWaypoint.PossibleTurns != null && candidateWaypoint.PossibleTurns.Count > 0)
                {
                    float waypointDistance = _waypointDistances[candidateIndex] % Circuit.Length;
                    float diff = (waypointDistance - currentDistance + Circuit.Length) % Circuit.Length;

                    PossibleTurn candidateTurn = candidateWaypoint.PossibleTurns[Random.Range(0, candidateWaypoint.PossibleTurns.Count)];
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
                    bestCandidate = candidates[Random.Range(0, candidates.Count)];
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

        private WayPointCircuit FindCircuitForWaypoint(WayPoint wp)
        {
            foreach (var circuit in wayPointManager.Circuits)
            {
                if (circuit.WayPoints.Contains(wp))
                {
                    return circuit;
                }
            }
            return null;
        }
    }
}
