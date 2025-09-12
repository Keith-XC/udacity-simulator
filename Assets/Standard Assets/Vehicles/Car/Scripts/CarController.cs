using System;
using System.Collections;
using System.Collections.Generic;
using System.IO;
using UnityEngine;
using UnityEngine.EventSystems;

namespace UnityStandardAssets.Vehicles.Car
{
    internal enum CarDriveType
    {
        FrontWheelDrive,
        RearWheelDrive,
        FourWheelDrive
    }

    internal enum SpeedType
    {
        MPH,
        KPH
    }

    public class CarController : MonoBehaviour
    {
        [SerializeField] private CarDriveType m_CarDriveType = CarDriveType.FourWheelDrive;
        [SerializeField] private WheelCollider[] m_WheelColliders = new WheelCollider[4];
        [SerializeField] private GameObject[] m_WheelMeshes = new GameObject[4];
        [SerializeField] private WheelEffects[] m_WheelEffects = new WheelEffects[4];
        [SerializeField] private Vector3 m_CentreOfMassOffset;
        [SerializeField] private float m_MaximumSteerAngle;
        [Range(0, 1)][SerializeField] private float m_SteerHelper;
        [Range(0, 1)][SerializeField] private float m_TractionControl;
        [SerializeField] private float m_FullTorqueOverAllWheels;
        [SerializeField] private float m_ReverseTorque;
        [SerializeField] private float m_MaxHandbrakeTorque;
        [SerializeField] private float m_Downforce = 100f;
        [SerializeField] private SpeedType m_SpeedType;
        [SerializeField] private float m_Topspeed = 200;
        [SerializeField] private static int NoOfGears = 5;
        [SerializeField] private float m_RevRangeBoundary = 1f;
        [SerializeField] private float m_SlipLimit;
        [SerializeField] private float m_BrakeTorque;

        public List<BrakeLight> BrakingLights;
        public const string CSVFileName = "driving_log.csv";
        public const string DirFrames = "IMG";

        [SerializeField] private Camera CenterCamera;
        [SerializeField] private Camera LeftCamera;
        [SerializeField] private Camera RightCamera;

        private Quaternion[] m_WheelMeshLocalRotations;
        private Vector3 m_Prevpos, m_Pos;
        private float m_SteerAngle;
        private int m_GearNum;
        private float m_GearFactor;
        private float m_OldRotation;
        private float m_CurrentTorque;
        private Rigidbody m_Rigidbody;
        private const float k_ReversingThreshold = 0.01f;
        private string m_saveLocation = "";
        private Queue<CarSample> carSamples;
        private int TotalSamples;
        private bool isSaving;
        private Vector3 saved_position;
        private Quaternion saved_rotation;

        private string lastCollision = "none";
        private string endOfLineColliderTarget = "end_line_collider";

        public bool Skidding { get; private set; }
        public float BrakeInput { get; private set; }
        private bool m_isRecording = false;
        public bool IsRecording
        {
            get { return m_isRecording; }
            set
            {
                m_isRecording = value;
                if (value)
                {
                    Debug.Log("Starting to record");
                    carSamples = new Queue<CarSample>();
                    StartCoroutine(Sample());
                }
                else
                {
                    Debug.Log("Stopping record");
                    StopCoroutine(Sample());
                    Debug.Log("Writing to disk");
                    saved_position = transform.position;
                    saved_rotation = transform.rotation;
                    TotalSamples = carSamples != null ? carSamples.Count : 0;
                    isSaving = true;
                    StartCoroutine(WriteSamplesToDisk());
                }
            }
        }

        /// <summary>
        /// Checks if a save location is set; if not, opens a dialog to select one.
        /// </summary>
        public bool checkSaveLocation()
        {
            if (!string.IsNullOrEmpty(m_saveLocation))
            {
                return true;
            }
            else
            {
                SimpleFileBrowser.ShowSaveDialog(OpenFolder, null, true, null, "Select Output Folder", "Select");
            }
            return false;
        }

        public float CurrentSteerAngle
        {
            get { return m_SteerAngle; }
            set { m_SteerAngle = value; }
        }

        public float CurrentSpeed { get { return m_Rigidbody != null ? m_Rigidbody.linearVelocity.magnitude : 0f; } }
        public float MaxSpeed { get { return m_Topspeed; } }
        public float Revs { get; private set; }
        public float AccelInput { get; set; }

        /// <summary>
        /// Initializes wheel meshes, sets the center of mass, and initializes torque.
        /// </summary>
        private void Start()
        {
            if (m_WheelMeshes != null && m_WheelMeshes.Length > 0)
            {
                m_WheelMeshLocalRotations = new Quaternion[m_WheelMeshes.Length];
                for (int i = 0; i < m_WheelMeshes.Length; i++)
                {
                    if (m_WheelMeshes[i] != null)
                        m_WheelMeshLocalRotations[i] = m_WheelMeshes[i].transform.localRotation;
                }
            }
            if (m_WheelColliders != null && m_WheelColliders.Length > 0 && m_WheelColliders[0] != null && m_WheelColliders[0].attachedRigidbody != null)
            {
                m_WheelColliders[0].attachedRigidbody.centerOfMass = m_CentreOfMassOffset;
            }
            m_MaxHandbrakeTorque = float.MaxValue;
            m_Rigidbody = GetComponent<Rigidbody>();
            if (m_Rigidbody == null)
            {
                Debug.LogWarning("No Rigidbody found on the Car GameObject.");
            }
            m_CurrentTorque = m_FullTorqueOverAllWheels - (m_TractionControl * m_FullTorqueOverAllWheels);
        }

        /// <summary>
        /// Stops the car by zeroing its velocity.
        /// </summary>
        public void Stop()
        {
            if (m_Rigidbody != null)
                m_Rigidbody.linearVelocity = Vector3.zero;
        }

        /// <summary>
        /// Adjusts the gear based on current speed.
        /// </summary>
        private void GearChanging()
        {
            float f = Mathf.Abs(CurrentSpeed / MaxSpeed);
            float upgearlimit = (1f / NoOfGears) * (m_GearNum + 1);
            float downgearlimit = (1f / NoOfGears) * m_GearNum;
            if (m_GearNum > 0 && f < downgearlimit)
                m_GearNum--;
            if (f > upgearlimit && (m_GearNum < (NoOfGears - 1)))
                m_GearNum++;
        }

        private static float CurveFactor(float factor)
        {
            return 1 - (1 - factor) * (1 - factor);
        }

        private static float ULerp(float from, float to, float value)
        {
            return (1.0f - value) * from + value * to;
        }

        /// <summary>
        /// Calculates the gear factor based on current speed.
        /// </summary>
        private void CalculateGearFactor()
        {
            float f = 1f / NoOfGears;
            var targetGearFactor = Mathf.InverseLerp(f * m_GearNum, f * (m_GearNum + 1), Mathf.Abs(CurrentSpeed / MaxSpeed));
            m_GearFactor = Mathf.Lerp(m_GearFactor, targetGearFactor, Time.deltaTime * 5f);
        }

        /// <summary>
        /// Calculates engine revolutions based on gear factor.
        /// </summary>
        private void CalculateRevs()
        {
            CalculateGearFactor();
            float gearNumFactor = m_GearNum / (float)NoOfGears;
            float revsRangeMin = ULerp(0f, m_RevRangeBoundary, CurveFactor(gearNumFactor));
            float revsRangeMax = ULerp(m_RevRangeBoundary, 1f, gearNumFactor);
            Revs = ULerp(revsRangeMin, revsRangeMax, m_GearFactor);
        }

        /// <summary>
        /// Sets the Rigidbody's position and rotation.
        /// </summary>
        public void Set(Vector3 pos, Quaternion rot)
        {
            if (m_Rigidbody != null)
            {
                m_Rigidbody.position = pos;
                m_Rigidbody.rotation = rot;
            }
        }

        /// <summary>
        /// Repeatedly sets the Rigidbody's position and rotation over a number of iterations.
        /// </summary>
        IEnumerator KeepSetting(Vector3 pos, Quaternion rot, int numIter)
        {
            while (numIter > 0)
            {
                if (m_Rigidbody != null)
                    m_Rigidbody.isKinematic = true;
                yield return new WaitForFixedUpdate();
                if (m_Rigidbody != null)
                {
                    m_Rigidbody.position = pos;
                    m_Rigidbody.rotation = rot;
                }
                transform.position = pos;
                transform.rotation = rot;
                numIter--;
            }
            if (m_Rigidbody != null)
                m_Rigidbody.isKinematic = false;
        }

        /// <summary>
        /// Updates the controller, processing recording if active.
        /// </summary>
        public void Update()
        {
            if (IsRecording)
            {
                // Recording logic can be added here.
            }
        }

        /// <summary>
        /// Processes car movement based on input parameters.
        /// </summary>
        public void Move(float steering, float accel, float footbrake, float handbrake)
        {
            if (m_WheelColliders != null && m_WheelMeshes != null)
            {
                int count = Mathf.Min(m_WheelColliders.Length, m_WheelMeshes.Length);
                for (int i = 0; i < count; i++)
                {
                    if (m_WheelColliders[i] != null && m_WheelMeshes[i] != null)
                    {
                        Quaternion quat;
                        Vector3 position;
                        m_WheelColliders[i].GetWorldPose(out position, out quat);
                        m_WheelMeshes[i].transform.position = position;
                        m_WheelMeshes[i].transform.rotation = quat;
                    }
                }
            }
            steering = Mathf.Clamp(steering, -1, 1);
            AccelInput = accel = Mathf.Clamp(accel, 0, 1);
            BrakeInput = footbrake = -1 * Mathf.Clamp(footbrake, -1, 0);
            handbrake = Mathf.Clamp(handbrake, 0, 1);
            if (footbrake == 0f)
            {
                for (int i = 0; i < m_WheelColliders.Length; i++)
                {
                    if (m_WheelColliders[i] != null)
                    {
                        m_WheelColliders[i].brakeTorque = 0f;
                    }
                }
            }
            if (accel > 0.9f && footbrake == 0f && CurrentSpeed < 1f)
            {
                m_CurrentTorque = m_FullTorqueOverAllWheels;
            }
            if (m_WheelColliders != null && m_WheelColliders.Length >= 2)
            {
                m_SteerAngle = steering * m_MaximumSteerAngle;
                if (m_WheelColliders[0] != null)
                    m_WheelColliders[0].steerAngle = m_SteerAngle;
                if (m_WheelColliders[1] != null)
                    m_WheelColliders[1].steerAngle = m_SteerAngle;
            }
            SteerHelper();
            ApplyDrive(accel, footbrake);
            CapSpeed();
            if (handbrake > 0f && m_WheelColliders != null && m_WheelColliders.Length >= 4)
            {
                float hbTorque = handbrake * m_MaxHandbrakeTorque;
                if (m_WheelColliders[2] != null)
                    m_WheelColliders[2].brakeTorque = hbTorque;
                if (m_WheelColliders[3] != null)
                    m_WheelColliders[3].brakeTorque = hbTorque;
            }
            CalculateRevs();
            GearChanging();
            AddDownForce();
            CheckForWheelSpin();
            TractionControl();
        }

        /// <summary>
        /// Limits the car's speed based on the top speed and speed type.
        /// </summary>
        private void CapSpeed()
        {
            if (m_Rigidbody == null)
                return;
            float speed = m_Rigidbody.linearVelocity.magnitude;
            switch (m_SpeedType)
            {
                case SpeedType.MPH:
                    speed *= 2.23693629f;
                    if (speed > m_Topspeed)
                        m_Rigidbody.linearVelocity = (m_Topspeed / 2.23693629f) * m_Rigidbody.linearVelocity.normalized;
                    break;
                case SpeedType.KPH:
                    speed *= 3.6f;
                    if (speed > m_Topspeed)
                        m_Rigidbody.linearVelocity = (m_Topspeed / 3.6f) * m_Rigidbody.linearVelocity.normalized;
                    break;
            }
        }

        /// <summary>
        /// Applies drive torque and brake forces based on input.
        /// </summary>
        private void ApplyDrive(float accel, float footbrake)
        {
            float thrustTorque = 0f;
            if (m_WheelColliders == null)
                return;
            switch (m_CarDriveType)
            {
                case CarDriveType.FourWheelDrive:
                    thrustTorque = accel * (m_CurrentTorque / 4f);
                    for (int i = 0; i < m_WheelColliders.Length; i++)
                    {
                        if (m_WheelColliders[i] != null)
                            m_WheelColliders[i].motorTorque = thrustTorque;
                    }
                    break;
                case CarDriveType.FrontWheelDrive:
                    thrustTorque = accel * (m_CurrentTorque / 2f);
                    if (m_WheelColliders.Length >= 2)
                    {
                        if (m_WheelColliders[0] != null)
                            m_WheelColliders[0].motorTorque = thrustTorque;
                        if (m_WheelColliders[1] != null)
                            m_WheelColliders[1].motorTorque = thrustTorque;
                    }
                    break;
                case CarDriveType.RearWheelDrive:
                    thrustTorque = accel * (m_CurrentTorque / 2f);
                    if (m_WheelColliders.Length >= 4)
                    {
                        if (m_WheelColliders[2] != null)
                            m_WheelColliders[2].motorTorque = thrustTorque;
                        if (m_WheelColliders[3] != null)
                            m_WheelColliders[3].motorTorque = thrustTorque;
                    }
                    break;
            }
            for (int i = 0; i < m_WheelColliders.Length; i++)
            {
                if (m_WheelColliders[i] == null)
                    continue;
                if (CurrentSpeed > 5 && Vector3.Angle(transform.forward, (m_Rigidbody != null ? m_Rigidbody.linearVelocity : Vector3.zero)) < 50f)
                {
                    m_WheelColliders[i].brakeTorque = m_BrakeTorque * footbrake;
                }
                else if (footbrake > 0)
                {
                    if (CurrentSpeed < 1f)
                    {
                        m_WheelColliders[i].brakeTorque = m_BrakeTorque * footbrake;
                        m_WheelColliders[i].motorTorque = 0f;
                    }
                    else
                    {
                        m_WheelColliders[i].brakeTorque = 0f;
                        m_WheelColliders[i].motorTorque = -m_ReverseTorque * footbrake;
                    }
                }
            }
            if (BrakeInput < 0)
            {
                BrakingLights.ForEach(x => x.ActivateBraking());
            }
            else
            {
                BrakingLights.ForEach(x => x.DeactivateBraking());
            }
        }

        /// <summary>
        /// Adjusts the car's velocity vector based on its steering to help stabilize the vehicle.
        /// </summary>
        private void SteerHelper()
        {
            if (m_WheelColliders == null)
                return;
            for (int i = 0; i < m_WheelColliders.Length; i++)
            {
                if (m_WheelColliders[i] == null)
                    continue;
                WheelHit wheelhit;
                m_WheelColliders[i].GetGroundHit(out wheelhit);
                if (wheelhit.normal == Vector3.zero)
                    return;
            }
            if (Mathf.Abs(m_OldRotation - transform.eulerAngles.y) < 10f)
            {
                float turnadjust = (transform.eulerAngles.y - m_OldRotation) * m_SteerHelper;
                Quaternion velRotation = Quaternion.AngleAxis(turnadjust, Vector3.up);
                if (m_Rigidbody != null)
                    m_Rigidbody.linearVelocity = velRotation * m_Rigidbody.linearVelocity;
            }
            m_OldRotation = transform.eulerAngles.y;
        }

        /// <summary>
        /// Adds downward force proportional to the vehicle's speed.
        /// </summary>
        private void AddDownForce()
        {
            if (m_WheelColliders != null && m_WheelColliders.Length > 0 && m_WheelColliders[0] != null && m_WheelColliders[0].attachedRigidbody != null)
            {
                m_WheelColliders[0].attachedRigidbody.AddForce(-transform.up * m_Downforce * m_WheelColliders[0].attachedRigidbody.linearVelocity.magnitude);
            }
        }

        /// <summary>
        /// Checks each wheel for slip and triggers effects if necessary.
        /// </summary>
        private void CheckForWheelSpin()
        {
            if (m_WheelEffects == null || m_WheelColliders == null)
                return;
            int count = Mathf.Min(m_WheelEffects.Length, m_WheelColliders.Length);
            for (int i = 0; i < count; i++)
            {
                if (m_WheelColliders[i] == null)
                    continue;
                WheelHit wheelHit;
                m_WheelColliders[i].GetGroundHit(out wheelHit);
                if (Mathf.Abs(wheelHit.forwardSlip) >= m_SlipLimit || Mathf.Abs(wheelHit.sidewaysSlip) >= m_SlipLimit)
                {
                    if (m_WheelEffects[i] != null)
                        m_WheelEffects[i].EmitTyreSmoke();
                    continue;
                }
                if (m_WheelEffects[i] != null)
                {
                    if (m_WheelEffects[i].PlayingAudio)
                        m_WheelEffects[i].StopAudio();
                    m_WheelEffects[i].EndSkidTrail();
                }
            }
        }

        /// <summary>
        /// Applies traction control adjustments based on wheel slip.
        /// </summary>
        private void TractionControl()
        {
            if (m_WheelColliders == null)
                return;
            WheelHit wheelHit;
            switch (m_CarDriveType)
            {
                case CarDriveType.FourWheelDrive:
                    for (int i = 0; i < m_WheelColliders.Length; i++)
                    {
                        if (m_WheelColliders[i] != null)
                        {
                            m_WheelColliders[i].GetGroundHit(out wheelHit);
                            AdjustTorque(wheelHit.forwardSlip);
                        }
                    }
                    break;
                case CarDriveType.RearWheelDrive:
                    if (m_WheelColliders.Length > 2 && m_WheelColliders[2] != null)
                    {
                        m_WheelColliders[2].GetGroundHit(out wheelHit);
                        AdjustTorque(wheelHit.forwardSlip);
                    }
                    if (m_WheelColliders.Length > 3 && m_WheelColliders[3] != null)
                    {
                        m_WheelColliders[3].GetGroundHit(out wheelHit);
                        AdjustTorque(wheelHit.forwardSlip);
                    }
                    break;
                case CarDriveType.FrontWheelDrive:
                    if (m_WheelColliders.Length > 0 && m_WheelColliders[0] != null)
                    {
                        m_WheelColliders[0].GetGroundHit(out wheelHit);
                        AdjustTorque(wheelHit.forwardSlip);
                    }
                    if (m_WheelColliders.Length > 1 && m_WheelColliders[1] != null)
                    {
                        m_WheelColliders[1].GetGroundHit(out wheelHit);
                        AdjustTorque(wheelHit.forwardSlip);
                    }
                    break;
            }
        }

        /// <summary>
        /// Adjusts the current torque based on the amount of forward slip.
        /// </summary>
        private void AdjustTorque(float forwardSlip)
        {
            if (forwardSlip >= m_SlipLimit && m_CurrentTorque >= 0)
            {
                m_CurrentTorque -= 10 * m_TractionControl;
            }
            else
            {
                m_CurrentTorque += 10 * m_TractionControl;
                if (m_CurrentTorque > m_FullTorqueOverAllWheels)
                {
                    m_CurrentTorque = m_FullTorqueOverAllWheels;
                }
            }
        }

        /// <summary>
        /// Writes recorded car samples to disk.
        /// </summary>
        public IEnumerator WriteSamplesToDisk()
        {
            yield return new WaitForSeconds(0.000f);
            if (carSamples != null && carSamples.Count > 0)
            {
                CarSample sample = carSamples.Dequeue();
                transform.position = sample.position;
                transform.rotation = sample.rotation;
                string centerPath = WriteImage(CenterCamera, "center", sample.timeStamp);
                string leftPath = WriteImage(LeftCamera, "left", sample.timeStamp);
                string rightPath = WriteImage(RightCamera, "right", sample.timeStamp);
                string row = string.Format("{0},{1},{2},{3},{4},{5},{6},{7},{8},{9}\n",
                                             centerPath, leftPath, rightPath, sample.steeringAngle, sample.throttle, sample.brake, sample.speed, sample.lap, sample.sector, sample.cte);
                File.AppendAllText(Path.Combine(m_saveLocation, CSVFileName), row);
            }
            if (carSamples != null && carSamples.Count > 0)
            {
                StartCoroutine(WriteSamplesToDisk());
            }
            else
            {
                StopCoroutine(WriteSamplesToDisk());
                isSaving = false;
                transform.position = saved_position;
                transform.rotation = saved_rotation;
                if (m_Rigidbody != null)
                {
                    m_Rigidbody.linearVelocity = new Vector3(0f, -10f, 0f);
                }
                Move(0f, 0f, 0f, 0f);
            }
        }

        /// <summary>
        /// Returns the percentage of samples saved.
        /// </summary>
        public float getSavePercent()
        {
            return (float)(TotalSamples - (carSamples != null ? carSamples.Count : 0)) / TotalSamples;
        }

        /// <summary>
        /// Returns the current saving status.
        /// </summary>
        public bool getSaveStatus()
        {
            return isSaving;
        }

        /// <summary>
        /// Records a car sample at fixed intervals.
        /// </summary>
        public IEnumerator Sample()
        {
            yield return new WaitForSeconds(0.06666667f);
            WaypointTracker tracker = new WaypointTracker();
            WayPointManager manager = GetComponent<WayPointManager>();
            if (!string.IsNullOrEmpty(m_saveLocation))
            {
                CarSample sample = new CarSample();
                sample.timeStamp = DateTime.Now.ToString("yyyy_MM_dd_HH_mm_ss_fff");
                sample.steeringAngle = m_SteerAngle / m_MaximumSteerAngle;
                sample.throttle = AccelInput;
                sample.brake = BrakeInput;
                sample.speed = CurrentSpeed;
                sample.position = transform.position;
                sample.rotation = transform.rotation;
                sample.cte = tracker != null ? tracker.CrossTrackError(this) : 0f;
                sample.lap = manager != null ? manager.getLapNumber() : 0;
                sample.sector = manager != null ? manager.getCurrentWayPointNumber() : 0;
                if (carSamples != null)
                    carSamples.Enqueue(sample);
            }
            if (IsRecording)
            {
                StartCoroutine(Sample());
            }
        }

        /// <summary>
        /// Opens a folder dialog to set the save location and creates the necessary directory.
        /// </summary>
        private void OpenFolder(string location)
        {
            m_saveLocation = location;
            Directory.CreateDirectory(Path.Combine(m_saveLocation, DirFrames));
        }

        /// <summary>
        /// Captures an image from the specified camera and writes it to disk.
        /// </summary>
        private string WriteImage(Camera camera, string prepend, string timestamp)
        {
            if (camera == null)
                return "";
            camera.Render();
            RenderTexture targetTexture = camera.targetTexture;
            if (targetTexture == null)
                return "";
            RenderTexture.active = targetTexture;
            Texture2D texture2D = new Texture2D(targetTexture.width, targetTexture.height, TextureFormat.RGB24, false, false);
            texture2D.ReadPixels(new Rect(0, 0, targetTexture.width, targetTexture.height), 0, 0);
            texture2D.Apply();
            byte[] image = texture2D.EncodeToPNG();
            UnityEngine.Object.DestroyImmediate(texture2D);
            string directory = Path.Combine(m_saveLocation, DirFrames);
            string path = Path.Combine(directory, prepend + "_" + timestamp + ".png");
            File.WriteAllBytes(path, image);
            return path;
        }

        /// <summary>
        /// Returns the name of the last collision.
        /// </summary>
        public string GetLastCollision()
        {
            return lastCollision;
        }

        /// <summary>
        /// Clears the record of the last collision.
        /// </summary>
        public void ClearLastCollision()
        {
            lastCollision = "none";
        }

        private void OnCollisionEnter(Collision col)
        {
        }

        private void OnTriggerEnter(Collider col)
        {
            if (col != null && col.gameObject != null && col.gameObject.name == endOfLineColliderTarget)
            {
                Debug.Log("OnTriggerEnter Car Collision between " + gameObject.name + " and " + col.gameObject.name);
                lastCollision = col.gameObject.name;
            }
        }

        /// <summary>
        /// Sets the top speed of the car.
        /// </summary>
        public void SetTopSpeed(int speed)
        {
            m_Topspeed = speed;
        }

        public void SetCarPositionAndRotation(Vector3 pos, Quaternion rot)
        {
            Set(pos, rot);

            StartCoroutine(UpdateWheelMeshesAfterFixedUpdate());
        }

        private IEnumerator UpdateWheelMeshesAfterFixedUpdate()
        {
            yield return new WaitForFixedUpdate();

            if (m_WheelColliders != null && m_WheelMeshes != null)
            {
                int count = Mathf.Min(m_WheelColliders.Length, m_WheelMeshes.Length);
                for (int i = 0; i < count; i++)
                {
                    if (m_WheelColliders[i] != null && m_WheelMeshes[i] != null)
                    {
                        Vector3 wheelPos;
                        Quaternion wheelRot;
                        m_WheelColliders[i].GetWorldPose(out wheelPos, out wheelRot);
                        m_WheelMeshes[i].transform.position = wheelPos;
                        m_WheelMeshes[i].transform.rotation = wheelRot;
                    }
                }
            }
        }


    }

    internal class CarSample
    {
        public Quaternion rotation;
        public Vector3 position;
        public float steeringAngle;
        public float throttle;
        public float brake;
        public float speed;
        public string timeStamp;
        public float cte;
        public int lap;
        public int sector;
    }
}
