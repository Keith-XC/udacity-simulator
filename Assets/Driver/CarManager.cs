using UnityEngine;
using System;
using System.Collections.Generic;
using System.Linq;
using Assets._1_SelfDrivingCar.Scripts;
using UnityStandardAssets.Vehicles.Car;
using Assets.TcpServerManager;
using Assets.WayPointSystem2;
using Assets.Driver;
using System.Collections;

public class CarManager : MonoBehaviour
{
    private Dictionary<int, CarInfo> carDictionary = new Dictionary<int, CarInfo>();
    private TcpServerManager _tcpServerManager;
    private long lastTimeTelemetryUpdated;

    // Index of the currently displayed camera
    private int currentCameraIndex = 0;

    // Reference to the currently active car (the one being controlled)
    private CarInfo currentActiveCar;

    /// <summary>
    /// Initializes telemetry update time.
    /// </summary>
    private void Awake()
    {
        lastTimeTelemetryUpdated = -1;
    }

    /// <summary>
    /// Gets the TcpServerManager instance, connects car controllers, and updates the camera view.
    /// </summary>
    private IEnumerator Start()
    {
        // listen to spawn events to register new cars
        Spawner.OnCarSpawned += OnCarSpawned;
    

        _tcpServerManager = TcpServerManager.Instance;

        // 等待场景完全加载
        yield return new WaitForSeconds(0.1f);
        
        // 等待所有车辆完成初始化
        yield return new WaitUntil(() => GameObject.FindGameObjectsWithTag("Car").Length > 0);
        
        ConnectCarControllers();
        yield return new WaitForSeconds(0.1f);

        InitializeEgoCarView();
    }

    private void InitializeEgoCarView()
    {
        // try to find the ego car (CarId = 1)
        if (carDictionary.TryGetValue(1, out CarInfo egoCar) && egoCar.mainCamera != null)
        {   
            // 首先检查是否有任何车辆注册
            if (carDictionary.Count == 0)
            {
                Debug.Log("Waiting for cars to be registered...");
                return;
            }

            // Get list of cars that have a camera
            List<CarInfo> carsWithCamera = carDictionary.Values
                .Where(info => info.mainCamera != null)
                .ToList();

            // Set the current camera index to the ego car's index
            currentCameraIndex = carsWithCamera.IndexOf(egoCar);
            
            if (currentCameraIndex == -1)
            {
                Debug.LogError("Failed to find ego car in cars with camera list");
                currentCameraIndex = 0;
            }
        }
        else
        {
            Debug.LogWarning("Ego car (ID=1) not found or has no camera, using first available camera");
            currentCameraIndex = 0;
        }

        UpdateCameraView();
}

    private void OnDestroy()
    {
        // unsubscribe to avoid memory leaks
        Spawner.OnCarSpawned -= OnCarSpawned;
    }

    // Called when a new car is spawned in the scene
    private void OnCarSpawned()
    {
        Debug.Log("CarManager: Detected new car spawned, updating controllers...");
        ConnectCarControllers();
        UpdateCameraView();
    }

    /// <summary>
    /// Connects new car controllers, updates telemetry, and handles camera cycling via arrow keys.
    /// </summary>
    private void Update()
    {

        UpdateTelemetry();
        // ConnectCarControllers();

        if (Input.GetKeyDown(KeyCode.RightArrow))
        {
            CycleCamera(1);
        }
        else if (Input.GetKeyDown(KeyCode.LeftArrow))
        {
            CycleCamera(-1);
        }
    }

    /// <summary>
    /// Moves the currently active car (for manual or remote control) and drives autonomous vehicles.
    /// </summary>
    private void FixedUpdate()
    {
        if (currentActiveCar != null && currentActiveCar.carController != null)
        {
            if (Input.GetKey(KeyCode.W) || Input.GetKey(KeyCode.A) ||
                Input.GetKey(KeyCode.S) || Input.GetKey(KeyCode.D))
            {
                currentActiveCar.moveAlongCircuit.enabled = false;
                currentActiveCar.steering.UpdateValues();
                currentActiveCar.carController.Move(
                    currentActiveCar.steering.H,
                    currentActiveCar.steering.V,
                    currentActiveCar.steering.V,
                    0f
                );
            }
            else
            {
                if (currentActiveCar != null && currentActiveCar.moveAlongCircuit != null)
                {
                    currentActiveCar.moveAlongCircuit.enabled = true;
                }
                Debug.Log($"0000 Car ID: {currentActiveCar.CarId}, Steering Angle: {currentActiveCar.RemoteSteeringAngle}, Acceleration: {currentActiveCar.RemoteAcceleration}");
                currentActiveCar.carController.Move(
                    currentActiveCar.RemoteSteeringAngle,
                    currentActiveCar.RemoteAcceleration,
                    currentActiveCar.RemoteAcceleration,
                    0f
                );

            }
        }

        foreach (var kvp in carDictionary)
        {
            var carInfo = kvp.Value;
            if (carInfo.autonomous && carInfo.carController != null && carInfo != currentActiveCar)
            {
                carInfo.carController.Move(
                    carInfo.RemoteSteeringAngle,
                    carInfo.RemoteAcceleration,
                    carInfo.RemoteAcceleration,
                    0f
                );
            }
        }
    }


    /// <summary>
    /// Registers all cars in the scene based on the "Car" tag and the CarIdentifier component.
    /// </summary>
    private void ConnectCarControllers()
    {
        GameObject[] carObjects = GameObject.FindGameObjectsWithTag("Car");
        foreach (GameObject car in carObjects)
        {
            CarIdentifier identifier = car.GetComponent<CarIdentifier>();
            if (identifier != null)
            {
                int id = identifier.CarId;
                if (!carDictionary.ContainsKey(id))
                {
                    CarInfo info = new CarInfo
                    {
                        CarId = id,
                        carController = car.GetComponent<CarController>(),
                        wayPointTracker = car.GetComponent<WayPointTracker>()
                    };

                    Transform ffcTransform = car.transform.Find("Front Facing Camera");
                    if (ffcTransform != null)
                    {
                        info.frontFacingCamera = ffcTransform.GetComponent<Camera>();
                    }
                    Transform mcTransform = car.transform.Find("Main Camera");
                    if (mcTransform != null)
                    {
                        info.mainCamera = mcTransform.GetComponent<Camera>();
                    }

                    info.steering = new Steering();
                    info.steering.Start(); // TODO: check this out

                    info.RemoteSteeringAngle = 0f;
                    info.RemoteAcceleration = 0.7f;
                    info.autonomous = identifier.autonomous;
                    info.moveAlongCircuit = car.GetComponent<MoveAlongCircuitWithCarControl>();
                    info.currentTelemetry = new CarTelemetry();

                    carDictionary.Add(id, info);
                    Debug.Log("Car registered: CarId " + id);
                }
            }
        }
    }

    /// <summary>
    /// Called by the TcpServerManager to set car actions based on the received CommandData.
    /// </summary>
    /// <param name="commandData">Contains fields such as carId, Indicator, steering_angle, and throttle.</param>
    public void SetCarAction(CommandData commandData)
    {
        var carControll = commandData.carControll;
        int carId = carControll.carId;

        if (carDictionary.TryGetValue(carId, out CarInfo carInfo))
        {
            currentActiveCar = carInfo; // <<< make sure we drive THIS car

            carInfo.RemoteSteeringAngle = carControll.steering_angle;
            carInfo.RemoteAcceleration = carControll.throttle;

            if (carInfo.moveAlongCircuit != null)
            {
                if (carControll.Indicator == IndicatorDirection.Left)
                    carInfo.moveAlongCircuit.indicatorManager.ActivateLeft();
                else if (carControll.Indicator == IndicatorDirection.Right)
                    carInfo.moveAlongCircuit.indicatorManager.ActivateRight();
                else
                    carInfo.moveAlongCircuit.indicatorManager.DeactivateAll();
            }
        }
    }

    /// <summary>
    /// Updates telemetry for each car and broadcasts the data via TcpServerManager.
    /// </summary>
    private void UpdateTelemetry()
    {
        long currentTime = DateTimeOffset.Now.ToUnixTimeMilliseconds();
        if (currentTime - lastTimeTelemetryUpdated < 10)
            return;

        foreach (var kvp in carDictionary)
        {
            CarInfo carInfo = kvp.Value;
            if (carInfo.carController == null || carInfo.frontFacingCamera == null || !carInfo.autonomous)
                continue;

            CarTelemetry telemetry = new CarTelemetry
            {
                carId = carInfo.CarId,
                timestamp = DateTimeOffset.Now.ToUnixTimeMilliseconds().ToString(),
                image = Convert.ToBase64String(CameraHelper.CaptureFrame(carInfo.frontFacingCamera)),
                pos_x = carInfo.carController.transform.position.x,
                pos_y = carInfo.carController.transform.position.y,
                pos_z = carInfo.carController.transform.position.z,
                steering_angle = carInfo.carController.CurrentSteerAngle,
                throttle = carInfo.carController.AccelInput,
                speed = carInfo.carController.CurrentSpeed
            };

            if (carInfo.wayPointTracker != null)
            {
                var point = carInfo.wayPointTracker.getClosestWayPoint();
                if (point != null)
                {
                    telemetry.sector = point.Number;

                    // compute angular difference 
                    Vector3 toNext = (point.Position - carInfo.carController.transform.position).normalized;
                    Vector3 forward = carInfo.carController.transform.forward;
                    float angular_difference = Vector3.SignedAngle(forward, toNext, Vector3.up);
                    telemetry.angular_difference = angular_difference;
                }
                else
                {
                    telemetry.angular_difference = 0f;
                }
            }
            else
            {
                telemetry.angular_difference = 0f;
            }

            carInfo.currentTelemetry = telemetry;
            _tcpServerManager.BroadcastTelemetry(telemetry);
        }
        lastTimeTelemetryUpdated = currentTime;
    }

    /// <summary>
    /// Updates the camera view by enabling the main camera of the selected car.
    /// </summary>
    private void UpdateCameraView()
    {
        // Erstelle eine Liste aller Fahrzeuge mit einer MainCamera und deaktiviere diese
        List<CarInfo> carsWithCamera = new List<CarInfo>();
        foreach (var info in carDictionary.Values)
        {
            if (info.mainCamera != null)
            {
                carsWithCamera.Add(info);
                info.mainCamera.gameObject.SetActive(false);

                var audioListener = info.mainCamera.GetComponent<AudioListener>();
                if (audioListener != null)
                {
                    audioListener.enabled = false;
                }
            }
        }
        if (carsWithCamera.Count == 0)
        {
            Debug.LogWarning("No vehicles with MainCamera found.");
            return;
        }

        // Stelle sicher, dass der currentCameraIndex im g�ltigen Bereich liegt
        currentCameraIndex = Mathf.Clamp(currentCameraIndex, 0, carsWithCamera.Count - 1);
        CarInfo selectedCar = carsWithCamera[currentCameraIndex];

        // Aktiviere die Kamera und den zugeh�rigen AudioListener
        selectedCar.mainCamera.gameObject.SetActive(true);
        AudioListener selectedAudio = selectedCar.mainCamera.GetComponent<AudioListener>();
        if (selectedAudio != null)
            selectedAudio.enabled = true;

        currentActiveCar = selectedCar;
        Debug.Log("Active camera switched: CarId " + currentActiveCar.CarId);
    }



    /// <summary>
    /// Cycles through available cameras based on the provided direction.
    /// </summary>
    /// <param name="direction">The direction to cycle (1 for next, -1 for previous).</param>
    private void CycleCamera(int direction)
    {
        List<CarInfo> carsWithCamera = new List<CarInfo>();
        foreach (var info in carDictionary.Values)
        {
            if (info.mainCamera != null)
            {
                carsWithCamera.Add(info);
            }
        }
        if (carsWithCamera.Count == 0)
            return;

        currentCameraIndex = (currentCameraIndex + direction + carsWithCamera.Count) % carsWithCamera.Count;
        UpdateCameraView();
    }


    /// <summary>
    /// Gets the active car info.
    /// </summary>
    public CarInfo ActiveCarInfo
    {
        get { return currentActiveCar != null ? currentActiveCar : null; }
    }

    /// <summary>
    /// Helper class that bundles all relevant components and parameters of a vehicle.
    /// </summary>
    public class CarInfo
    {
        public int CarId;
        public bool autonomous;
        public CarController carController;
        public WayPointTracker wayPointTracker;
        public Camera frontFacingCamera;
        public Camera mainCamera;
        public Steering steering;
        public CarTelemetry currentTelemetry;
        public float RemoteSteeringAngle { get; set; }
        public float RemoteAcceleration { get; set; }
        public MoveAlongCircuitWithCarControl moveAlongCircuit;
    }
}
