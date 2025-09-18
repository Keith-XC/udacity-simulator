using UnityEngine;
using UnityStandardAssets.Utility;
using Assets.TcpServerManager;
using System;
using System.Collections.Generic;
using System.Linq;
using Assets.OtherDriver;
using Assets.WayPointSystem2;
using Random = UnityEngine.Random;
using Assets.Driver;
using System.Text;
using UnityStandardAssets.Vehicles.Car;
using WayPointManager = Assets.WayPointSystem2.WayPointManager;
using System.Reflection;

public class Spawner : MonoBehaviour
{
    private TcpServerManager _tcpServerManager;
    private OtherObjectsServer _otherObjectsServer;
    public static event Action OnCarSpawned;
    // To track used car IDs and avoid duplicates
    private HashSet<int> usedCarIds = new HashSet<int>();
    
    [Header("Car Prefabs and Spawn Settings")]
    public string[] possibleCarPrefabs = { "Objects/CarBlue", "Objects/CarRed", "Objects/CarBlack" };
    public float minSpeed = 10f;
    public float maxSpeed = 50f;
    public Vector3 minScale = new Vector3(0.8f, 0.8f, 0.8f);
    public Vector3 maxScale = new Vector3(1.2f, 1.2f, 1.2f);
    public float offsetRange = 1f;
    public int RandomCars = 0;

    /// <summary>
    /// Initializes the TcpServerManager and subscribes to commands.
    /// Also spawns random cars if specified and loads the UI.
    /// </summary>
    void Start()

    {
        _tcpServerManager = TcpServerManager.Instance;
        _otherObjectsServer = _tcpServerManager.GetOtherCarsServer();

        if (_otherObjectsServer != null)
        {
            _otherObjectsServer.OnOtherCarsCommandReceived += HandleOtherCarsCommand;
            Debug.Log("Spawner: Subscribed to OnOtherCarsCommandReceived event.");
        }
        else
        {
            Debug.LogError("Spawner: TcpServerManager instance is not available.");
        }

        if (RandomCars > 0)
        {
            SpawnRandomCars(RandomCars);
        }

        GameObject UI = Resources.Load<GameObject>("Objects/CarUI");
        Instantiate(UI, Vector3.zero, Quaternion.identity);
    }

    /// <summary>
    /// Unsubscribes from server command events on destroy.
    /// </summary>
    void OnDestroy()
    {
        if (_otherObjectsServer != null)
        {
            _otherObjectsServer.OnOtherCarsCommandReceived -= HandleOtherCarsCommand;
        }
        // Clear used IDs on destroy
        usedCarIds.Clear();
    }

    /// <summary>
    /// Handles commands received from the OtherCarsServer.
    /// </summary>
    /// <param name="commandData">The received command data.</param>
    void HandleOtherCarsCommand(CommandData commandData)
    {
        Debug.Log("Spawner: HandleOtherCarsCommand invoked.");
        if (commandData == null || string.IsNullOrEmpty(commandData.command))
        {
            Debug.LogWarning("Spawner: Invalid CommandData received.");
            return;
        }

        try
        {
            Debug.Log($"Spawner: Received command {commandData.command}");
            SpawnVectors spawnVector;

            switch (commandData.command.ToLower())
            {
                case "spawn_car":
                    if (commandData.spawn_point != null)
                    {
                        Debug.Log($"Spawner: Spawn point: {commandData.spawn_point}");
                    }
                    spawnVector = new SpawnVectors(commandData.offset, commandData.scale_Vektor, commandData.rotation);
                    var waitList = commandData.waitingPoints != null ? commandData.waitingPoints.ToList() : new List<WaitPoint>();
                    var wpListSingle = commandData.waypoints != null ? commandData.waypoints.ToList() : new List<string>();
                    int assignedCarId = SpawnCarsWithResponse(
                        commandData.name,
                        commandData.prefab_name,
                        commandData.spawn_point,
                        commandData.speed,
                        spawnVector,
                        waitList,
                        wpListSingle,
                        commandData.layer,
                        commandData.humanBehavior,
                        commandData.autonomous,
                        commandData.requestedCarId
                    );
                    if (commandData.autonomous)
                        SendSpawnResponse(assignedCarId, commandData.requestedCarId);

                    break;

                case "spawn_cars":
                    if (commandData.cars != null)
                    {
                        foreach (var carData in commandData.cars)
                        {
                            SpawnVectors sv = new SpawnVectors(carData.offset, carData.scale_Vektor, carData.rotation);
                            var waitPts = carData.waitingPoints != null ? carData.waitingPoints.ToList() : new List<WaitPoint>();
                            var wpList = carData.waypoints != null ? carData.waypoints.ToList() : new List<string>();
                            int id = SpawnCarsWithResponse(
                                carData.name,
                                carData.prefab_name,
                                carData.spawn_point,
                                carData.speed,
                                sv,
                                waitPts,
                                wpList,
                                carData.layer,
                                carData.humanBehavior,
                                carData.autonomous,
                                carData.requestedCarId
                            );
                            if (carData.autonomous)
                                Debug.Log($"Spawner: Sent spawn response for car ID {id}");
                                SendSpawnResponse(id, carData.requestedCarId);
                        }
                    }
                    break;

                case "spawn_static_object":
                    if (commandData.spawn_point != null)
                    {
                        Debug.Log($"Spawner: Spawn point: {commandData.spawn_point}");
                    }
                    spawnVector = new SpawnVectors(commandData.offset, commandData.scale_Vektor, commandData.rotation);
                    SpawnObject(commandData.prefab_name, commandData.spawn_point, spawnVector, commandData.waypoints.ToList());
                    break;

                case "spawn_random_cars":
                    SpawnRandomCars(commandData.randomCarAmount);
                    break;

                default:
                    Debug.Log($"Spawner: Unknown command received: {commandData.command}");
                    break;
            }
        }
        catch (Exception e)
        {
            Debug.LogError($"Spawner: Error processing command {commandData.command}: {e.Message}");
        }
    }
    
    // get the next available car ID, ensuring no duplicates
    private int GetNextAvailableId(int requestedId)
    {
        // if requested ID is 1, make sure it's for ego car
        if (requestedId == 1)
        {
            if (usedCarIds.Contains(1))
            {
                Debug.LogError("Spawner: Car ID 1 is reserved for ego car and already in use!");
                return -1;
            }
            usedCarIds.Add(1);
            return 1;
        }

        // for other cars, assign the next available ID starting from 2
        int newId = 2;
        while (usedCarIds.Contains(newId))
        {
            newId++;
        }
        usedCarIds.Add(newId);
        return newId;
    }

    /// <summary>
    /// Spawns a car with the provided parameters and returns the assigned CarId.
    /// </summary>
    public int SpawnCarsWithResponse(
        string carName,
        string prefabName,
        float startPosition,
        float speedPerCar,
        SpawnVectors spawnVectors,
        List<WaitPoint> waitPoints,
        List<string> waypoints,
        string layer,
        float humanBehavior,
        bool autonomous,
        int requestedCarId)
    {
        if (string.IsNullOrEmpty(prefabName))
        {
            Debug.LogError("Spawner: Prefab name is not provided.");
            return -1;
        }
        GameObject carPrefab = Resources.Load<GameObject>(prefabName);
        if (carPrefab == null)
        {
            Debug.LogError($"Spawner: Could not find prefab with the name {prefabName} in the Resources folder.");
            return -1;
        }

        var manager = FindObjectsOfType<WayPointManager>().First();

        
        /*WayPointCircuit wayPointCircuit = waypoints.Any()
            ? manager.combineWayPointCircuit(waypoints)
            : manager.Circuits[Random.Range(0, manager.Circuits.Count)];
        */
        WayPointCircuit wayPointCircuit;
        if (waypoints != null && waypoints.Any())
        {
            try 
            {
                wayPointCircuit = manager.combineWayPointCircuit(waypoints);
                if (wayPointCircuit == null)
                {
                    Debug.LogWarning($"Spawner: Could not find waypoint circuit {waypoints[0]}, falling back to first available circuit");
                    if (manager.Circuits == null || manager.Circuits.Count == 0)
                    {
                        Debug.LogError("Spawner: No circuits available in WayPointManager");
                        SendAvailableCircuits(new List<string>());
                        return -1;
                    }
                    wayPointCircuit = manager.Circuits[0];
                    Debug.Log($"Spawner: Using circuit: {wayPointCircuit.name}");
                    
                    // Send available circuits to client
                    var circuitNames = manager.Circuits.Select(c => c.name).ToList();
                    SendAvailableCircuits(circuitNames);
                }
            }
            catch (Exception e)
            {
                Debug.LogWarning($"Spawner: Error finding waypoint circuit: {e.Message}, falling back to first available circuit");
                if (manager.Circuits == null || manager.Circuits.Count == 0)
                {
                    Debug.LogError("Spawner: No circuits available in WayPointManager");
                    return -1;
                }
                // Send available circuits to client
                var circuitNames = manager.Circuits.Select(c => c.name).ToList();
                SendAvailableCircuits(circuitNames);

                wayPointCircuit = manager.Circuits[0];
                Debug.Log($"Spawner: Using circuit: {wayPointCircuit.name}");
            }
        }
        else
        {
            if (manager.Circuits == null || manager.Circuits.Count == 0)
            {
                Debug.LogError("Spawner: No circuits available in WayPointManager");
                return -1;
            }
            wayPointCircuit = manager.Circuits[0];
            Debug.Log($"Spawner: No waypoints specified, using first circuit: {wayPointCircuit.name}");
        }

        // --- Pose on track (NOT spawnVectors._size!) ---
        int count = wayPointCircuit.WayPoints.Count;
        int waypointIndex = Mathf.Clamp((int)startPosition, 0, count - 1);
        int nextIndex = (waypointIndex + 1) % count;

        var wpA = wayPointCircuit.WayPoints[waypointIndex];
        var wpB = wayPointCircuit.WayPoints[nextIndex];

        Vector3 waypointPosition = wpA.Position;
        Vector3 forward = (wpB.Position - wpA.Position).normalized;
        Quaternion pathRotation = Quaternion.LookRotation(forward);
        Quaternion spawnRotation = pathRotation * spawnVectors._rotation;

        // --- Instantiate at the waypoint pose ---
        GameObject car = Instantiate(carPrefab, waypointPosition, spawnRotation);
        car.transform.localScale = spawnVectors._size;
        car.tag = "Car";

        MoveAlongCircuitWithCarControl moveScript = car.GetComponent<MoveAlongCircuitWithCarControl>();

        if (moveScript != null && wayPointCircuit.WayPoints.Any() && !autonomous)
        {
            // NPC cars follow the circuit
            moveScript.Speed = speedPerCar;
            moveScript.Offset = spawnVectors._offset;
            moveScript.WaitPoints = waitPoints;
            moveScript.SetRoadLayerByName(layer);
            moveScript.Circuit = wayPointCircuit;
            moveScript.PrecomputeWaypointDistances();
            moveScript.SetStartPoint(waypointIndex);
            moveScript.HumanBehavior = humanBehavior;
            moveScript.CarName = carName;

            // keep exact initial pose
            car.transform.SetPositionAndRotation(waypointPosition, spawnRotation);
        }
        else
        {
            // Remote/agent car: lock exact pose via CarController API
            var ctrl = car.GetComponent<CarController>();
            if (ctrl != null)
                ctrl.SetCarPositionAndRotation(waypointPosition, spawnRotation);
        }

        // Assign id + mark autonomous (remote) on the identifier
        CarIdentifier identifier = car.GetComponent<CarIdentifier>();
        int assignedId = -1;
        if (identifier != null)
        {
            // 检查是否是ego car (requestedId == 1)
            if (requestedCarId == 1 && autonomous)
            {
                assignedId = GetNextAvailableId(1);
                if (assignedId == -1)
                {
                    Debug.LogError("Spawner: Failed to assign ID 1 for ego car - already in use");
                    Destroy(car);
                    return -1;
                }
            }
            else
            {
                // 其他车辆使用自动分配的ID
                assignedId = GetNextAvailableId(2); // 从2开始分配
            }

            identifier.CarId = assignedId;
            identifier.autonomous = autonomous;
            Debug.Log($"Spawner: Assigned ID {assignedId} to car {carName}");
        }
        else
        {
            Debug.LogWarning("Spawner: CarIdentifier not found. Ensure that the prefab contains one.");
            Destroy(car);
            return -1;
        }

        if (autonomous)
            Debug.Log("Spawner: " + carName + " is in REMOTE mode (controlled via CarManager).");
            if (requestedCarId == 1)
            {
                // Main camera setup
                var mainCam = car.transform.Find("Main Camera");
                if (mainCam == null) 
                {
                    GameObject mainCamera = new GameObject("Main Camera");
                    mainCamera.transform.SetParent(car.transform);
                    mainCamera.transform.localPosition = new Vector3(0, 2, -4);
                    mainCamera.transform.localRotation = Quaternion.Euler(10, 0, 0);
                    Camera cam = mainCamera.AddComponent<Camera>();
                    cam.tag = "MainCamera";
                    mainCamera.AddComponent<AudioListener>();  // Only add AudioListener to ego car
                }

                // Front camera setup
                var frontCam = car.transform.Find("Front Facing Camera");
                if (frontCam == null) 
                {
                    GameObject frontCamera = new GameObject("Front Facing Camera");
                    frontCamera.transform.SetParent(car.transform);
                    frontCamera.transform.localPosition = new Vector3(0, 2.5f, -0.5f);
                    frontCamera.transform.localRotation = Quaternion.identity;
                    Camera fcam = frontCamera.AddComponent<Camera>();
                    fcam.enabled = false;
                }
            }
        else
            Debug.Log("Spawner: " + carName + " is in autonomous mode (controlled by MoveAlongCircuit).");

        // Keep list consistent for existing code paths; null is OK if remote car has no mover
        manager.cars.Add(moveScript);
        
        // Triggering a vehicle spawn event, will be listened by CarManager to update its list
        OnCarSpawned?.Invoke();
        return assignedId;
    }

    [Serializable]
    private class CircuitEventData
    {
        public string event_type = "available_circuits";
        public List<string> circuits;
    }

    private void SendAvailableCircuits(List<string> circuits)
    {
        CircuitEventData eventData = new CircuitEventData
        {
            circuits = circuits
        };
        string json = JsonUtility.ToJson(eventData);
        Debug.Log($"Spawner: Available circuits count: {circuits.Count}");
        foreach (var circuit in circuits)
        {
            Debug.Log($"Spawner: Found circuit: {circuit}");
        }
        _tcpServerManager.GetEventServer().SendEventResponse(json);
        Debug.Log($"Spawner: Sent available circuits to client: {json}");
    }

    /// <summary>
    /// Sends a spawn response with the assigned CarId.
    /// </summary>
    private void SendSpawnResponse(int assignedCarId, int requestedCarId)
    {
        SpawnResponse response = new SpawnResponse
        {
            command = "spawn_response",
            assignedCarId = assignedCarId,
            requestedCarId = requestedCarId
        };
        _tcpServerManager.SendSpawnResponse(response);
    }

    /// <summary>
    /// Spawns a static object at the specified waypoint.
    /// </summary>
    public void SpawnObject(string prefabName, float startPosition, SpawnVectors spawnVectors, List<string> waypoints)
    {
        if (string.IsNullOrEmpty(prefabName))
        {
            Debug.LogError("Spawner: Prefab name is not provided.");
            return;
        }
        GameObject objectPrefab = Resources.Load<GameObject>(prefabName);
        if (objectPrefab == null)
        {
            Debug.LogError($"Spawner: Could not find prefab with name {prefabName} in the Resources folder.");
            return;
        }

        var manager = FindObjectsOfType<WayPointManager>().First();
        var wayPointCircuit = manager.combineWayPointCircuit(waypoints);
        float waypointIndex = startPosition;
        if (waypointIndex < 0 || waypointIndex >= wayPointCircuit.WayPoints.Count)
        {
            Debug.LogWarning($"Spawner: Invalid waypoint index: {waypointIndex}. Using waypoint 0.");
            waypointIndex = 0;
        }
        Vector3 spawnPosition = wayPointCircuit.WayPoints[(int)waypointIndex].Position;
        GameObject objectToSpawn = Instantiate(objectPrefab, spawnPosition, spawnVectors._rotation);
        objectToSpawn.transform.localScale = spawnVectors._size;
        SpawnObject spawnObject = objectToSpawn.GetComponent<SpawnObject>();
        if (spawnObject != null)
        {
            spawnObject._circuit = wayPointCircuit;
            spawnObject.SpawnAtWaypoint(waypointIndex, spawnVectors._offset, spawnVectors._size, spawnVectors._rotation);
        }
        else
        {
            Debug.LogWarning("Spawner: SpawnObject script not found on the object prefab.");
        }
    }

    /// <summary>
    /// Spawns a given number of random cars.
    /// </summary>
    public void SpawnRandomCars(int count)
    {
        var manager = FindObjectsOfType<WayPointManager>().First();
        for (int i = 0; i < count; i++)
        {
            string carName = "RandomCar_" + i;
            if (possibleCarPrefabs == null || possibleCarPrefabs.Length == 0)
            {
                Debug.LogError("Spawner: No possible prefab names defined.");
                return;
            }
            string prefabName = possibleCarPrefabs[Random.Range(0, possibleCarPrefabs.Length)];
            float speedPerCar = Random.Range(minSpeed, maxSpeed);
            Vector3 randomOffset = new Vector3(Random.Range(-offsetRange, offsetRange), 0, Random.Range(-offsetRange, offsetRange));
            Vector3 randomScale = new Vector3(
                Random.Range(minScale.x, maxScale.x),
                Random.Range(minScale.y, maxScale.y),
                Random.Range(minScale.z, maxScale.z)
            );
            Quaternion spawnRotation = Quaternion.Euler(0, 0, 0);
            List<WaitPoint> waitPoints = new List<WaitPoint>();
            string layer = "Road";
            float humanBehavior = 50;
            GameObject carPrefab = Resources.Load<GameObject>(prefabName);
            GameObject car = Instantiate(carPrefab, randomScale, spawnRotation);
            car.transform.localScale = randomScale;
            car.tag = "Car";
            
            // Remove any existing AudioListener since random cars shouldn't have one
            var audioListener = car.GetComponentInChildren<AudioListener>();
            if (audioListener != null)
            {
                Destroy(audioListener);
            }
            
            MoveAlongCircuitWithCarControl moveScript = car.GetComponent<MoveAlongCircuitWithCarControl>();
            if (moveScript != null)
            {
                moveScript.Speed = speedPerCar;
                moveScript.Offset = randomOffset;
                moveScript.WaitPoints = waitPoints;
                moveScript.SetRoadLayerByName(layer);
                moveScript.SetStartPoint(Int16.MaxValue);
                moveScript.HumanBehavior = humanBehavior;
                moveScript.CarName = carName;
            }
            else
            {
                Debug.LogWarning("Spawner: MoveAlongCircuit script not found on the car prefab.");
            }
            CarIdentifier identifier = car.GetComponent<CarIdentifier>();
            if (identifier != null)
            {
                // 使用GetNextAvailableId而不是直接赋值
                int newId = GetNextAvailableId(2); // NPC车从2开始
                if (newId == -1)
                {
                    Debug.LogError($"Failed to assign ID for random car {i}");
                    Destroy(car);
                    continue;
                }
                identifier.CarId = newId;
                Debug.Log($"Assigned ID {newId} to random car {carName}");
            }
            
            manager.cars.Add(moveScript);
            }
    }
}
