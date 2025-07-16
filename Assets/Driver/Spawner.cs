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

    [Header("Car Prefabs and Spawn Settings")]
    public string[] possibleCarPrefabs = { "Objects/CarBlue", "Objects/CarRed", "Objects/CarBlack" };
    public float minSpeed = 10f;
    public float maxSpeed = 50f;
    public Vector3 minScale = new Vector3(0.8f, 0.8f, 0.8f);
    public Vector3 maxScale = new Vector3(1.2f, 1.2f, 1.2f);
    public float offsetRange = 1f; 
    public int RandomCars = 0;
    public bool spawnMinmumCar = true;

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

        if (RandomCars != 0)
        {
            SpawnRandomCars(RandomCars);
        }
        else if (spawnMinmumCar)
        {
            SpawnRandomCars(1);
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
                    int assignedCarId = SpawnCarsWithResponse(
                        commandData.name,
                        commandData.prefab_name,
                        commandData.spawn_point,
                        commandData.speed,
                        spawnVector,
                        commandData.waitingPoints.ToList(),
                        commandData.waypoints.ToList(),
                        commandData.layer,
                        commandData.humanBehavior,
                        commandData.autonomous
                    );
                    if(commandData.autonomous)
                        SendSpawnResponse(assignedCarId, commandData.requestedCarId);
                    break;

                case "spawn_static_object":
                    if (commandData.spawn_point != null)
                    {
                        Debug.Log($"Spawner: Spawn point: {commandData.spawn_point}");
                    }
                    spawnVector = new SpawnVectors(commandData.offset, commandData.scale_Vektor, commandData.rotation);
                    SpawnObject(commandData.prefab_name, commandData.spawn_point, spawnVector, commandData.waypoints.ToList());
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

    /// <summary>
    /// Spawns a car with the provided parameters and returns the assigned CarId.
    /// </summary>
    public int SpawnCarsWithResponse(string carName, string prefabName, float startPosition, float speedPerCar, SpawnVectors spawnVectors,
                         List<WaitPoint> waitPoints, List<string> waypoints, string layer, float humanBehavior, bool autonomous)
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
        var wayPointCircuit = new WayPointCircuit();
        if (waypoints.Any())
        {
            wayPointCircuit = manager.combineWayPointCircuit(waypoints);
        }
        else
        {
            wayPointCircuit = manager.Circuits[Random.Range(0, manager.Circuits.Count)];
        }
        
        int waypointIndex = (int)startPosition;
        if (waypointIndex < 0 || waypointIndex >= wayPointCircuit.GetComponent<WayPointCircuit>().WayPoints.Count)
        {
            Debug.LogWarning($"Spawner: Invalid waypoint index: {waypointIndex}. Using waypoint 0.");
            waypointIndex = 0;
        }
        Quaternion spawnRotation = spawnVectors._rotation;
        Debug.Log($"Spawner: Spawning car at waypoint {waypointIndex}.");

        GameObject car = Instantiate(carPrefab, spawnVectors._size, spawnRotation);
        car.transform.localScale = spawnVectors._size;
        car.tag = "Car";

        MoveAlongCircuitWithCarControl moveScript = car.GetComponent<MoveAlongCircuitWithCarControl>();
        if (moveScript != null && wayPointCircuit.WayPoints.Any() && !autonomous)
        {
            moveScript.Speed = speedPerCar;
            moveScript.Offset = spawnVectors._offset;
            moveScript.WaitPoints = waitPoints;
            moveScript.SetRoadLayerByName(layer);
            moveScript.Circuit = wayPointCircuit;
            moveScript.PrecomputeWaypointDistances();
            moveScript.SetStartPoint(waypointIndex);
            moveScript.HumanBehavior = humanBehavior;
            moveScript.CarName = carName;
        }
        else
        {
            if (manager != null && manager.Circuits != null && manager.Circuits.Count > 0)
            { 

                var waypoint = wayPointCircuit.WayPoints[waypointIndex];
                var nextWaypoint = new WayPoint();
                if (wayPointCircuit.WayPoints.Count > waypointIndex++)
                {
                    nextWaypoint = wayPointCircuit.WayPoints[waypointIndex++];
                }

                Vector3 direction = nextWaypoint.Position - waypoint.Position;
                car.GetComponent<CarController>().SetCarPositionAndRotation(waypoint.Position, Quaternion.LookRotation(direction.normalized) * spawnVectors._rotation);
            }
        }

        CarIdentifier identifier = car.GetComponent<CarIdentifier>();
        int assignedId = -1;
        if (identifier != null)
        {
            assignedId = manager.cars.Count() + 1;
            identifier.CarId = assignedId;
            identifier.autonomous = autonomous;
        }
        else
        {
            Debug.LogWarning("Spawner: CarIdentifier not found. Ensure that the prefab contains one.");
        }

        if (autonomous)
        {
            if (moveScript != null)
            {
                moveScript.enabled = false;
            }
            Debug.Log("Spawner: " + carName + " is in REMOTE mode (controlled via CarManager).");
        }
        else
        {
            Debug.Log("Spawner: " + carName + " is in autonomous mode (controlled by MoveAlongCircuit).");
        }
        manager.cars.Add(moveScript);

        return assignedId;
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
            identifier.CarId = manager.cars.Count() + 1;
            manager.cars.Add(moveScript);
        }
    }
}
