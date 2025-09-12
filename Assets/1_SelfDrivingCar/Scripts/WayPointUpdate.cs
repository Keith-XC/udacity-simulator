using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityStandardAssets.Vehicles.Car;

public class WayPointUpdate : MonoBehaviour
{
    private GameObject app;
    private EpisodeManager episodeManager;
    public GameObject[] waypoints;
    public int laps;
    public GameObject currentWayPoint;
    public GameObject firstWayPoint;
    public int numberOfWayPoints = 0;
    public float cteThreshold = 7.0f;
    private float wayPointActivationDistance = 5.00f;
    private float timeToGo;
    private float updateDelay = 0.1f;
    private CarController carController;
    private bool isCarCrashed;
    private int stuckTimer;
    private bool isCrashedInTheLastSecond = false;
    private long lastCrash;
    private float prevSpeed = 0;
    private int decelerationSign = 1;
    private int oot_counter = 0;
    private int collision_counter = 0;
    public CarCollider carCollider;
    private float timeLeft = 120.0f;
    private float total_driven_distance = 0.0f;
    private float angular_difference = 0.0f;
    private int timeAfterRepositioning = 0;
    private bool autoResetEnabled = false;

    /// <summary>
    /// Finds the application object and retrieves the EpisodeManager.
    /// </summary>
    void Awake()
    {
        app = GameObject.Find("__app");
        if (app != null)
        {
            episodeManager = app.GetComponent<EpisodeManager>();
        }
    }

    /// <summary>
    /// Initializes waypoints, sets the first waypoint, and retrieves the CarController and CarCollider components.
    /// </summary>
    void Start()
    {
        waypoints = GameObject.FindGameObjectsWithTag("Waypoint");
        numberOfWayPoints = waypoints.Length;
        firstWayPoint = findWayPointByNumber(0);
        laps = 1;
        currentWayPoint = firstWayPoint;
        carController = GetComponent<CarController>();
        carCollider = GetComponent<CarCollider>();
        timeToGo = Time.fixedTime + updateDelay;
    }

    /// <summary>
    /// Updates waypoint activation, stuck detection, and triggers repositioning if necessary.
    /// </summary>
    void Update()
    {
        if (Input.GetKeyDown(KeyCode.Space))
        {
            moveCarToNextWayPoint();
        }
        if (Input.GetKeyDown(KeyCode.K))
        {
            autoResetEnabled = !autoResetEnabled;
        }
        if (currentWayPoint != null && autoResetEnabled)
        {
            bool stuck = detectCarIsStuck();
            if (stuck && timeAfterRepositioning > 180)
            {
                moveCarToNextWayPoint();
                timeAfterRepositioning = 0;
                return;
            }
            if (Time.fixedTime >= timeToGo)
            {
                foreach (GameObject wayPoint in waypoints)
                {
                    if (getWayPointNumber(wayPoint) == (getWayPointNumber(currentWayPoint) + 1) % numberOfWayPoints)
                    {
                        float dist = Vector3.Distance(transform.position, wayPoint.transform.position);
                        Vector3 targetDir = wayPoint.transform.position - transform.position;
                        angular_difference = Vector3.Angle(targetDir, transform.forward);
                        if (dist < wayPointActivationDistance)
                        {
                            currentWayPoint = wayPoint;
                            if (currentWayPoint == firstWayPoint)
                            {
                                laps += 1;
                            }
                        }
                    }
                }
                timeToGo = Time.fixedTime + updateDelay;
            }
            timeAfterRepositioning++;
        }
    }

    /// <summary>
    /// Resets the car by stopping and repositioning it at the next waypoint.
    /// </summary>
    private void HandleCarReset()
    {
        Debug.Log("Car reset triggered.");
        stopCar();
        moveCarToNextWayPoint();
        stopCar();
    }

    /// <summary>
    /// Determines if the car is stuck based on its current speed.
    /// </summary>
    /// <returns>True if the car is considered stuck; otherwise false.</returns>
    private bool detectCarIsStuck()
    {
        solveLastCrash();
        if (carController.CurrentSpeed < 1)
        {
            stuckTimer++;
            if (stuckTimer > 120)
            {
                isCarCrashed = true;
                isCrashedInTheLastSecond = true;
                stuckTimer = 0;
                return true;
            }
            return true;
        }
        else
        {
            isCarCrashed = false;
            isCrashedInTheLastSecond = false;
            stuckTimer = 0;
            return false;
        }
    }

    /// <summary>
    /// Stops the car by invoking the CarController's Stop method.
    /// </summary>
    private void stopCar()
    {
        carController.Stop();
    }

    /// <summary>
    /// Registers an out-of-track event and manages the repositioning.
    /// </summary>
    public void registerOutOfTrack()
    {
        if (currentWayPoint != null)
        {
            if (laps != 1 || getWayPointNumber(currentWayPoint) != 0)
            {
                isCarCrashed = true;
                isCrashedInTheLastSecond = true;
                lastCrash = System.DateTime.Now.ToFileTime();
                oot_counter++;
                episodeManager.AddEvent("out_of_track", "");
            }
            manageOutOfTrack();
        }
    }

    /// <summary>
    /// Registers a collision event.
    /// </summary>
    public void registerCollision()
    {
        if (currentWayPoint != null)
        {
            if (laps != 1 || getWayPointNumber(currentWayPoint) != 0)
            {
                isCarCrashed = true;
                isCrashedInTheLastSecond = true;
                lastCrash = System.DateTime.Now.ToFileTime();
                collision_counter++;
                episodeManager.AddEvent("collision", "");
            }
            manageCollision();
        }
    }

    /// <summary>
    /// Handles an out-of-track event by stopping and repositioning the car.
    /// </summary>
    public void manageOutOfTrack()
    {
        stopCar();
        moveCarToNextWayPoint();
        stopCar();
    }

    /// <summary>
    /// Handles a collision event. (Currently no repositioning is performed.)
    /// </summary>
    public void manageCollision()
    {
    }

    /// <summary>
    /// Clears crash status.
    /// </summary>
    public void solveCrash()
    {
        isCarCrashed = false;
        solveLastCrash();
    }

    /// <summary>
    /// Resets the crash flag if sufficient time has passed since the last crash.
    /// </summary>
    public void solveLastCrash()
    {
        if (!isCarCrashed && (System.DateTime.Now.ToFileTime() - lastCrash) > 10000000)
        {
            isCrashedInTheLastSecond = false;
        }
    }

    /// <summary>
    /// Returns whether the car is crashed.
    /// </summary>
    public bool isCrash()
    {
        return isCarCrashed;
    }

    /// <summary>
    /// Returns whether the car crashed in the last second.
    /// </summary>
    public bool isCrashInTheLastSecond()
    {
        return isCrashedInTheLastSecond;
    }

    /// <summary>
    /// Moves the car to the next waypoint based on the current waypoint's number.
    /// </summary>
    public void moveCarToNextWayPoint()
    {
        int currentWayPointNumber = getWayPointNumber(currentWayPoint);
        int nextWayPointNumber = (currentWayPointNumber + 1) % numberOfWayPoints;
        GameObject nextWayPoint = findWayPointByNumber(nextWayPointNumber);
        Vector3 newRotation = nextWayPoint.transform.rotation.eulerAngles;
        transform.position = nextWayPoint.transform.position;
        transform.eulerAngles = newRotation;
        solveCrash();
    }

    /// <summary>
    /// Returns the current waypoint number.
    /// </summary>
    public int getCurrentWayPointNumber()
    {
        return getWayPointNumber(currentWayPoint);
    }

    /// <summary>
    /// Returns the total number of waypoints.
    /// </summary>
    public int getTotalWayPointNmber()
    {
        return numberOfWayPoints;
    }

    /// <summary>
    /// Returns the current lap number.
    /// </summary>
    public int getLapNumber()
    {
        return laps;
    }

    /// <summary>
    /// Returns the out-of-track counter.
    /// </summary>
    public int getOBENumber()
    {
        return oot_counter;
    }

    /// <summary>
    /// Returns the total driven distance.
    /// </summary>
    public float getDrivenDistance()
    {
        return total_driven_distance;
    }

    /// <summary>
    /// Returns the angular difference.
    /// </summary>
    public float getAngularDifference()
    {
        return angular_difference;
    }

    /// <summary>
    /// Returns the collision counter.
    /// </summary>
    public int getCrashNumber()
    {
        return collision_counter;
    }

    /// <summary>
    /// Extracts the waypoint number from the GameObject's name.
    /// </summary>
    private int getWayPointNumber(GameObject wayPoint)
    {
        string name = wayPoint.name;
        string stringNumb = name.Substring(name.Length - 3);
        return int.Parse(stringNumb);
    }

    /// <summary>
    /// Finds a waypoint by its number based on its name.
    /// </summary>
    private GameObject findWayPointByNumber(int number)
    {
        string suffix = "";
        if (number < 10)
            suffix = "00" + number;
        else if (number < 100)
            suffix = "0" + number;
        else
            suffix += number;
        string wayPointName = "Waypoint " + suffix;
        return GameObject.Find(wayPointName);
    }
}
