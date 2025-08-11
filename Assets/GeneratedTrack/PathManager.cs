using System.Collections.Generic;
using System.Linq;
using Assets.OtherDriver;
using Assets.Traffic_Lights_System.Scripts;
using System.Threading.Tasks;
using UnityEngine;
using UnityEngine.SceneManagement;
using Random = UnityEngine.Random;
using Assets.TcpServerManager;
using System.Collections;

public class PathManager : MonoBehaviour
{
    public List<CarPath> carPaths = new List<CarPath>();

    [Header("Path Settings")]
    public bool doMakeRandomPath = false;
    public bool doLoadPointPath = true;

    [Header("Path Source")]
    public PathDrawer pathDrawer;

    [Header("Smoothing")]
    public int smoothPathIter = 1;

    [Header("Start Position")]
    public Transform startPos;

    [Header("Other Settings")]
    public bool invertNodes = false;

    [Tooltip("Scripts that implement IWaitCarPath (e.g., RoadBuilder)")]
    public GameObject[] initAfterCarPathLoaded;

    [Header("Random Path Parameters")]
    public int numSpans = 100;
    public float turnInc = 1f;
    public float spanDist = 5f;
    public bool sameRandomPath = true;
    public int randSeed = 2;
    public bool ClosedCircuit;

    [Header("Multiple Circuits")]
    public int numberOfRandomCircuits = 1;

    [Header("Random Path Fluctuations")]
    public int turnChangeInterval = 5;

    [Header("Random Start Offset")]
    public Vector2 startPointRandomRange = new Vector2(50, 50);


    public int randomCarAmount = 15;

    private OtherObjectsServer _otherObjectsServer;
    private TcpServerManager _tcpServerManager;

    /// <summary>
    /// Initializes car paths using random or point-based methods.
    /// </summary>
    private void Awake()
    {
        if (sameRandomPath)
        {
            Random.InitState(randSeed);
        }
        InitCarPaths();
    }

    /// <summary>
    /// Initializes TCP communication and registers event handlers.
    /// </summary>
    private void Start()
    {
        _tcpServerManager = TcpServerManager.Instance;
        _otherObjectsServer = _tcpServerManager.GetOtherCarsServer();
        if (_otherObjectsServer != null)
        {
            _otherObjectsServer.OnOtherCarsCommandReceived += ProcessEventCommand;
        }
        else
        {
            Debug.LogError("EventServer is null. Cannot register event handlers.");
        }
    }

    /// <summary>
    /// Unregisters event handlers on destruction.
    /// </summary>
    private void OnDestroy()
    {
        if (_otherObjectsServer != null)
        {
            _otherObjectsServer.OnOtherCarsCommandReceived -= ProcessEventCommand;
        }
    }

    /// <summary>
    /// Processes incoming TCP commands to generate track data.
    /// </summary>
    /// <param name="commandData">Command data received from TCP.</param>
    private void ProcessEventCommand(CommandData commandData)
    {
        if (commandData.command == "Generate_Track")
        {
            UnityMainThreadDispatcher.Instance().Enqueue(() =>
            {
                bool hasCoords = commandData.trackDataList != null &&
                                 commandData.trackDataList.Any(td => td.coords != null && td.coords.Count > 0);
                if (hasCoords)
                {
                    foreach (TrackData track in commandData.trackDataList)
                    {
                        if (track.coords != null)
                        {
                            PathSegment newSegment = new PathSegment();
                            newSegment.closed = true;
                            newSegment.roadWidth = track.roadWidth;
                            newSegment.laneWidth = track.laneWidth;
                            foreach (Vector3Data v3 in track.coords)
                            {
                                Vector3 worldPos = new Vector3(v3.x, v3.y, v3.z);
                                Vector3 localPos = pathDrawer.transform.InverseTransformPoint(worldPos);
                                newSegment.points.Add(localPos);
                            }
                            pathDrawer.segments.Add(newSegment);
                        }
                    }
                    MakePointPaths();
                }
                else
                {
                    for (int i = 0; i < numberOfRandomCircuits; i++)
                    {
                        if (sameRandomPath)
                        {
                            Random.InitState(randSeed + i * 1000);
                        }
                        CarPath cp = MakeRandomPath(i);
                        cp.streetWidth = Random.Range(2, 15);
                        cp.pathType = "random_circuit_" + i;
                        carPaths.Add(cp);
                    }
                }
                UnityMainThreadDispatcher.Instance().Enqueue(
                    InitAfterCarPathLoaded(initAfterCarPathLoaded, commandData.randomCarAmount)
                );
            });
        }
    }

    /// <summary>
    /// Clears existing car paths and generates new ones using random or point-based methods.
    /// </summary>
    public void InitCarPaths()
    {
        carPaths.Clear();
        if (doMakeRandomPath)
        {
            for (int i = 0; i < numberOfRandomCircuits; i++)
            {
                if (sameRandomPath)
                {
                    Random.InitState(randSeed + i * 1000);
                }
                CarPath cp = MakeRandomPath(i);
                cp.pathType = "random_circuit_" + i;
                cp.streetWidth = Random.Range(2, 15);
                carPaths.Add(cp);
            }
        }
        if (doLoadPointPath)
        {
            MakePointPaths();
        }
        if (invertNodes)
        {
            for (int p = 0; p < carPaths.Count; p++)
            {
                CarPath orig = carPaths[p];
                CarPath newPath = new CarPath();
                for (int i = orig.nodes.Count - 1; i >= 0; i--)
                {
                    PathNode node = orig.nodes[i];
                    newPath.nodes.Add(node);
                    newPath.centerNodes.Add(node);
                }
                carPaths[p] = newPath;
            }
        }
    }

    /// <summary>
    /// Generates car paths from point data provided by the PathDrawer.
    /// </summary>
    public void MakePointPaths()
    {
        carPaths.Clear();
        if (pathDrawer == null || pathDrawer.segments == null || pathDrawer.segments.Count == 0)
        {
            Debug.LogError("No PathDrawer or segments defined!");
            return;
        }
        foreach (var segment in pathDrawer.segments)
        {
            if (segment.points == null || segment.points.Count < 2)
                continue;
            List<Vector3> points = new List<Vector3>(segment.points.Count);
            for (int i = 0; i < segment.points.Count; i++)
            {
                Vector3 worldPos = pathDrawer.transform.TransformPoint(segment.points[i]);
                points.Add(worldPos);
            }
            for (int i = 0; i < smoothPathIter; i++)
            {
                points = OptimizedChaikin(points, segment.closed);
            }
            CarPath cp = BuildCarPathFromPoints(points, segment.closed);
            cp.streetWidth = segment.roadWidth;
            cp.laneWidth = segment.laneWidth;
            carPaths.Add(cp);
        }
        UnityMainThreadDispatcher.Instance().Enqueue(() =>
        {
            UnityMainThreadDispatcher.Instance().Enqueue(
                InitAfterCarPathLoaded(initAfterCarPathLoaded, randomCarAmount)
            );
        });
    }

    /// <summary>
    /// Optimizes the given points using a parallelized Chaikin algorithm.
    /// </summary>
    public List<Vector3> OptimizedChaikin(List<Vector3> pts, bool closed)
    {
        return ChaikinParallel(pts, closed);
    }

    /// <summary>
    /// Performs a parallelized Chaikin subdivision on the given points.
    /// </summary>
    public List<Vector3> ChaikinParallel(List<Vector3> pts, bool closed)
    {
        if (pts == null || pts.Count < 3)
            return pts;
        int count = pts.Count;
        int newCount = closed ? count * 2 + 1 : (count - 2) * 2 + 2;
        Vector3[] newPts = new Vector3[newCount];
        if (closed)
        {
            System.Threading.Tasks.Parallel.For(0, count, i =>
            {
                Vector3 p0 = pts[i];
                Vector3 p1 = pts[(i + 1) % count];
                Vector3 p2 = pts[(i + 2) % count];
                Vector3 q = p0 + 0.75f * (p1 - p0);
                Vector3 r = p1 + 0.25f * (p2 - p1);
                newPts[i * 2] = q;
                newPts[i * 2 + 1] = r;
            });
            newPts[newCount - 1] = newPts[0];
        }
        else
        {
            newPts[0] = pts[0];
            System.Threading.Tasks.Parallel.For(0, count - 2, i =>
            {
                Vector3 p0 = pts[i];
                Vector3 p1 = pts[i + 1];
                Vector3 p2 = pts[i + 2];
                Vector3 q = p0 + 0.75f * (p1 - p0);
                Vector3 r = p1 + 0.25f * (p2 - p1);
                newPts[i * 2 + 1] = q;
                newPts[i * 2 + 2] = r;
            });
            newPts[newCount - 1] = pts[count - 1];
        }
        return newPts.ToList();
    }

    /// <summary>
    /// Builds a CarPath from a list of points.
    /// </summary>
    public CarPath BuildCarPathFromPoints(List<Vector3> points, bool closed)
    {
        CarPath cp = new CarPath("point_path");
        cp.isClosed = closed;
        Vector3 offset = new Vector3(0, -0.1f, 0);
        int count = points.Count;
        for (int i = 0; i < count; i++)
        {
            int nextIndex = closed ? (i + 1) % count : Mathf.Min(i + 1, count - 1);
            int prevIndex = closed ? (i - 1 + count) % count : Mathf.Max(i - 1, 0);
            Vector3 current = points[i] + offset;
            Vector3 nextPos = points[nextIndex] + offset;
            Vector3 prevPos = points[prevIndex] + offset;
            PathNode node = new PathNode
            {
                pos = current,
                rotation = Quaternion.LookRotation(nextPos - prevPos, Vector3.up)
            };
            cp.nodes.Add(node);
            cp.centerNodes.Add(node);
        }
        return cp;
    }

    /// <summary>
    /// Generates a random CarPath.
    /// </summary>
    public CarPath MakeRandomPath(int circuitIndex)
    {
        CarPath cp = new CarPath("random_circuit_" + circuitIndex);
        float rx = Random.Range(-startPointRandomRange.x, startPointRandomRange.x);
        float rz = Random.Range(-startPointRandomRange.y, startPointRandomRange.y);
        Vector3 randomOffset = new Vector3(rx, 0f, rz);
        Vector3 currentPos = startPos.position + randomOffset;
        currentPos.y = 0.2f;
        List<Vector3> points = new List<Vector3> { currentPos };
        float currentHeading = Random.Range(0f, 360f);
        int numIntermediate = ClosedCircuit ? (numSpans - 2) : numSpans;
        for (int i = 0; i < numIntermediate; i++)
        {
            if (i % turnChangeInterval == 0)
            {
                float delta = Random.Range(-turnInc, turnInc);
                currentHeading += delta;
            }
            Quaternion rot = Quaternion.Euler(0, currentHeading, 0);
            Vector3 direction = rot * Vector3.forward;
            currentPos += direction * spanDist;
            points.Add(currentPos);
        }
        if (ClosedCircuit)
        {
            if (points.Count >= 2)
            {
                Vector3 startTangent = (points[1] - points[0]).normalized;
                Vector3 adjustedSecondLast = points[0] - startTangent * spanDist;
                points.Add(adjustedSecondLast);
            }
            points.Add(points[0]);
        }
        for (int i = 0; i < smoothPathIter; i++)
        {
            points = OptimizedChaikin(points, ClosedCircuit);
        }
        return BuildCarPathFromPoints(points, ClosedCircuit);
    }

    /// <summary>
    /// Initializes scripts that implement IWaitCarPath after car paths are loaded.
    /// </summary>
    public IEnumerator InitAfterCarPathLoaded(GameObject[] scriptList, int randomCarAmount)
    {
        yield return new WaitUntil(() => SceneManager.GetActiveScene().isLoaded);
        yield return null;
        if (carPaths.Count > 0)
        {
            foreach (GameObject go in scriptList)
            {
                try
                {
                    if (go != null)
                    {
                        var script = go.GetComponent<IWaitCarPath>();
                        if (script != null)
                        {
                            script.Init();
                        }
                    }
                }
                catch (System.Exception e)
                {
                    Debug.LogError($"Could not initialize {go.name}: {e}");
                }
            }
            Spawner spawner = null;
            yield return new WaitUntil(() =>
            {
                spawner = FindObjectOfType<Spawner>();
                return spawner != null;
            });
            if (spawner != null && randomCarAmount != 0)
            {
                spawner.SpawnRandomCars(randomCarAmount);
            }
        }
        else
        {
            Debug.LogError("No CarPath loaded!");
        }
        yield return null;
    }
}
