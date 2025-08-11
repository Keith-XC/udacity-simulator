using Assets.WayPointSystem2;
using System.Collections.Generic;
using System.Linq;
using System.Threading.Tasks;
using UnityEngine;

public class RoadBuilder : MonoBehaviour, IWaitCarPath
{
    [Header("Path and WayPointManager")]
    public PathManager pathManager;
    public WayPointManager wayPointManager;
    public Spawner spawner;

    [Header("Road building params")]
    public bool doBuildRoad = true;
    public float roadOffsetW = 0.0f;
    public Texture2D customRoadTexture;
    public GameObject roadPrefabMesh;

    [Header("Terrain params (optional)")]
    public bool doErodeTerrain = true;
    public TerrainToolkit terToolkit;

    [Header("Intersection settings (based on intersections)")]
    public float intersectionNearDistance = 2f;
    public float intersectionClusterRadius = 3f;

    // Angle parameters for turns
    public float angleStraightThreshold = 15f;
    public float angleLeftMin = 60f, angleLeftMax = 120f;
    public float angleRightMin = 60f, angleRightMax = 120f;

    [Header("WayPoint settings")]
    public float waypointVerticalOffset = 1.0f;
    public float waypointSpacing = 1.0f;
    public float laneDivergenceAngle = 5.0f;
    public float defaultSpeed = 2f;

    [Header("Other parameters")]
    public float roadHeightIncrement = 0.01f;
    public int Cars = 50;

    private int roadCounter = 0;
    private List<Intersection> intersectionList = new List<Intersection>();

    [Header("Road Marking - Materials")]
    public Material edgeLineMaterial;
    public Material dashedLineMaterial;
    public Material solidLineMaterial;
    public float lineWidth = 0.1f;
    public float lineYOffset = 0.02f;
    public float doubleLineOffset = 0.15f;

    private struct MeshData
    {
        public Vector3[] vertices;
        public Vector2[] uv;
        public Vector3[] normals;
        public int[] triangles;
    }

    private void Start()
    {
        if (terToolkit != null && doErodeTerrain)
            terToolkit.SmoothTerrain(10, 1.0f);
    }

    /// <summary>
    /// Initializes the road building process by destroying previous road meshes and circuits, generating road meshes, lane circuits, intersections, turns, road markings, and optionally spawning vehicles.
    /// </summary>
    public void Init()
    {
        roadCounter = 0;
        if (!doBuildRoad) return;
        if (pathManager == null || pathManager.carPaths == null)
            return;
        DestroyRoad();
        if (wayPointManager != null && wayPointManager.Circuits != null)
        {
            foreach (var c in wayPointManager.Circuits)
                if (c != null)
                    DestroyImmediate(c.gameObject);
            wayPointManager.Circuits.Clear();
        }
        foreach (var carPath in pathManager.carPaths)
        {
            InitRoadParallel(carPath);
            GenerateLaneCircuits(carPath);
        }
        BuildIntersections();
        GenerateIntersectionBasedTurns();
        GenerateRoadMarkings();
        if (Cars != 0 && spawner != null)
            spawner.SpawnRandomCars(Cars);
    }

    #region RoadMesh-Erzeugung

    /// <summary>
    /// Generates the road mesh in a parallel task and instantiates it on the main thread.
    /// </summary>
    public void InitRoadParallel(CarPath path)
    {
        if (path == null) return;
        int nodeCount = path.nodes.Count;
        if (nodeCount < 2) return;
        Task.Run(() =>
        {
            bool isClosed = path.isClosed;
            int segmentCount = isClosed ? nodeCount : nodeCount - 1;
            int numVerts = (segmentCount + 1) * 2;
            Vector3[] vertices = new Vector3[numVerts];
            Vector2[] uv = new Vector2[numVerts];
            Vector3[] normals = new Vector3[numVerts];
            for (int i = 0; i < normals.Length; i++)
                normals[i] = Vector3.up;
            float cumulativeDistance = 0f;
            int vIndex = 0;
            Vector3 offsetUp = new Vector3(0, 0.2f, 0);
            for (int i = 0; i <= segmentCount; i++)
            {
                int indexA = i;
                if (indexA >= nodeCount)
                    indexA = nodeCount - 1;
                int nextIndex = isClosed ? (i + 1) % nodeCount : Mathf.Min(i + 1, nodeCount - 1);
                PathNode nodeA = path.nodes[indexA];
                PathNode nodeB = path.nodes[nextIndex];
                Vector3 posA = nodeA.pos + offsetUp;
                Vector3 posB = nodeB.pos + offsetUp;
                if (i > 0)
                    cumulativeDistance += Vector3.Distance(posA, posB);
                Vector3 vLength = posB - posA;
                Vector3 vWidth = Vector3.Cross(vLength, Vector3.up).normalized;
                Vector3 leftPos = posA + vWidth * (path.streetWidth + roadOffsetW);
                Vector3 rightPos = posA - vWidth * (path.streetWidth - roadOffsetW);
                vertices[vIndex] = leftPos;
                vertices[vIndex + 1] = rightPos;
                uv[vIndex] = new Vector2(cumulativeDistance * 0.2f, 0f);
                uv[vIndex + 1] = new Vector2(cumulativeDistance * 0.2f, 1f);
                vIndex += 2;
            }
            List<int> triIndices = new List<int>();
            int vertStart = 0;
            for (int i = 0; i < segmentCount; i++)
            {
                triIndices.Add(vertStart);
                triIndices.Add(vertStart + 2);
                triIndices.Add(vertStart + 1);
                triIndices.Add(vertStart + 2);
                triIndices.Add(vertStart + 3);
                triIndices.Add(vertStart + 1);
                vertStart += 2;
            }
            return new MeshData
            {
                vertices = vertices,
                uv = uv,
                normals = normals,
                triangles = triIndices.ToArray()
            };
        })
        .ContinueWith(task =>
        {
            MeshData data = task.Result;
            GameObject roadGO = Instantiate(roadPrefabMesh);
            roadGO.name = "RoadMesh_" + path.pathType;
            roadGO.tag = "road_mesh";
            Mesh mesh = new Mesh();
            mesh.vertices = data.vertices;
            mesh.uv = data.uv;
            mesh.normals = data.normals;
            mesh.triangles = data.triangles;
            mesh.RecalculateBounds();
            MeshFilter mf = roadGO.GetComponent<MeshFilter>();
            MeshRenderer mr = roadGO.GetComponent<MeshRenderer>();
            MeshCollider mc = roadGO.GetComponent<MeshCollider>();
            if (mf != null) mf.mesh = mesh;
            if (mc != null) mc.sharedMesh = mesh;
            if (mr != null && customRoadTexture != null)
                mr.material.mainTexture = customRoadTexture;
            float additionalYOffset = roadCounter * roadHeightIncrement;
            roadGO.transform.position += new Vector3(0, additionalYOffset, 0);
            roadCounter++;
        }, TaskScheduler.FromCurrentSynchronizationContext());
    }


    #endregion

    #region LaneCircuits-Erzeugung

    /// <summary>
    /// Generates one or more lane circuits based on the given CarPath.
    /// </summary>
    public void GenerateLaneCircuits(CarPath path)
    {
        if (path == null || path.centerNodes == null || path.centerNodes.Count == 0)
        {
            Debug.LogError("No CarPath or centerNodes available to generate Lane Circuits.");
            return;
        }
        List<PathNode> forwardNodes = GenerateInterpolatedNodes(path.centerNodes, path.isClosed, waypointSpacing);
        if (forwardNodes == null || forwardNodes.Count == 0)
        {
            Debug.LogError("Interpolation of centerNodes did not yield any points.");
            return;
        }
        List<PathNode> backwardNodes = new List<PathNode>(forwardNodes);
        backwardNodes.Reverse();
        float totalRoadWidth = 2 * path.streetWidth;
        int totalLanes = Mathf.FloorToInt(totalRoadWidth / path.laneWidth);
        if (totalLanes < 1) totalLanes = 1;
        int backwardCount = totalLanes / 2;
        int forwardCount = totalLanes - backwardCount;
        List<float> offsets = GetSymOffsets(totalLanes);
        for (int i = 0; i < totalLanes; i++)
        {
            float laneOffset = offsets[i] * path.laneWidth;
            if (i < backwardCount)
                CreateLaneCircuit(i, backwardNodes, laneOffset, false);
            else
                CreateLaneCircuit(i, forwardNodes, laneOffset, true);
        }
    }

    /// <summary>
    /// Generates interpolated nodes at equal intervals from the given center nodes.
    /// </summary>
    private List<PathNode> GenerateInterpolatedNodes(List<PathNode> nodes, bool isClosed, float spacing)
    {
        List<PathNode> interpNodes = new List<PathNode>();
        if (nodes == null || nodes.Count < 2)
            return interpNodes;
        interpNodes.Add(new PathNode { pos = nodes[0].pos, rotation = nodes[0].rotation });
        int segmentCount = isClosed ? nodes.Count : nodes.Count - 1;
        for (int i = 0; i < segmentCount; i++)
        {
            int nextIndex = isClosed ? (i + 1) % nodes.Count : i + 1;
            Vector3 start = nodes[i].pos;
            Vector3 end = nodes[nextIndex].pos;
            float segmentLength = Vector3.Distance(start, end);
            if (segmentLength < 0.001f)
                continue;
            int numSamples = Mathf.Max(1, Mathf.FloorToInt(segmentLength / spacing));
            for (int j = 1; j <= numSamples; j++)
            {
                float t = (j * spacing) / segmentLength;
                t = Mathf.Min(t, 1f);
                Vector3 pos = Vector3.Lerp(start, end, t);
                Quaternion rot = Quaternion.LookRotation(end - start);
                interpNodes.Add(new PathNode { pos = pos, rotation = rot });
            }
        }
        if (!isClosed)
        {
            int last = nodes.Count - 1;
            interpNodes.Add(new PathNode { pos = nodes[last].pos, rotation = nodes[last].rotation });
        }
        return interpNodes;
    }

    /// <summary>
    /// Creates a lane circuit from the given nodes with a lateral offset.
    /// </summary>
    private void CreateLaneCircuit(int laneIndex, List<PathNode> nodeList, float laneOffset, bool isForward)
    {
        GameObject circuitGO = new GameObject($"LaneCircuit_{laneIndex}");
        WayPointCircuit circuit = circuitGO.AddComponent<WayPointCircuit>();
        circuit.closedLoop = true;
        circuit.WayPoints = new List<WayPoint>();
        circuitGO.transform.parent = this.transform;
        foreach (var node in nodeList)
        {
            Vector3 lateral = node.rotation * Vector3.right;
            float angle = (laneOffset > 0) ? laneDivergenceAngle : (laneOffset < 0 ? -laneDivergenceAngle : 0f);
            lateral = Quaternion.Euler(0, angle, 0) * lateral;
            Vector3 wpPos = node.pos + lateral * laneOffset;
            wpPos.y += waypointVerticalOffset;
            GameObject wpGO = new GameObject($"Waypoint_{laneIndex}_{circuit.WayPoints.Count}");
            wpGO.transform.position = wpPos;
            wpGO.transform.rotation = node.rotation;
            wpGO.transform.parent = circuitGO.transform;
            WayPoint wp = wpGO.AddComponent<WayPoint>();
            wp.Position = wpPos;
            wp.Speed = defaultSpeed;
            wp.Number = circuit.WayPoints.Count;
            circuit.WayPoints.Add(wp);
        }
        if (wayPointManager != null)
        {
            if (wayPointManager.Circuits == null)
                wayPointManager.Circuits = new List<WayPointCircuit>();
            wayPointManager.Circuits.Add(circuit);
        }
    }


    /// <summary>
    /// Calculates symmetric offsets for lane placement.
    /// </summary>
    private List<float> GetSymOffsets(int totalLanes)
    {
        List<float> result = new List<float>();
        float mid = (totalLanes - 1) / 2f;
        for (int i = 0; i < totalLanes; i++)
            result.Add(i - mid);
        return result;
    }

    #endregion

    #region Intersection-Logik

    /// <summary>
    /// Clusters nearby waypoints into intersections and marks them as intersections.
    /// </summary>
    public void BuildIntersections()
    {
        intersectionList.Clear();
        List<WayPoint> allPoints = new List<WayPoint>();
        if (wayPointManager != null && wayPointManager.Circuits != null)
        {
            foreach (var c in wayPointManager.Circuits)
            {
                if (c != null && c.WayPoints != null)
                    allPoints.AddRange(c.WayPoints);
            }
        }
        var closePairs = new List<(WayPoint, WayPoint)>();
        for (int i = 0; i < allPoints.Count; i++)
        {
            for (int j = i + 1; j < allPoints.Count; j++)
            {
                float dist = Vector3.Distance(allPoints[i].Position, allPoints[j].Position);
                if (dist < intersectionNearDistance)
                    closePairs.Add((allPoints[i], allPoints[j]));
            }
        }
        HashSet<WayPoint> assigned = new HashSet<WayPoint>();
        foreach (var pair in closePairs)
        {
            WayPoint wpA = pair.Item1;
            WayPoint wpB = pair.Item2;
            if (assigned.Contains(wpA) && assigned.Contains(wpB))
                continue;
            Vector3 center = (wpA.Position + wpB.Position) * 0.5f;
            Intersection inters = new Intersection();
            inters.position = center;
            inters.wayPoints = FindPointsInRadius(allPoints, center, intersectionClusterRadius);
            foreach (var wp in inters.wayPoints)
            {
                assigned.Add(wp);
                wp.IsIntersection = true;
            }
            intersectionList.Add(inters);
        }
    }

    /// <summary>
    /// Returns all waypoints within the specified radius of a center point.
    /// </summary>
    private List<WayPoint> FindPointsInRadius(List<WayPoint> points, Vector3 center, float radius)
    {
        List<WayPoint> result = new List<WayPoint>();
        foreach (var p in points)
        {
            float dist = Vector3.Distance(p.Position, center);
            if (dist <= radius)
                result.Add(p);
        }
        return result;
    }

    /// <summary>
    /// Generates possible turn connections (left/right/straight) at each intersection.
    /// </summary>
    public void GenerateIntersectionBasedTurns()
    {
        foreach (var intersection in intersectionList)
        {
            List<LaneInfo> laneInfos = new List<LaneInfo>();
            foreach (var wp in intersection.wayPoints)
            {
                if (wp == null) continue;
                Vector3 dir = GetApproachDirection(wp);
                laneInfos.Add(new LaneInfo { wp = wp, direction = dir });
                wp.PossibleTurns = new List<PossibleTurn>();
            }
            for (int i = 0; i < laneInfos.Count; i++)
            {
                LaneInfo inc = laneInfos[i];
                Vector3 dA = inc.direction;
                for (int j = 0; j < laneInfos.Count; j++)
                {
                    if (i == j) continue;
                    LaneInfo outg = laneInfos[j];
                    Vector3 dB = outg.direction;
                    float angle = Vector3.Angle(dA, dB);
                    Vector3 cross = Vector3.Cross(dA, dB);
                    float sideSign = Mathf.Sign(Vector3.Dot(cross, Vector3.up));
                    IndicatorDirection turnDir = IndicatorDirection.Straight;
                    if (angle <= angleStraightThreshold)
                        turnDir = IndicatorDirection.Straight;
                    else
                    {
                        if (sideSign < 0 && angle >= angleLeftMin && angle <= angleLeftMax)
                            turnDir = IndicatorDirection.Left;
                        else if (sideSign > 0 && angle >= angleRightMin && angle <= angleRightMax)
                            turnDir = IndicatorDirection.Right;
                    }
                    if (turnDir != IndicatorDirection.Straight)
                    {
                        if (!inc.wp.PossibleTurns.Any(x => x.wayPoint == outg.wp))
                            inc.wp.PossibleTurns.Add(new PossibleTurn(outg.wp, turnDir));
                    }
                }
            }
        }
    }

    /// <summary>
    /// Returns the approach direction vector for the specified waypoint.
    /// </summary>
    private Vector3 GetApproachDirection(WayPoint wp)
    {
        WayPointCircuit circuit = FindCircuitOf(wp);
        if (circuit == null || circuit.WayPoints.Count < 2)
            return Vector3.forward;
        int index = circuit.WayPoints.IndexOf(wp);
        if (index < 0)
            return Vector3.forward;
        int prevIndex = index - 1;
        if (prevIndex < 0)
        {
            if (circuit.closedLoop)
                prevIndex = circuit.WayPoints.Count - 1;
            else
                return Vector3.forward;
        }
        Vector3 pPrev = circuit.WayPoints[prevIndex].Position;
        Vector3 pThis = wp.Position;
        return (pThis - pPrev).normalized;
    }

    private WayPointCircuit FindCircuitOf(WayPoint wp)
    {
        if (wayPointManager == null || wayPointManager.Circuits == null)
            return null;
        foreach (var c in wayPointManager.Circuits)
        {
            if (c != null && c.WayPoints.Contains(wp))
                return c;
        }
        return null;
    }

    #endregion

    #region Destroy / Regenerate

    /// <summary>
    /// Removes all road_mesh objects from the scene.
    /// </summary>
    public void DestroyRoad()
    {
        GameObject[] prev = GameObject.FindGameObjectsWithTag("road_mesh");
        foreach (GameObject g in prev)
            Destroy(g);
    }

    /// <summary>
    /// Regenerates lane circuits and road meshes, then rebuilds intersections and turn logic.
    /// </summary>
    [ContextMenu("Regenerate Lane Circuits")]
    public void RegenerateLaneCircuits()
    {
        if (pathManager == null || pathManager.carPaths == null)
        {
            Debug.LogWarning("No valid CarPath available.");
            return;
        }
        DestroyRoad();
        if (wayPointManager != null && wayPointManager.Circuits != null)
        {
            foreach (var circuit in wayPointManager.Circuits)
                if (circuit != null)
                    DestroyImmediate(circuit.gameObject);
            wayPointManager.Circuits.Clear();
        }
        foreach (var carPath in pathManager.carPaths)
        {
            InitRoadParallel(carPath);
            GenerateLaneCircuits(carPath);
        }
        BuildIntersections();
        GenerateIntersectionBasedTurns();
        Debug.Log("Road Mesh and Lane Circuits have been regenerated (Intersection-based).");
    }

    #endregion
    /// <summary>
    /// Generates lane markings as LineRenderers for each CarPath.
    /// </summary>
    public void GenerateRoadMarkings()
    {
        if (pathManager == null || pathManager.carPaths == null)
            return;
        foreach (var carPath in pathManager.carPaths)
        {
            int nodeCount = carPath.nodes.Count;
            if (nodeCount < 2) continue;
            float totalRoadWidth = 2f * carPath.streetWidth;
            int totalLanes = Mathf.FloorToInt(totalRoadWidth / carPath.laneWidth);
            if (totalLanes < 1) totalLanes = 1;
            bool isClosed = carPath.isClosed;
            CreateSingleLineRenderer("EdgeLine_Left_" + carPath.pathType, carPath, 0f, edgeLineMaterial, isClosed);
            CreateSingleLineRenderer("EdgeLine_Right_" + carPath.pathType, carPath, totalRoadWidth, edgeLineMaterial, isClosed);
            for (int k = 1; k < totalLanes; k++)
            {
                bool isCenterLine = (totalLanes % 2 == 0 && k == totalLanes / 2);
                if (totalLanes == 2)
                {
                    CreateSingleLineRenderer("CenterLine_Dashed_" + carPath.pathType, carPath, carPath.laneWidth, dashedLineMaterial, isClosed);
                }
                else
                {
                    if (isCenterLine)
                    {
                        CreateDoubleLineRenderer("CenterLine_Double_" + carPath.pathType, carPath, k * carPath.laneWidth, solidLineMaterial, isClosed);
                    }
                    else
                    {
                        CreateSingleLineRenderer("InnerLine_Dashed_" + carPath.pathType + "_k" + k, carPath, k * carPath.laneWidth, dashedLineMaterial, isClosed);
                    }
                }
            }
        }
    }

    /// <summary>
    /// Creates a single LineRenderer for road marking.
    /// </summary>
    private void CreateSingleLineRenderer(string lineName, CarPath carPath, float offsetFromLeft, Material mat, bool isClosed)
    {
        var go = new GameObject(lineName);
        go.transform.parent = this.transform;
        var lr = go.AddComponent<LineRenderer>();
        lr.material = mat;
        lr.widthMultiplier = lineWidth;
        lr.positionCount = carPath.nodes.Count;
        lr.useWorldSpace = true;
        lr.textureMode = LineTextureMode.Tile;
        lr.loop = isClosed;
        float totalWidth = 2f * carPath.streetWidth;
        for (int i = 0; i < carPath.nodes.Count; i++)
        {
            Vector3 pos = carPath.nodes[i].pos;
            pos.y += lineYOffset;
            Vector3 dir;
            if (i < carPath.nodes.Count - 1)
                dir = (carPath.nodes[i + 1].pos - carPath.nodes[i].pos).normalized;
            else
                dir = (carPath.nodes[i].pos - carPath.nodes[i - 1].pos).normalized;
            Vector3 cross = Vector3.Cross(dir, Vector3.up).normalized;
            Vector3 leftBoundary = pos + cross * carPath.streetWidth;
            Vector3 rightBoundary = pos - cross * carPath.streetWidth;
            float t = offsetFromLeft / totalWidth;
            Vector3 linePos = Vector3.Lerp(leftBoundary, rightBoundary, t);
            lr.SetPosition(i, linePos);
        }
    }

    /// <summary>
    /// Creates a double LineRenderer by creating two parallel single LineRenderers.
    /// </summary>
    private void CreateDoubleLineRenderer(string lineName, CarPath carPath, float offsetFromLeft, Material mat, bool isClosed)
    {
        CreateSingleLineRenderer(lineName + "_A", carPath, offsetFromLeft, mat, isClosed);
        CreateSingleLineRenderer(lineName + "_B", carPath, offsetFromLeft - doubleLineOffset, mat, isClosed);
    }
}