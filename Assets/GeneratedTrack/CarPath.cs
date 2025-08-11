using System.Collections.Generic;
using UnityEngine;
using System.Threading.Tasks;

public class PathNode
{
    public Vector3 pos;
    public Quaternion rotation;
    public string activity;
}

public class CarPath
{
    public List<PathNode> nodes;
    public List<PathNode> centerNodes;
    public UnityEngine.AI.NavMeshPath navMeshPath;
    public int iActiveSpan = 0;
    public string pathType = "none";
    public bool isClosed = true;
    public float streetWidth = 5;
    public float laneWidth = 5;

    #region Konstruktoren

    public CarPath()
    {
        nodes = new List<PathNode>();
        centerNodes = new List<PathNode>();
        navMeshPath = new UnityEngine.AI.NavMeshPath();
        ResetActiveSpan();
    }

    public CarPath(string pathType) : this()
    {
        this.pathType = pathType;
    }

    #endregion

    #region Span-Logik

    public void ResetActiveSpan(bool sign = true)
    {
        if (nodes == null || nodes.Count < 2)
        {
            iActiveSpan = 0;
            return;
        }
        iActiveSpan = sign ? 0 : Mathf.Max(0, nodes.Count - 2);
    }

    public void GetClosestSpan(Vector3 carPos)
    {
        if (nodes == null || nodes.Count == 0)
        {
            iActiveSpan = 0;
            return;
        }
        float minDistance = float.MaxValue;
        int minDistanceIndex = -1;
        for (int i = 0; i < nodes.Count; i++)
        {
            float dist = Vector3.Distance(nodes[i].pos, carPos);
            if (dist < minDistance)
            {
                minDistance = dist;
                minDistanceIndex = i;
            }
        }
        iActiveSpan = (minDistanceIndex >= nodes.Count - 1)
            ? Mathf.Max(0, nodes.Count - 2)
            : minDistanceIndex;
    }

    public PathNode GetActiveNode()
    {
        if (nodes == null) return null;
        return (iActiveSpan >= 0 && iActiveSpan < nodes.Count) ? nodes[iActiveSpan] : null;
    }

    #endregion

    #region Zugriff auf Nodes

    public List<PathNode> GetPathNodes()
    {
        return centerNodes;
    }


    public void SmoothPathParallel(float factor = 0.5f)
    {
        if (nodes == null || nodes.Count < 3)
            return;

        Vector3[] newPositions = new Vector3[nodes.Count];
        newPositions[0] = nodes[0].pos;
        newPositions[nodes.Count - 1] = nodes[nodes.Count - 1].pos;

        Parallel.For(1, nodes.Count - 1, i =>
        {
            Vector3 p = nodes[i - 1].pos;
            Vector3 c = nodes[i].pos;
            Vector3 n = nodes[i + 1].pos;
            LineSeg3d seg = new LineSeg3d(ref p, ref n);
            LineSeg3d.SegResult segRes = new LineSeg3d.SegResult();
            Vector3 closestP = seg.ClosestPointOnSegmentTo(ref c, ref segRes);
            Vector3 dIntersect = closestP - c;
            newPositions[i] = c + dIntersect.normalized * factor;
        });
        for (int i = 1; i < nodes.Count - 1; i++)
        {
            nodes[i].pos = newPositions[i];
        }
    }
    #endregion
}
