using System.Collections.Generic;
using UnityEngine;

[System.Serializable]
public class PathSegment
{
    [Tooltip("List of points defining this sub-path (in local coordinates)")]
    public List<Vector3> points = new List<Vector3>();

    [Tooltip("Should this sub-path be closed?")]
    public bool closed = false;

    public float roadWidth = 5;
    public float laneWidth = 5;
}

[ExecuteAlways]
public class PathDrawer : MonoBehaviour
{
    [Tooltip("List of sub-segments that define the entire path")]
    public List<PathSegment> segments = new List<PathSegment>();

    [Tooltip("Index of the currently active segment to which points are added")]
    public int activeSegment = 0;

#if UNITY_EDITOR
    /// <summary>
    /// Adds a new segment to the path.
    /// </summary>
    [ContextMenu("Add New Segment")]
    public void AddNewSegment()
    {
        if (segments == null)
            segments = new List<PathSegment>();
        segments.Add(new PathSegment());
        activeSegment = segments.Count - 1;
    }
#endif

    /// <summary>
    /// Draws all path segments in the scene.
    /// </summary>
    void OnDrawGizmos()
    {
        if (segments == null || segments.Count == 0)
            return;

        Gizmos.color = Color.green;
        foreach (var segment in segments)
        {
            if (segment.points == null || segment.points.Count < 2)
                continue;

            for (int i = 0; i < segment.points.Count - 1; i++)
            {
                Vector3 p1 = transform.TransformPoint(segment.points[i]);
                Vector3 p2 = transform.TransformPoint(segment.points[i + 1]);
                Gizmos.DrawLine(p1, p2);
            }
            if (segment.closed)
            {
                Vector3 pLast = transform.TransformPoint(segment.points[segment.points.Count - 1]);
                Vector3 pFirst = transform.TransformPoint(segment.points[0]);
                Gizmos.DrawLine(pLast, pFirst);
            }
        }
    }
}
