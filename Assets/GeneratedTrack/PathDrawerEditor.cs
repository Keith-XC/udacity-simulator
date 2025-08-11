#if UNITY_EDITOR
using UnityEngine;
using UnityEditor;
using System.Collections.Generic;

[CustomEditor(typeof(PathDrawer))]
public class PathDrawerEditor : Editor
{
    /// <summary>
    /// Draws the default inspector and adds a button to add a new segment.
    /// </summary>
    public override void OnInspectorGUI()
    {
        DrawDefaultInspector();

        PathDrawer drawer = (PathDrawer)target;
        if (GUILayout.Button("Add New Segment"))
        {
            drawer.AddNewSegment();
        }
    }

    /// <summary>
    /// Handles scene view events to add path points with Ctrl + left mouse click.
    /// </summary>
    void OnSceneGUI()
    {
        PathDrawer drawer = (PathDrawer)target;
        Event e = Event.current;

        // When Ctrl + Left Mouse Button is pressed, add a new point
        if (e.type == EventType.MouseDown && e.button == 0 && e.control)
        {
            Ray ray = HandleUtility.GUIPointToWorldRay(e.mousePosition);
            // Define a plane (e.g. the XZ-plane at Y = drawer.transform.position.y)
            Plane plane = new Plane(Vector3.up, drawer.transform.position);
            float enter;
            if (plane.Raycast(ray, out enter))
            {
                Vector3 hitPoint = ray.GetPoint(enter);
                // Convert the hit point to local coordinates
                Vector3 localPoint = drawer.transform.InverseTransformPoint(hitPoint);

                // Ensure that a segment exists
                if (drawer.segments == null)
                    drawer.segments = new List<PathSegment>();
                if (drawer.segments.Count == 0)
                {
                    drawer.AddNewSegment();
                }
                // Add the point to the active segment
                PathSegment seg = drawer.segments[drawer.activeSegment];
                Undo.RecordObject(drawer, "Add Path Point");
                seg.points.Add(localPoint);
                EditorUtility.SetDirty(drawer);
            }
            e.Use();
        }
    }
}
#endif
