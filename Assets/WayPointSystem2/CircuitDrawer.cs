using System.Collections.Generic;
using UnityEngine;

[ExecuteAlways]
public class CircuitDrawer : MonoBehaviour
{
    [Tooltip("Liste der Punkte, die den Circuit definieren (in lokalen Koordinaten)")]
    public List<Vector3> points = new List<Vector3>();

    [Tooltip("Soll der Circuit geschlossen sein?")]
    public bool closedLoop = false;

    [Tooltip("Pfad des Circuit-Prefabs in Resources (z. B. 'Objects/WayPointCircuit')")]
    public string circuitPrefabPath = "Objects/WayPointCircuit";

    [Tooltip("Layer, der als Straße genutzt wird (z. B. 'Road')")]
    public LayerMask roadLayer;

    public bool useSmoothing;
    public float smoothingSpacing = 1.0f;


    private void OnDrawGizmos()
    {
        if (points == null || points.Count < 2)
            return;

        Gizmos.color = Color.yellow;
        for (int i = 0; i < points.Count - 1; i++)
        {
            Gizmos.DrawLine(transform.TransformPoint(points[i]), transform.TransformPoint(points[i + 1]));
        }
        if (closedLoop && points.Count > 1)
        {
            Gizmos.DrawLine(transform.TransformPoint(points[points.Count - 1]), transform.TransformPoint(points[0]));
        }
    }
}