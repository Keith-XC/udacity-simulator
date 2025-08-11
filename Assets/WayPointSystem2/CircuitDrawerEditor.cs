#if UNITY_EDITOR
using Assets.WayPointSystem2;
using UnityEditor;
using UnityEngine;
using System.Collections.Generic;

[CustomEditor(typeof(CircuitDrawer))]
public class CircuitDrawerEditor : Editor
{
    /// <summary>
    /// Zeichnet den Standard-Inspector und fügt einen Button zum Erstellen eines neuen Circuits hinzu.
    /// </summary>
    public override void OnInspectorGUI()
    {
        DrawDefaultInspector();
        CircuitDrawer drawer = (CircuitDrawer)target;
        if (GUILayout.Button("Create New Circuit"))
        {
            CreateCircuit(drawer);
        }
    }

    /// <summary>
    /// Verarbeitet Scene-View-Ereignisse, um Circuit-Punkte per Strg+Linksklick hinzuzufügen.
    /// </summary>
    private void OnSceneGUI()
    {
        CircuitDrawer drawer = (CircuitDrawer)target;
        Event e = Event.current;
        if (e.type == EventType.MouseDown && e.button == 0 && e.control)
        {
            Ray ray = HandleUtility.GUIPointToWorldRay(e.mousePosition);
            if (Physics.Raycast(ray, out RaycastHit hit, Mathf.Infinity, drawer.roadLayer))
            {
                Vector3 hitPoint = hit.point;
                Undo.RecordObject(drawer, "Add Circuit Point");
                drawer.points.Add(drawer.transform.InverseTransformPoint(hitPoint));
                EditorUtility.SetDirty(drawer);
            }
            e.Use();
        }
    }

    /// <summary>
    /// Erzeugt einen neuen Circuit basierend auf den in CircuitDrawer definierten Punkten.
    /// Falls smoothing aktiviert ist, werden zusätzlich interpolierte Wegpunkte erzeugt.
    /// </summary>
    private void CreateCircuit(CircuitDrawer drawer)
    {
        GameObject prefab = Resources.Load<GameObject>(drawer.circuitPrefabPath);
        if (prefab == null)
        {
            Debug.LogError("Circuit prefab not found at: " + drawer.circuitPrefabPath);
            return;
        }
        GameObject circuitObj = (GameObject)PrefabUtility.InstantiatePrefab(prefab);
        circuitObj.transform.position = drawer.transform.position;
        WayPointCircuit circuit = circuitObj.GetComponent<WayPointCircuit>();
        if (circuit == null)
        {
            Debug.LogError("The prefab does not have a WayPointCircuit component.");
            return;
        }
        circuit.WayPoints.Clear();

        // Entscheide, ob zusätzliche Wegpunkte mittels Smoothing eingefügt werden sollen
        List<Vector3> pointsToUse;
        if (drawer.useSmoothing)
        {
            pointsToUse = GenerateInterpolatedPoints(drawer.points, drawer.closedLoop, drawer.smoothingSpacing);
        }
        else
        {
            pointsToUse = new List<Vector3>(drawer.points);
        }

        // Erstelle für jeden (interpolierten) Punkt einen neuen Waypoint
        for (int i = 0; i < pointsToUse.Count; i++)
        {
            GameObject wpObj = new GameObject("Waypoint_" + i);
            Vector3 worldPos = drawer.transform.TransformPoint(pointsToUse[i]);
            wpObj.transform.position = worldPos;

            // Rotation anhand des vorherigen Punkts bestimmen (falls vorhanden)
            if (i > 0)
            {
                Vector3 prevWorldPos = drawer.transform.TransformPoint(pointsToUse[i - 1]);
                wpObj.transform.rotation = Quaternion.LookRotation(worldPos - prevWorldPos);
            }
            else if (pointsToUse.Count > 1)
            {
                Vector3 nextWorldPos = drawer.transform.TransformPoint(pointsToUse[i + 1]);
                wpObj.transform.rotation = Quaternion.LookRotation(nextWorldPos - worldPos);
            }
            wpObj.transform.parent = circuitObj.transform;
            WayPoint wp = wpObj.AddComponent<WayPoint>();
            circuit.WayPoints.Add(wp);
        }
        circuit.closedLoop = drawer.closedLoop;
        Undo.RegisterCreatedObjectUndo(circuitObj, "Create Circuit");
    }

    /// <summary>
    /// Erzeugt mittels linearer Interpolation Zwischenpunkte zwischen den Originalpunkten.
    /// </summary>
    /// <param name="points">Liste der Originalpunkte (im lokalen Raum).</param>
    /// <param name="closed">Gibt an, ob der Circuit geschlossen ist.</param>
    /// <param name="spacing">Gewünschter Abstand (Mindestabstand) zwischen den Wegpunkten.</param>
    /// <returns>Liste der interpolierten Punkte inklusive der Originalpunkte.</returns>
    private List<Vector3> GenerateInterpolatedPoints(List<Vector3> points, bool closed, float spacing)
    {
        List<Vector3> interpPoints = new List<Vector3>();
        if (points == null || points.Count < 2)
            return new List<Vector3>(points);

        int count = points.Count;
        // Bearbeite alle Segmente zwischen aufeinanderfolgenden Punkten
        for (int i = 0; i < count - 1; i++)
        {
            Vector3 start = points[i];
            Vector3 end = points[i + 1];
            // Bei der ersten Strecke den Startpunkt hinzufügen
            if (i == 0)
                interpPoints.Add(start);

            float segLength = Vector3.Distance(start, end);
            int numSamples = Mathf.Max(1, Mathf.FloorToInt(segLength / spacing));

            for (int j = 1; j < numSamples; j++)
            {
                float t = (float)j / numSamples;
                Vector3 pos = Vector3.Lerp(start, end, t);
                interpPoints.Add(pos);
            }
            interpPoints.Add(end);
        }

        // Falls der Circuit geschlossen ist, bearbeite auch das Segment vom letzten zum ersten Punkt
        if (closed)
        {
            Vector3 start = points[count - 1];
            Vector3 end = points[0];
            float segLength = Vector3.Distance(start, end);
            int numSamples = Mathf.Max(1, Mathf.FloorToInt(segLength / spacing));
            for (int j = 1; j < numSamples; j++)
            {
                float t = (float)j / numSamples;
                Vector3 pos = Vector3.Lerp(start, end, t);
                interpPoints.Add(pos);
            }
        }

        return interpPoints;
    }
}
#endif
