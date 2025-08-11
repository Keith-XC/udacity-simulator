using System;
using System.Collections;
using UnityEngine;
#if UNITY_EDITOR
using UnityEditor;
#endif

namespace UnityStandardAssets.Utility
{
    public class WaypointCircuit : MonoBehaviour
    {
        [Header("Waypoint Circuit Einstellungen")]
        [Tooltip("Gibt an, ob der Pfad geschlossen (Loop) sein soll oder nicht.")]
        public bool closedLoop = true;

        [SerializeField] private bool smoothRoute = true;
        public float editorVisualisationSubsteps = 100;
        public WaypointList waypointList = new WaypointList();
        public int baseSpeed;
        private Vector3[] points;
        private float[] distances;
        public float Length { get; private set; }
        private int numPoints;
        public Transform[] Waypoints { get { return waypointList.items; } }
        private int p0n, p1n, p2n, p3n;
        private float i;
        private Vector3 P0, P1, P2, P3;

        private void Awake()
        {
            if (Waypoints.Length > 1) CachePositionsAndDistances();
            numPoints = Waypoints.Length;
        }

        public RoutePoint GetRoutePoint(float dist)
        {
            if (!closedLoop && dist >= Length)
            {
                Vector3 endPos = GetRoutePosition(Length);
                return new RoutePoint(endPos, Vector3.zero);
            }
            Vector3 p1 = GetRoutePosition(dist);
            Vector3 p2 = GetRoutePosition(dist + 0.1f);
            Vector3 delta = p2 - p1;
            return new RoutePoint(p1, delta.normalized);
        }

        public Vector3 GetRoutePosition(float dist)
        {
            if (Length == 0 && distances != null && distances.Length > 0)
                Length = distances[distances.Length - 1];
            if (closedLoop) dist = Mathf.Repeat(dist, Length);
            else dist = Mathf.Clamp(dist, 0, Length);

            int point = 0;
            while (point < distances.Length && distances[point] < dist) point++;
            point = Mathf.Clamp(point, 1, distances.Length - 1);

            p1n = point - 1;
            p2n = point;
            i = Mathf.InverseLerp(distances[p1n], distances[p2n], dist);

            if (smoothRoute)
            {
                if (closedLoop)
                {
                    p0n = (point - 2 + numPoints) % numPoints;
                    p1n = (point - 1 + numPoints) % numPoints;
                    p2n = (point + 0) % numPoints;
                    p3n = (point + 1) % numPoints;
                }
                else
                {
                    p0n = Mathf.Max(point - 2, 0);
                    p1n = Mathf.Max(point - 1, 0);
                    p2n = Mathf.Clamp(point, 0, numPoints - 1);
                    p3n = Mathf.Min(point + 1, numPoints - 1);
                }
                P0 = points[p0n];
                P1 = points[p1n];
                P2 = points[p2n];
                P3 = points[p3n];
                return CatmullRom(P0, P1, P2, P3, i);
            }
            else
            {
                return Vector3.Lerp(points[p1n], points[p2n], i);
            }
        }

        private Vector3 CatmullRom(Vector3 p0, Vector3 p1, Vector3 p2, Vector3 p3, float t)
        {
            return 0.5f * (
                (2f * p1) +
                (-p0 + p2) * t +
                (2f * p0 - 5f * p1 + 4f * p2 - p3) * t * t +
                (-p0 + 3f * p1 - 3f * p2 + p3) * t * t * t
            );
        }

        private void CachePositionsAndDistances()
        {
            numPoints = Waypoints.Length;
            if (numPoints < 2) return;
            if (closedLoop)
            {
                points = new Vector3[numPoints + 1];
                distances = new float[numPoints + 1];
            }
            else
            {
                points = new Vector3[numPoints];
                distances = new float[numPoints];
            }
            float accumulateDistance = 0f;
            for (int i = 0; i < numPoints; i++) points[i] = Waypoints[i].position;
            if (closedLoop) points[numPoints] = Waypoints[0].position;
            distances[0] = 0f;
            for (int i = 1; i < points.Length; i++)
            {
                if (!closedLoop && i >= numPoints) break;
                float segmentDistance = (points[i] - points[i - 1]).magnitude;
                accumulateDistance += segmentDistance;
                distances[i] = accumulateDistance;
            }
            Length = distances[distances.Length - 1];
        }

        private void OnDrawGizmos() { DrawGizmos(false); }
        private void OnDrawGizmosSelected() { DrawGizmos(true); }

        private void DrawGizmos(bool selected)
        {
            waypointList.circuit = this;
            Transform[] waypointArray = Waypoints;
            if (waypointArray == null || waypointArray.Length < 2) return;
            CachePositionsAndDistances();
            Gizmos.color = selected ? Color.red : new Color(1, 1, 0, 0.5f);
            if (smoothRoute)
            {
                Vector3 prev = GetRoutePosition(0);
                int steps = Mathf.CeilToInt(editorVisualisationSubsteps);
                for (int s = 1; s <= steps; s++)
                {
                    float dist = (Length / steps) * s;
                    Vector3 next = GetRoutePosition(dist);
                    Gizmos.DrawLine(prev, next);
                    prev = next;
                }
                if (closedLoop) Gizmos.DrawLine(prev, GetRoutePosition(0));
            }
            else
            {
                for (int i = 0; i < waypointArray.Length - 1; i++)
                    Gizmos.DrawLine(waypointArray[i].position, waypointArray[i + 1].position);
                if (closedLoop) Gizmos.DrawLine(waypointArray[waypointArray.Length - 1].position, waypointArray[0].position);
            }
        }

        [Serializable]
        public class WaypointList
        {
            public WaypointCircuit circuit;
            public Transform[] items = new Transform[0];
        }

        public struct RoutePoint
        {
            public Vector3 position;
            public Vector3 direction;
            public RoutePoint(Vector3 position, Vector3 direction)
            {
                this.position = position;
                this.direction = direction;
            }
        }
    }
}

namespace UnityStandardAssets.Utility.Inspector
{
#if UNITY_EDITOR
    [CustomPropertyDrawer(typeof(WaypointCircuit.WaypointList))]
    public class WaypointListDrawer : PropertyDrawer
    {
        private float lineHeight = 18;
        private float spacing = 4;

        public override void OnGUI(Rect position, SerializedProperty property, GUIContent label)
        {
            EditorGUI.BeginProperty(position, label, property);
            float x = position.x;
            float y = position.y;
            float inspectorWidth = position.width;
            var items = property.FindPropertyRelative("items");
            var titles = new string[] { "Transform", "", "", "" };
            var props = new string[] { "transform", "^", "v", "-" };
            var widths = new float[] { 0.7f, 0.1f, 0.1f, 0.1f };
            float lineHeight = 18;
            float spacing = 4;
            float rowX = x;
            for (int n = 0; n < titles.Length; ++n)
            {
                float w = widths[n] * inspectorWidth;
                Rect rect = new Rect(rowX, y, w, lineHeight);
                EditorGUI.LabelField(rect, titles[n]);
                rowX += w;
            }
            y += lineHeight + spacing;
            if (items.arraySize > 0)
            {
                for (int i = 0; i < items.arraySize; ++i)
                {
                    var item = items.GetArrayElementAtIndex(i);
                    rowX = x;
                    for (int n = 0; n < props.Length; ++n)
                    {
                        float w = widths[n] * inspectorWidth;
                        Rect rect = new Rect(rowX, y, w, lineHeight);
                        rowX += w;
                        if (n == 0)
                        {
                            EditorGUI.ObjectField(rect, item.objectReferenceValue, typeof(Transform), true);
                        }
                        else
                        {
                            if (GUI.Button(rect, props[n]))
                            {
                                switch (props[n])
                                {
                                    case "-":
                                        items.DeleteArrayElementAtIndex(i);
                                        items.DeleteArrayElementAtIndex(i);
                                        break;
                                    case "v":
                                        if (i < items.arraySize - 1) items.MoveArrayElement(i, i + 1);
                                        break;
                                    case "^":
                                        if (i > 0) items.MoveArrayElement(i, i - 1);
                                        break;
                                }
                            }
                        }
                    }
                    y += lineHeight + spacing;
                }
            }
            else
            {
                var addButtonRect = new Rect(x, y, inspectorWidth, lineHeight);
                if (GUI.Button(addButtonRect, "+"))
                    items.InsertArrayElementAtIndex(items.arraySize);
                y += lineHeight + spacing;
            }
            var addAllButtonRect = new Rect(x, y, inspectorWidth, lineHeight);
            if (GUI.Button(addAllButtonRect, "Assign using all child objects"))
            {
                var circuit = property.FindPropertyRelative("circuit").objectReferenceValue as WaypointCircuit;
                if (circuit != null)
                {
                    var children = new Transform[circuit.transform.childCount];
                    int n = 0;
                    foreach (Transform child in circuit.transform) children[n++] = child;
                    System.Array.Sort(children, new TransformNameComparer());
                    circuit.waypointList.items = new Transform[children.Length];
                    for (n = 0; n < children.Length; ++n) circuit.waypointList.items[n] = children[n];
                }
            }
            y += lineHeight + spacing;
            var renameButtonRect = new Rect(x, y, inspectorWidth, lineHeight);
            if (GUI.Button(renameButtonRect, "Auto Rename numerically from this order"))
            {
                var circuit = property.FindPropertyRelative("circuit").objectReferenceValue as WaypointCircuit;
                if (circuit != null)
                {
                    int n = 0;
                    foreach (Transform child in circuit.waypointList.items)
                        child.name = "Waypoint "+ circuit.name +" "+ (n++).ToString("000");
                }
            }
            y += lineHeight + spacing;
            EditorGUI.EndProperty();
        }

        public override float GetPropertyHeight(SerializedProperty property, GUIContent label)
        {
            SerializedProperty items = property.FindPropertyRelative("items");
            float lineAndSpace = lineHeight + spacing;
            return 40 + (items.arraySize * lineAndSpace) + lineAndSpace;
        }

        public class TransformNameComparer : System.Collections.IComparer
        {
            public int Compare(object x, object y)
            {
                return ((Transform)x).name.CompareTo(((Transform)y).name);
            }
        }
    }
#endif
}
