using System;
using System.Collections.Generic;
using UnityEditor;
using UnityEngine;

namespace Assets.WayPointSystem2
{
    [ExecuteAlways]
    public class WayPointCircuit : MonoBehaviour
    {
        public bool closedLoop = false;

        [SerializeField] private bool smoothRoute = true;

        [Tooltip("Gibt an, in welchen Abständen Zwischenpunkte gezeichnet werden sollen.")]
        public float distancePerSegmentStep = 0.5f;

        public List<WayPoint> WayPoints = new List<WayPoint>();
        public int baseSpeed = 2;
        public float Length { get; set; }

#if UNITY_EDITOR
        private void Update()
        {
            if (!Application.isPlaying)
            {
                for (int i = 0; i < WayPoints.Count; i++)
                {
                    var go = WayPoints[i].gameObject;
                    go.name = $"Waypoint {gameObject.name} {i:D3}";
                    WayPoints[i].Number = i;
                }
                RefreshWayPoints();
            }
        }

        private void OnValidate()
        {
            if (!Application.isPlaying)
            {
                RefreshWayPoints();
            }
        }
#endif

        private void RefreshWayPoints()
        {
            WayPoints.Clear();
            for (int i = 0; i < transform.childCount; i++)
            {
                Transform child = transform.GetChild(i);
                var wp = child.GetComponent<WayPoint>();
                if (wp != null)
                {
                    wp.Speed = baseSpeed;
                    WayPoints.Add(wp);
                }
            }
        }

        [ContextMenu("Reverse And Rename Waypoints")]
        private void ReverseAndRenameWaypoints()
        {
            WayPoints.Reverse();

            for (int i = 0; i < WayPoints.Count; i++)
            {
                WayPoints[i].transform.SetSiblingIndex(i);
            }

            for (int i = 0; i < WayPoints.Count; i++)
            {
                var go = WayPoints[i].gameObject;
                go.name = $"Waypoint {gameObject.name} {i:D3}";
                WayPoints[i].Number = i;
            }

            RefreshWayPoints();
        }

        /// <summary>
        /// Fügt in bestehenden Circuits zusätzliche Waypoints mithilfe des Smoothings (Catmull-Rom) hinzu.
        /// </summary>
#if UNITY_EDITOR
        [ContextMenu("Add Smooth Waypoints")]
        public void AddSmoothWaypoints()
        {
            if (WayPoints.Count < 2)
            {
                Debug.LogWarning("Es sind mindestens zwei Waypoints erforderlich, um zusätzliche Punkte zu erzeugen.");
                return;
            }

            // Erstelle eine Liste der Originalpositionen der vorhandenen Waypoints
            List<Vector3> originalPositions = new List<Vector3>();
            foreach (var wp in WayPoints)
            {
                originalPositions.Add(wp.Position);
            }

            // Lokale Hilfsfunktion, um bei offenen bzw. geschlossenen Circuits den Index korrekt zu behandeln.
            Vector3 GetPos(int index)
            {
                int count = originalPositions.Count;
                if (closedLoop)
                {
                    index = (index % count + count) % count;
                    return originalPositions[index];
                }
                else
                {
                    if (index < 0) return originalPositions[0];
                    if (index >= count) return originalPositions[count - 1];
                    return originalPositions[index];
                }
            }

            List<Vector3> newPositions = new List<Vector3>();
            int countPoints = originalPositions.Count;

            // Für jedes Segment zwischen zwei benachbarten Originalpunkten:
            for (int i = 0; i < countPoints - 1; i++)
            {
                // Füge den Originalpunkt hinzu
                newPositions.Add(originalPositions[i]);

                Vector3 p0 = GetPos(i - 1);
                Vector3 p1 = originalPositions[i];
                Vector3 p2 = originalPositions[i + 1];
                Vector3 p3 = GetPos(i + 2);

                float segLength = Vector3.Distance(p1, p2);
                int subSteps = Mathf.Max(1, Mathf.RoundToInt(segLength / distancePerSegmentStep));
                // Füge interpolierte Punkte ein (ohne Duplikate des Endpunkts)
                for (int step = 1; step < subSteps; step++)
                {
                    float t = (float)step / subSteps;
                    Vector3 newPos = GetCatmullRomPosition(t, p0, p1, p2, p3);
                    newPositions.Add(newPos);
                }
            }

            if (!closedLoop)
            {
                // Bei offenen Circuits den letzten Originalpunkt hinzufügen
                newPositions.Add(originalPositions[countPoints - 1]);
            }
            else
            {
                // Bei geschlossenen Circuits: Letzten Punkt hinzufügen und zusätzlich das letzte Segment zwischen letztem und erstem Punkt interpolieren
                newPositions.Add(originalPositions[countPoints - 1]);

                Vector3 p0 = GetPos(countPoints - 2);
                Vector3 p1 = originalPositions[countPoints - 1];
                Vector3 p2 = originalPositions[0];
                Vector3 p3 = GetPos(1);

                float segLength = Vector3.Distance(p1, p2);
                int subSteps = Mathf.Max(1, Mathf.RoundToInt(segLength / distancePerSegmentStep));
                for (int step = 1; step < subSteps; step++)
                {
                    float t = (float)step / subSteps;
                    Vector3 newPos = GetCatmullRomPosition(t, p0, p1, p2, p3);
                    newPositions.Add(newPos);
                }
            }

            // Lösche alle bisherigen Waypoint-Objekte (Childs)
            for (int i = transform.childCount - 1; i >= 0; i--)
            {
                DestroyImmediate(transform.GetChild(i).gameObject);
            }

            // Erstelle neue Waypoint-Objekte basierend auf der neuen Punktfolge
            for (int i = 0; i < newPositions.Count; i++)
            {
                GameObject wpGO = new GameObject($"Waypoint {gameObject.name} {i:D3}");
                wpGO.transform.position = newPositions[i];
                wpGO.transform.parent = transform;
                wpGO.AddComponent<WayPoint>();
            }

            RefreshWayPoints();
        }
#endif

        private void OnDrawGizmos()
        {
            DrawGizmos(false);
        }

        private void OnDrawGizmosSelected()
        {
            DrawGizmos(true);
        }

        private void DrawGizmos(bool selected)
        {
            if (WayPoints.Count < 2) return;

            Gizmos.color = selected ? Color.red : new Color(1, 1, 0, 0.5f);

            if (!smoothRoute)
            {
                for (int i = 0; i < WayPoints.Count - 1; i++)
                {
                    DrawArrowGizmo(WayPoints[i].Position, WayPoints[i + 1].Position);
                }
                if (closedLoop && WayPoints.Count > 1)
                {
                    DrawArrowGizmo(WayPoints[^1].Position, WayPoints[0].Position);
                }
            }
            else
            {
                DrawSmoothRoute();
            }
        }

        private void DrawSmoothRoute()
        {
            for (int i = 0; i < WayPoints.Count; i++)
            {
                Vector3 p0 = GetWaypointPosition(i - 1);
                Vector3 p1 = GetWaypointPosition(i);
                Vector3 p2 = GetWaypointPosition(i + 1);
                Vector3 p3 = GetWaypointPosition(i + 2);

                float distanceBetweenP1AndP2 = Vector3.Distance(p1, p2);
                int subSteps = Mathf.Max(1, Mathf.RoundToInt(distanceBetweenP1AndP2 / distancePerSegmentStep));

                Vector3 lastPos = p1;

                for (int step = 1; step <= subSteps; step++)
                {
                    float t = step / (float)subSteps;
                    Vector3 newPos = GetCatmullRomPosition(t, p0, p1, p2, p3);
                    DrawArrowGizmo(lastPos, newPos);
                    lastPos = newPos;
                }
            }
        }

        private Vector3 GetWaypointPosition(int index)
        {
            if (WayPoints.Count == 0)
            {
                return Vector3.zero;
            }

            if (closedLoop)
            {
                index = (index % WayPoints.Count + WayPoints.Count) % WayPoints.Count;
            }
            else
            {
                if (index < 0) index = 0;
                if (index >= WayPoints.Count) index = WayPoints.Count - 1;
            }

            return WayPoints[index].Position;
        }

        private Vector3 GetCatmullRomPosition(float t, Vector3 p0, Vector3 p1, Vector3 p2, Vector3 p3)
        {
            float t2 = t * t;
            float t3 = t2 * t;

            float b0 = 0.5f * (-t3 + 2f * t2 - t);
            float b1 = 0.5f * (3f * t3 - 5f * t2 + 2f);
            float b2 = 0.5f * (-3f * t3 + 4f * t2 + t);
            float b3 = 0.5f * (t3 - t2);

            return p0 * b0 + p1 * b1 + p2 * b2 + p3 * b3;
        }

        private void DrawArrowGizmo(Vector3 from, Vector3 to, float arrowHeadLength = 0.25f, float arrowHeadAngle = 20f)
        {
            Gizmos.DrawLine(from, to);

            Vector3 direction = (to - from).normalized;
            Vector3 right = Quaternion.LookRotation(direction)
                            * Quaternion.Euler(0, 180 + arrowHeadAngle, 0)
                            * Vector3.forward;
            Vector3 left = Quaternion.LookRotation(direction)
                           * Quaternion.Euler(0, 180 - arrowHeadAngle, 0)
                           * Vector3.forward;

            Gizmos.DrawLine(to, to + right * arrowHeadLength);
            Gizmos.DrawLine(to, to + left * arrowHeadLength);
        }

        public void RemoveNearDuplicateWayPoints(float threshold = 0.1f)
        {
            if (WayPoints.Count < 2) return;

            for (int i = 0; i < WayPoints.Count - 1; i++)
            {
                var wpA = WayPoints[i];
                var wpB = WayPoints[i + 1];

                if (Vector3.Distance(wpA.Position, wpB.Position) < threshold)
                {
                    WayPoints.RemoveAt(i + 1);
                    i--;
                }
            }
        }
    }
}
