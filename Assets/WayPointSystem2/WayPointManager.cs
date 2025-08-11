using System.Collections.Generic;
using System.Linq;
using Assets.OtherDriver;
using UnityEngine;

namespace Assets.WayPointSystem2
{
    [ExecuteAlways]
    public class WayPointManager : MonoBehaviour
    {
        public List<WayPointCircuit> Circuits = new List<WayPointCircuit>();
        public List<WayPoint> WayPoints = new List<WayPoint>();

        public List<WayPointCircuit> CombineInEditor = new List<WayPointCircuit>();
        public List<MoveAlongCircuitWithCarControl> cars = new List<MoveAlongCircuitWithCarControl>();
        public bool ShowBrakingSphere = true;

        /// <summary>
        /// Updates the list of circuits and waypoints.
        /// </summary>
        void Update()
        {
            Circuits = FindObjectsOfType<WayPointCircuit>().ToList();
            WayPoints = FindObjectsOfType<WayPoint>().ToList();
#if UNITY_EDITOR
            UnityEditor.EditorUtility.SetDirty(this);
#endif
        }

        /// <summary>
        /// Combines the specified list of circuit names into a single circuit.
        /// </summary>
        /// <param name="planString">List of circuit names to combine.</param>
        /// <returns>The combined WayPointCircuit.</returns>
        public WayPointCircuit combineWayPointCircuit(List<string> planString)
        {
            List<WayPointCircuit> plan = new List<WayPointCircuit>();
            foreach (var element in planString)
            {
                plan.Add(Circuits.First(x => x.name == element));
            }

            GameObject objectPrefab = Resources.Load<GameObject>("Objects/WayPointCircuit");
            GameObject objectToSpawn = Instantiate(objectPrefab, Vector3.zero, Quaternion.identity);

            WayPointCircuit combinedCircuit = objectToSpawn.GetComponent<WayPointCircuit>();
            combinedCircuit.baseSpeed = 2;
            combinedCircuit.name = "combined";
            combinedCircuit.closedLoop = false;

            for (int x = 0; x < plan.Count; x++)
            {
                bool connectionPointReached = false;

                foreach (var wayPoint in plan[x].WayPoints)
                {
                    if (!combinedCircuit.WayPoints.Any() || (combinedCircuit.WayPoints.Last().PossibleTurns != null && combinedCircuit.WayPoints.Last().PossibleTurns.Any(x => x.wayPoint == wayPoint)))
                    {
                        connectionPointReached = true;
                        int count = combinedCircuit.WayPoints.Count;
                        if (count >= 3)
                        {
                            WayPoint p1 = combinedCircuit.WayPoints[count - 2];
                            WayPoint p2 = combinedCircuit.WayPoints[count - 1];
                            WayPoint p3 = wayPoint;
                            wayPoint.IndicatorOrientation = EvaluateTurnDirection(p1, p2, p3);
                            combinedCircuit.WayPoints[count - 1].IndicatorOrientation = wayPoint.IndicatorOrientation;
                            combinedCircuit.WayPoints[count - 2].IndicatorOrientation = wayPoint.IndicatorOrientation;
                        }
                    }

                    if (!combinedCircuit.WayPoints.Any() || connectionPointReached)
                    {
                        combinedCircuit.WayPoints.Add(wayPoint);
                    }

                    if (x + 1 < plan.Count && wayPoint.PossibleTurns.Any(y => plan[x + 1].WayPoints.Contains(y.wayPoint)))
                    {
                        break;
                    }
                }
            }

            Circuits.Add(combinedCircuit);
            return combinedCircuit;
        }

        /// <summary>
        /// Generates a random path based on existing waypoints and circuits.
        /// </summary>
        /// <param name="maxSteps">Maximum number of steps for the path.</param>
        /// <param name="turnProbability">Probability of taking a turn.</param>
        /// <returns>The generated WayPointCircuit.</returns>
        public WayPointCircuit GenerateRandomPath(int maxSteps = 50, float turnProbability = 0.5f)
        {
            WayPoints = FindObjectsOfType<WayPoint>().ToList();

            WayPoint startPoint = WayPoints[Random.Range(0, WayPoints.Count)];

            GameObject objectPrefab = Resources.Load<GameObject>("Objects/WayPointCircuit");
            GameObject objectToSpawn = Instantiate(objectPrefab, Vector3.zero, Quaternion.identity);
            WayPointCircuit randomCircuit = objectToSpawn.GetComponent<WayPointCircuit>();
            randomCircuit.name = "randomPath";
            randomCircuit.closedLoop = true;
            randomCircuit.baseSpeed = 2;

            randomCircuit.WayPoints.Add(startPoint);
            WayPoint current = startPoint;
            int steps = 0;
            int stepsSinceLastTurn = 0;

            while (steps < maxSteps)
            {
                if (stepsSinceLastTurn >= 10 && current.PossibleTurns != null && current.PossibleTurns.Count > 0)
                {
                    if (Random.value < turnProbability)
                    {
                        int turnIndex = Random.Range(0, current.PossibleTurns.Count);
                        WayPoint chosenTurn = current.PossibleTurns[turnIndex].wayPoint;
                        randomCircuit.WayPoints.Add(chosenTurn);
                        current = chosenTurn;
                        stepsSinceLastTurn = 0;

                        if (randomCircuit.WayPoints.Count >= 3)
                        {
                            int count = randomCircuit.WayPoints.Count;
                            WayPoint p1 = randomCircuit.WayPoints[count - 3];
                            WayPoint p2 = randomCircuit.WayPoints[count - 2];
                            WayPoint p3 = randomCircuit.WayPoints[count - 1];
                            IndicatorDirection turnDir = EvaluateTurnDirection(p1, p2, p3);
                            p1.IndicatorOrientation = turnDir;
                            p2.IndicatorOrientation = turnDir;
                            p3.IndicatorOrientation = turnDir;
                        }
                        steps++;
                        continue;
                    }
                }

                WayPointCircuit parentCircuit = Circuits.FirstOrDefault(c => c.WayPoints.Contains(current));
                if (parentCircuit != null)
                {
                    int currentIndex = parentCircuit.WayPoints.IndexOf(current);
                    if (currentIndex < parentCircuit.WayPoints.Count - 1)
                    {
                        WayPoint nextPoint = parentCircuit.WayPoints[currentIndex + 1];
                        randomCircuit.WayPoints.Add(nextPoint);
                        current = nextPoint;
                        stepsSinceLastTurn++;
                    }
                    else
                    {
                        break;
                    }
                }
                else
                {
                    break;
                }

                if (randomCircuit.WayPoints.Count >= 3)
                {
                    int count = randomCircuit.WayPoints.Count;
                    WayPoint p1 = randomCircuit.WayPoints[count - 3];
                    WayPoint p2 = randomCircuit.WayPoints[count - 2];
                    WayPoint p3 = randomCircuit.WayPoints[count - 1];
                    IndicatorDirection turnDir = EvaluateTurnDirection(p1, p2, p3);
                    p1.IndicatorOrientation = turnDir;
                    p2.IndicatorOrientation = turnDir;
                    p3.IndicatorOrientation = turnDir;
                }
                steps++;
            }

            Circuits.Add(randomCircuit);
            return randomCircuit;
        }

        /// <summary>
        /// Generates a random path and visualizes it in the Editor.
        /// </summary>
        [ContextMenu("Generate Random Path")]
        public void GenerateRandomPathEditor()
        {
            WayPointCircuit randomPathCircuit = GenerateRandomPath();
            for (int i = 0; i < randomPathCircuit.WayPoints.Count - 1; i++)
            {
                DrawArrowGizmo(randomPathCircuit.WayPoints[i].Position, randomPathCircuit.WayPoints[i + 1].Position);
            }
        }

        /// <summary>
        /// Evaluates the turn direction based on three waypoints.
        /// </summary>
        /// <param name="p1">First waypoint.</param>
        /// <param name="p2">Second waypoint.</param>
        /// <param name="p3">Third waypoint.</param>
        /// <returns>IndicatorDirection.Left if left turn, otherwise IndicatorDirection.Right.</returns>
        private IndicatorDirection EvaluateTurnDirection(WayPoint p1, WayPoint p2, WayPoint p3)
        {
            Vector3 pos1 = p1.transform.position;
            Vector3 pos2 = p2.transform.position;
            Vector3 pos3 = p3.transform.position;
            Vector3 dir12 = (pos2 - pos1).normalized;
            Vector3 dir23 = (pos3 - pos2).normalized;
            float crossY = Vector3.Cross(dir12, dir23).y;
            return crossY > 0 ? IndicatorDirection.Left : IndicatorDirection.Right;
        }

        /// <summary>
        /// Called by Unity to draw Gizmos.
        /// </summary>
        private void OnDrawGizmos()
        {
            DrawGizmos();
        }

        /// <summary>
        /// Draws Gizmos for possible turn connections.
        /// </summary>
        private void DrawGizmos()
        {
            if (WayPoints.Count < 2) return;

            foreach (var point in WayPoints)
            {
                if (point.PossibleTurns != null && point.PossibleTurns.Count > 0)
                {
                    foreach (var turn in point.PossibleTurns)
                    {
                        if (turn != null)
                        {
                            if (turn.indicatorDirection == IndicatorDirection.Right)
                                Gizmos.color = Color.cyan;
                            else if (turn.indicatorDirection == IndicatorDirection.Left)
                                Gizmos.color = Color.magenta;
                            else
                                Gizmos.color = Color.white;
                            Gizmos.DrawLine(point.Position, turn.wayPoint.Position);
                        }
                    }
                }
            }
        }

        /// <summary>
        /// Combines the circuits listed in CombineInEditor into one circuit and visualizes it.
        /// </summary>
        [ContextMenu("CombineToTest")]
        public void CombineSelectedCircuits()
        {
            var strings = new List<string>();
            foreach (var circuit in CombineInEditor)
            {
                strings.Add(circuit.name);
            }
            var combination = combineWayPointCircuit(strings);
            for (int i = 0; i < combination.WayPoints.Count - 1; i++)
            {
                DrawArrowGizmo(combination.WayPoints[i].Position, combination.WayPoints[i + 1].Position);
            }
        }

        /// <summary>
        /// Draws an arrow from one point to another.
        /// </summary>
        /// <param name="from">Start position.</param>
        /// <param name="to">End position.</param>
        /// <param name="arrowHeadLength">Length of the arrowhead.</param>
        /// <param name="arrowHeadAngle">Angle of the arrowhead.</param>
        private void DrawArrowGizmo(Vector3 from, Vector3 to, float arrowHeadLength = 0.25f, float arrowHeadAngle = 20f)
        {
            Gizmos.color = Color.blue;
            Gizmos.DrawLine(from, to);
            Vector3 direction = (to - from).normalized;
            Vector3 right = Quaternion.LookRotation(direction) * Quaternion.Euler(0, 180 + arrowHeadAngle, 0) * Vector3.forward;
            Vector3 left = Quaternion.LookRotation(direction) * Quaternion.Euler(0, 180 - arrowHeadAngle, 0) * Vector3.forward;
            Gizmos.DrawLine(to, to + right * arrowHeadLength);
            Gizmos.DrawLine(to, to + left * arrowHeadLength);
        }
    }
}
