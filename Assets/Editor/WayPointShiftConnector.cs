using Assets.WayPointSystem2;
using UnityEditor;
using UnityEngine;


    [InitializeOnLoad]
    public static class WayPointShiftConnector
    {
        static WayPointShiftConnector()
        {
            SceneView.duringSceneGui += OnSceneGUI;
        }

        private static void OnSceneGUI(SceneView sceneView)
        {
            Event e = Event.current;
            if (e == null)
                return;

            // Reagiere auf einen Linksklick (MouseDown, button 0) während die Shift-Taste gedrückt wird
            if (e.type == EventType.MouseDown && e.button == 0 && e.shift)
            {
                // Hole den aktuell im Editor ausgewählten WayPoint (Ausgangspunkt)
                GameObject sourceObj = Selection.activeGameObject;
                if (sourceObj == null)
                    return;

                WayPoint sourceWP = sourceObj.GetComponent<WayPoint>();
                if (sourceWP == null)
                    return;

                // Ermittle den WayPoint, der an der aktuellen Mausposition angeklickt wurde
                GameObject clickedObj = HandleUtility.PickGameObject(e.mousePosition, false);
                if (clickedObj != null)
                {
                    WayPoint targetWP = clickedObj.GetComponent<WayPoint>();
                    // Nur wenn es ein anderer WayPoint ist
                    if (targetWP != null && targetWP != sourceWP)
                    {
                        AddPossibleTurn(sourceWP, targetWP);
                        EditorUtility.SetDirty(sourceWP);
                        Debug.Log($"Verbindung von {sourceWP.name} zu {targetWP.name} hinzugefügt.");
                    }
                }
                // Das Event wird verbraucht, sodass keine weitere Standardaktion erfolgt
                e.Use();
            }
        }

        private static void AddPossibleTurn(WayPoint from, WayPoint to)
        {
            // Erstelle eine neue Instanz von PossibleTurn.
            // Passe dies an deine konkrete Implementierung von PossibleTurn an.
            PossibleTurn newTurn = new PossibleTurn(to, IndicatorDirection.Straight);
            newTurn.wayPoint = to;

            // Verhindere doppelte Einträge, falls bereits vorhanden
            if (!from.PossibleTurns.Exists(pt => pt.wayPoint == to))
            {
                from.PossibleTurns.Add(newTurn);
            }
        }
    }

