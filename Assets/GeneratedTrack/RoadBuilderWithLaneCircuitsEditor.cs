#if UNITY_EDITOR
using UnityEngine;
using UnityEditor;

[CustomEditor(typeof(RoadBuilder))]
public class RoadBuilderWithLaneCircuitsEditor : Editor
{
    public override void OnInspectorGUI()
    {
        DrawDefaultInspector();

        RoadBuilder builder = (RoadBuilder)target;
        if (GUILayout.Button("Regenerate Lane Circuits"))
        {
            if (builder.pathManager != null && builder.pathManager.carPaths != null)
            {
                builder.RegenerateLaneCircuits();
            }
            else
            {
                Debug.LogWarning("No CarPath exist!");
            }
        }
    }
}
#endif