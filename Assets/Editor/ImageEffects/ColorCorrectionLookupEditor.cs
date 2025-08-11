using UnityEngine;
using UnityStandardAssets.Utility;

public class MoveAlongCircuit : MonoBehaviour
{
    public WaypointCircuit circuit;  // Referenz auf die WaypointCircuit-Komponente
    public float speed = 5.0f;       // Geschwindigkeit
    private float distanceTravelled; // Verfolgung der zur¸ckgelegten Strecke

    void Start()
    {
        if (circuit != null && circuit.Waypoints.Length > 0)
        {
            distanceTravelled = 0;
            Debug.Log($"Startpunkt standardm‰ﬂig auf Waypoint 0 gesetzt.");
        }
        else
        {
            Debug.LogWarning("Circuit oder Waypoints sind nicht richtig gesetzt.");
        }
    }

    public void SetStartPoint(int waypointIndex)
    {
        if (circuit == null || circuit.Waypoints.Length <= waypointIndex || waypointIndex < 0)
        {
            Debug.LogWarning($"Ung¸ltiger Wegpunktindex: {waypointIndex}. Startpunkt bleibt unver‰ndert.");
            return;
        }

        distanceTravelled = GetDistanceToWaypoint(waypointIndex);
        Debug.Log($"Startpunkt auf Waypoint {waypointIndex} gesetzt. Anfangsdistanz: {distanceTravelled}");
    }

    float GetDistanceToWaypoint(int waypointIndex)
    {
        float distance = 0.0f;

        for (int i = 0; i < waypointIndex; i++)
        {
            if (i + 1 < circuit.Waypoints.Length)
            {
                distance += Vector3.Distance(circuit.Waypoints[i].position, circuit.Waypoints[i + 1].position);
            }
        }

        return distance;
    }

    void Update()
    {
        if (circuit == null)
        {
            Debug.LogWarning("Circuit oder Waypoints sind nicht richtig gesetzt.");
            return;
        }

        distanceTravelled += speed * Time.deltaTime;
        WaypointCircuit.RoutePoint routePoint = circuit.GetRoutePoint(distanceTravelled);
        transform.position = routePoint.position;
        transform.rotation = Quaternion.LookRotation(routePoint.direction);
    }
}
