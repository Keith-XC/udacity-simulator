using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Assets.WayPointSystem2;
using UnityEngine;
using UnityStandardAssets.Utility;

namespace Assets.OtherDriver
{
    public class SpawnObject : MonoBehaviour
    {
        public WayPointCircuit _circuit;
        public LayerMask _roadLayer;

        /// <summary>
        /// Initializes the spawn object and logs whether the circuit and waypoints are set.
        /// </summary>
        void Start()
        {
            if (_circuit != null && _circuit.WayPoints.Count > 0)
            {
                Debug.Log("Circuit and waypoints are set. Object can be spawned.");
            }
            else
            {
                Debug.LogWarning("Circuit or waypoints are not properly set.");
            }
        }

        /// <summary>
        /// Spawns the object at a specified waypoint with given offset, scale, and rotation.
        /// </summary>
        /// <param name="waypointIndex">The waypoint index (can be fractional) at which to spawn.</param>
        /// <param name="offset">The offset vector to apply (x,z for horizontal and y for vertical offset).</param>
        /// <param name="scale">The local scale to set on the spawned object.</param>
        /// <param name="rotation">The external rotation to apply.</param>
        public void SpawnAtWaypoint(float waypointIndex, Vector3 offset, Vector3 scale, Quaternion rotation)
        {
            if (_circuit == null || _circuit.WayPoints.Count == 0)
            {
                Debug.LogWarning("No waypoints available. Cannot spawn object.");
                return;
            }

            int waypointCount = _circuit.WayPoints.Count;
            float clampedIndex = Mathf.Clamp(waypointIndex, 0f, waypointCount - 1f);

            int indexA = Mathf.FloorToInt(clampedIndex);
            int indexB = (indexA + 1) % waypointCount;
            float t = clampedIndex - indexA;

            Vector3 positionA = _circuit.WayPoints[indexA].Position;
            Vector3 positionB = _circuit.WayPoints[indexB].Position;
            Vector3 waypointPosition = Vector3.Lerp(positionA, positionB, t);

            Vector3 direction = (positionB - positionA).normalized;
            Vector3 right = Vector3.Cross(Vector3.up, direction).normalized;

            Vector3 horizontalOffset = right * offset.x + direction * offset.z;
            waypointPosition += horizontalOffset;

            Vector3 rayOrigin = waypointPosition + Vector3.up * 10f;
            Ray ray = new Ray(rayOrigin, Vector3.down);
            RaycastHit hit;

            if (Physics.Raycast(ray, out hit, 20f, _roadLayer))
            {
                waypointPosition = hit.point;
            }
            else
            {
                Debug.LogWarning("No surface found below the waypoint. Using original position.");
            }

            waypointPosition += Vector3.up * offset.y;
            transform.position = waypointPosition;

            Quaternion pathRotation = Quaternion.LookRotation(direction);
            transform.rotation = pathRotation * rotation;
            transform.localScale = scale;

            Debug.Log($"Object spawned at waypoint index {waypointIndex} with offset {offset}, scale {scale} and rotation {rotation.eulerAngles}.");
        }
    }
}
