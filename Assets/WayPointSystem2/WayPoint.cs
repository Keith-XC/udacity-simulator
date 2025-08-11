using Assets.Traffic_Lights_System.Scripts;
using System.Collections.Generic;
using Assets.OtherDriver;
using UnityEngine;

namespace Assets.WayPointSystem2
{
    public enum TravelDirection { Forward, Backward, Both }

    [ExecuteAlways]
    public class WayPoint : MonoBehaviour
    {
        public RealTrafficLight TrafficLight;
        public List<PossibleTurn> PossibleTurns;
        public float Speed;
        public bool Stop = false;
        public int Number = 000;
        public IndicatorDirection IndicatorOrientation;
        public bool IsIntersection = false;

        [SerializeField] public Vector3 Position;

        void Update()
        {
            Position = transform.position;

#if UNITY_EDITOR
            UnityEditor.EditorUtility.SetDirty(this);
#endif
        }

        void OnDrawGizmos()
        {
            Gizmos.color = Color.red;
            Gizmos.DrawSphere(Position, 0.1f);
        }
    }

}


