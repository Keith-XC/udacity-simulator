using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using UnityEngine;

namespace Assets.WayPointSystem2
{


    [System.Serializable]
    public class Intersection
    {
        public Vector3 position;
        public List<WayPoint> wayPoints = new List<WayPoint>();
    }

    public class LaneInfo
    {
        public WayPoint wp;
        public Vector3 direction;
    }

}
