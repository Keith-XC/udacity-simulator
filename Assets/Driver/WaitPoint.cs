using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Assets.OtherDriver
{
    [System.Serializable]
    public class WaitPoint
    {
        public float _routeDistance;
        public float _waitTime;
        public bool _repeatPerRount;

        public WaitPoint(float routeDistance, float waitTime, bool repeatPerRount)
        {
            _routeDistance = routeDistance;
            _waitTime = waitTime;
            _repeatPerRount = repeatPerRount;
        }
    }
}
