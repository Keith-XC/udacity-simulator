using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Assets.TcpServerManager
{
    [System.Serializable]
    public class TelemetryData
    {
        public float steering_angle;
        public float throttle;
        public float speed;
    }

}
