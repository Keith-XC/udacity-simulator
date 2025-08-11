using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Assets.WayPointSystem2;

namespace Assets.TcpServerManager
{
    [Serializable]
    public class CarControll
    {
        public int carId;
        public IndicatorDirection Indicator;
        public float steering_angle;
        public float throttle;
    }
}
