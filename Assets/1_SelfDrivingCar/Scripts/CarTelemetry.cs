using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Assets._1_SelfDrivingCar.Scripts
{
    [Serializable]
    public class CarTelemetry
    {
        public int carId; 
        public string timestamp;
        public string image;
        public float pos_x;
        public float pos_y;
        public float pos_z;
        public float steering_angle;
        public float throttle;
        public float speed;
        public float cte;
        public float next_cte;
        public int sector;
        public int lap;
    }
}
