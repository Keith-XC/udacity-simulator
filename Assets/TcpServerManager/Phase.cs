using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Assets.TcpServerManager
{
    [System.Serializable]
    public class Phase
    {
        public string Name;
        public List<string> TrafficLights;
        public float PhaseStartTime;
        public float PhaseActiveTime;
        public float PhaseEndTime;
    }
}
