using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Assets.Traffic_Lights_System.Scripts;

namespace Assets.TcpServerManager
{
    [System.Serializable]
    public class TrafficManager
    {
        public string ManagerName;
        public List<Phase> PhaseList;
        private float PhaseDelay;
    }
}
