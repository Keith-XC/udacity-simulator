using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Assets.TcpServerManager
{
    [Serializable]
    public class SpawnResponse
    {
        public string command;
        public int requestedCarId;
        public int assignedCarId;
    }
}
    