using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Assets.TcpServerManager
{
    [System.Serializable]
    public class TrackData
    {
        public float roadWidth;
        public float laneWidth;
        public List<Vector3Data> coords;
        // Add whatever extra fields you need
    }
}
