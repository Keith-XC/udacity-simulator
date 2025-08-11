using Assets.TcpServerManager;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

namespace Assets.WayPointSystem2
{
    public class WayPointTracker : MonoBehaviour
    {
        private EventServer _eventServer;
        private TcpServerManager.TcpServerManager _tcpServerManager;
        public WayPointManager wayPointManager;
        public WayPointCircuit wayPointCircuit;
        [SerializeField] private Vector3 Position;

        public WayPoint getClosestWayPoint()
        {
            var allWayPoints = new List<WayPoint>();
            WayPoint closestWayPoint = null;
            float closestDistance = 0;
            if (wayPointManager == null)
            {
                wayPointManager = FindObjectsOfType<WayPointManager>().First();
            }
            allWayPoints = wayPointManager.WayPoints;
            Position = transform.position;

            foreach (var wayPoint in allWayPoints)
            {
                if (closestWayPoint == null)
                {
                    closestWayPoint = wayPoint;
                    closestDistance = Vector3.Distance(wayPoint.Position, Position);
                }
                else if (closestDistance > Vector3.Distance(wayPoint.Position, Position))
                {
                    closestWayPoint = wayPoint;
                    closestDistance = Vector3.Distance(wayPoint.Position, Position);
                }
            }

            return closestWayPoint;
        }

        public void Start()
        {
            wayPointManager = FindObjectsOfType<WayPointManager>().First();
            _tcpServerManager = TcpServerManager.TcpServerManager.Instance;

            _eventServer = _tcpServerManager.GetEventServer();
        }

        public void Update()
        {
            if (wayPointManager != null && wayPointManager.cars != null)
            {
                var cars = wayPointManager.cars;
                foreach (var car in cars)
                {
                    if(car == null) break;
                    var distance = Vector3.Distance(car.Position, transform.position);
                    if (distance < 7f)
                    {
                        //_eventServer.SendEventResponse("Hit Car: " + car.CarName);
                    }
                }
            }
            
        }
        
    }
}
