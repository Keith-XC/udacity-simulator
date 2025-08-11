using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Assets.OtherDriver;

namespace Assets.WayPointSystem2
{
    [System.Serializable]
    public class PossibleTurn
    {
        public WayPoint wayPoint;
        public IndicatorDirection indicatorDirection;

        public PossibleTurn(WayPoint wayPoint, IndicatorDirection indicatorDirection)
        {
            this.wayPoint = wayPoint;
            this.indicatorDirection = indicatorDirection;
        }
    }

    [System.Serializable]
    public enum IndicatorDirection
    {
        Left,
        Straight,
        Right
    }
}
