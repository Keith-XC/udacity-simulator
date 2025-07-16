using Assets.WayPointSystem2;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace Assets.OtherDriver
{
    public class IndicatorManager : MonoBehaviour
    {
        public List<Indicator> Indicators = new List<Indicator>();
        public IndicatorDirection Direction = IndicatorDirection.Straight;

        [SerializeField] private float blinkInterval = 0.5f;

        private Coroutine blinkRoutine;

        /// <summary>
        /// Starts the blinking routine.
        /// </summary>
        private void Start()
        {
            blinkRoutine = StartCoroutine(BlinkIndicators());
        }

        /// <summary>
        /// Coroutine that toggles the lights of activated indicators at fixed intervals.
        /// </summary>
        private IEnumerator BlinkIndicators()
        {
            while (true)
            {
                foreach (Indicator indicator in Indicators)
                {
                    if (indicator.Activated)
                    {
                        indicator.ToggleLight();
                    }
                    else
                    {
                        indicator.TurnOffLight();
                    }
                }
                yield return new WaitForSeconds(blinkInterval);
            }
        }

        /// <summary>
        /// Activates all left indicators and deactivates the others.
        /// </summary>
        public void ActivateLeft()
        {
            Direction = IndicatorDirection.Left;
            foreach (var indicator in Indicators)
            {
                indicator.Activated = (indicator.Orientation == IndicatorDirection.Left);
            }
        }

        /// <summary>
        /// Activates all right indicators and deactivates the others.
        /// </summary>
        public void ActivateRight()
        {
            Direction = IndicatorDirection.Right;
            foreach (var indicator in Indicators)
            {
                indicator.Activated = (indicator.Orientation == IndicatorDirection.Right);
            }
        }

        /// <summary>
        /// Deactivates all indicators.
        /// </summary>
        public void DeactivateAll()
        {
            Direction = IndicatorDirection.Straight;
            foreach (var indicator in Indicators)
            {
                indicator.Activated = false;
                indicator.TurnOffLight();
            }
        }
    }
}
