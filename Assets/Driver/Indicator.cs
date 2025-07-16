using Assets.WayPointSystem2;
using UnityEngine;

namespace Assets.OtherDriver
{
    public class Indicator : MonoBehaviour
    {
        public IndicatorDirection Orientation;
        public bool Activated;

        [SerializeField] private Light indicatorLight;

        private void Start()
        {
            if (indicatorLight == null)
            {
                indicatorLight = GetComponent<Light>();
            }
            if (indicatorLight != null)
                indicatorLight.enabled = false;
        }

        public void ToggleLight()
        {
            if (indicatorLight != null)
            {
                indicatorLight.enabled = !indicatorLight.enabled;
            }
        }

        public void TurnOffLight()
        {
            if (indicatorLight != null)
            {
                indicatorLight.enabled = false;
            }
        }
    }
}