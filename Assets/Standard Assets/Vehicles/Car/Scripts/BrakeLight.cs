using System;
using UnityEngine;

namespace UnityStandardAssets.Vehicles.Car
{
    public class BrakeLight : MonoBehaviour
    {
        public CarController car; // reference to the car controller, must be dragged in inspector

        private Renderer m_Renderer;


        private void Start()
        {
            m_Renderer = GetComponent<Renderer>();
        }


        private void Update()
        {
            // enable the Renderer when the car is braking, disable it otherwise.
            m_Renderer.enabled = car.BrakeInput > 0f;
        }

        public void ActivateBraking()
        {
            if (m_Renderer != null)
                m_Renderer.enabled = true;
        }

        public void DeactivateBraking()
        {
            if(m_Renderer != null)
                m_Renderer.enabled = false;
        }
    }
}
