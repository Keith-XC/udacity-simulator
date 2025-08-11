using System;
using System.Collections.Generic;
using System.Net.Sockets;
using System.Text;
using Assets._1_SelfDrivingCar.Scripts;
using UnityEngine;

namespace Assets.TcpServerManager
{
    public class TelemetryServer : TcpServerBase
    {
        public TelemetryServer(int port) : base(port) { }

        protected override void HandleClientComm(TcpClient client)
        {
            // Telemetry clients only receive data; no handling of incoming data is necessary.
        }

        public void BroadcastTelemetry(CarTelemetry telemetryData)
        {
            if (clients.Count == 0)
            {
                return;
            }

            string message = JsonUtility.ToJson(telemetryData) + "\n";
            byte[] data = Encoding.UTF8.GetBytes(message);
            List<TcpClient> disconnectedClients = new List<TcpClient>();

            lock (clients)
            {
                foreach (var client in clients)
                {
                    try
                    {
                        NetworkStream stream = client.GetStream();
                        stream.Write(data, 0, data.Length);
                    }
                    catch (Exception e)
                    {
                        disconnectedClients.Add(client);
                    }
                }
                foreach (var client in disconnectedClients)
                {
                    clients.Remove(client);
                }
            }
        }
    }
}