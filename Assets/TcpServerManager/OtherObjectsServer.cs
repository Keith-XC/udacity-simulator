using Assets._1_SelfDrivingCar.Scripts;
using System;
using System.Collections.Generic;
using System.Net.Sockets;
using System.Text;
using UnityEngine;

namespace Assets.TcpServerManager
{
    public class OtherObjectsServer : TcpServerBase
    {
        /// <summary>
        /// Event invoked when a command for other cars is received.
        /// </summary>
        public event Action<CommandData> OnOtherCarsCommandReceived;

        public OtherObjectsServer(int port) : base(port) { }

        /// <summary>
        /// Handles communication with a connected client.
        /// </summary>
        /// <param name="client">The connected TcpClient.</param>
        protected override void HandleClientComm(TcpClient client)
        {
            NetworkStream stream = client.GetStream();
            byte[] buffer = new byte[4096];
            StringBuilder data = new StringBuilder();

            while (true)
            {
                try
                {
                    int bytesRead = stream.Read(buffer, 0, buffer.Length);
                    if (bytesRead == 0)
                    {
                        lock (clients)
                        {
                            clients.Remove(client);
                        }
                        client.Close();
                        break;
                    }

                    data.Append(Encoding.UTF8.GetString(buffer, 0, bytesRead));

                    while (data.ToString().Contains("\n"))
                    {
                        string[] messages = data.ToString().Split(new[] { '\n' }, StringSplitOptions.RemoveEmptyEntries);
                        foreach (string message in messages)
                        {
                            ProcessOtherCarsMessage(message);
                        }
                        data.Clear();
                    }
                }
                catch (Exception)
                {
                    lock (clients)
                    {
                        clients.Remove(client);
                    }
                    client.Close();
                    break;
                }
            }
        }

        /// <summary>
        /// Processes a received message for other cars.
        /// </summary>
        /// <param name="message">The received message string.</param>
        private void ProcessOtherCarsMessage(string message)
        {
            Debug.Log($"Received Other Cars message: {message}");
            try
            {
                CommandData commandData = JsonUtility.FromJson<CommandData>(message);
                if (UnityMainThreadDispatcher.Exists())
                {
                    UnityMainThreadDispatcher.Instance().Enqueue(() =>
                    {
                        OnOtherCarsCommandReceived?.Invoke(commandData);
                    });
                }
            }
            catch (Exception e)
            {
                Debug.LogError($"Error processing Other Cars message: {e.Message}");
            }
        }

        /// <summary>
        /// Sends a spawn response message to all connected clients.
        /// </summary>
        /// <param name="spawnResponse">The spawn response data.</param>
        public void SendSpawnResponse(SpawnResponse spawnResponse)
        {
            if (clients.Count == 0)
            {
                return;
            }
            string message = JsonUtility.ToJson(spawnResponse) + "\n";
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
                    catch (Exception)
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
