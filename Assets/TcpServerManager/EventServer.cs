using System;
using System.Collections.Generic;
using System.Net.Sockets;
using System.Text;
using UnityEngine;

namespace Assets.TcpServerManager
{
    public class EventServer : TcpServerBase
    {
        public event Action<CommandData> OnEventReceived;

        public EventServer(int port) : base(port) { }

        protected override void HandleClientComm(TcpClient client)
        {
            NetworkStream stream = client.GetStream();
            byte[] buffer = new byte[4096];
            int bytesRead;
            StringBuilder data = new StringBuilder();

            while (true)
            {
                try
                {
                    bytesRead = stream.Read(buffer, 0, buffer.Length);
                    if (bytesRead == 0)
                    {
                        Debug.Log("Command client disconnected.");
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
                            ProcessEventMessage(message);
                        }

                        data.Clear();
                    }
                }
                catch (Exception e)
                {
                    Debug.LogError($"Error receiving event data: {e.Message}");
                    lock (clients)
                    {
                        clients.Remove(client);
                    }
                    client.Close();
                    break;
                }
            }
        }

        private void ProcessEventMessage(string message)
        {
            Debug.Log($"Received event message: {message}");
            try
            {
                CommandData commandData = JsonUtility.FromJson<CommandData>(message);
                if (UnityMainThreadDispatcher.Exists())
                {
                    UnityMainThreadDispatcher.Instance().Enqueue(() => { OnEventReceived?.Invoke(commandData); });
                }
            }
            catch (Exception e)
            {
                Debug.LogError($"Error processing event message: {e.Message}");
            }
        }

        public void SendEventResponse(string message)
        {
            byte[] data = Encoding.UTF8.GetBytes(message + "\n");
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
                        Debug.LogError($"Error sending event response to client: {e.Message}");
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
