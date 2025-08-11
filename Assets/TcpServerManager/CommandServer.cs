using System;
using System.Collections.Generic;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using UnityEngine;

namespace Assets.TcpServerManager
{
    public class CommandServer : TcpServerBase
    {
        public event Action<CommandData> OnCommandReceived;

        public CommandServer(int port) : base(port) { }

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
                            ProcessCommandMessage(message);
                        }
                        data.Clear();
                    }
                }
                catch (Exception e)
                {
                    Debug.LogError($"Error receiving command data: {e.Message}");
                    lock (clients)
                    {
                        clients.Remove(client);
                    }
                    client.Close();
                    break;
                }
            }
        }

        private void ProcessCommandMessage(string message)
        {
            Debug.Log($"Received command message: {message}");
            try
            {
                CommandData commandData = JsonUtility.FromJson<CommandData>(message);
                if (UnityMainThreadDispatcher.Exists())
                {
                    UnityMainThreadDispatcher.Instance().Enqueue(() => { OnCommandReceived?.Invoke(commandData); });
                }
            }
            catch (Exception e)
            {
                Debug.LogError($"Error processing command message: {e.Message}");
            }
        }
    }
}
