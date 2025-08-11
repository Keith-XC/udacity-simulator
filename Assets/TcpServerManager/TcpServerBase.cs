using System;
using System.Collections.Generic;
using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using UnityEngine;

namespace Assets.TcpServerManager
{
    public abstract class TcpServerBase
    {
        protected TcpListener server;
        protected List<TcpClient> clients = new List<TcpClient>();
        protected bool isServerRunning = false;
        protected Thread listenerThread;
        protected int port;

        public TcpServerBase(int port)
        {
            this.port = port;
        }

        public void StartServer()
        {
            if (isServerRunning)
            {
                Debug.Log($"{GetType().Name} is already running.");
                return;
            }

            try
            {
                server = new TcpListener(IPAddress.Any, port);
                server.Start();
                isServerRunning = true;
                Debug.Log($"{GetType().Name} started on port {port} and waiting for connections.");
                listenerThread = new Thread(HandleClients);
                listenerThread.IsBackground = true;
                listenerThread.Start();
            }
            catch (Exception e)
            {
                Debug.LogError($"Error starting {GetType().Name} on port {port}: {e.Message}");
                isServerRunning = false;
            }
        }

        protected virtual void HandleClients()
        {
            while (isServerRunning)
            {
                try
                {
                    TcpClient client = server.AcceptTcpClient();
                    lock (clients)
                    {
                        clients.Add(client);
                    }
                    Debug.Log($"New client connected to {GetType().Name}.");
                    Thread clientThread = new Thread(() => HandleClientComm(client));
                    clientThread.IsBackground = true;
                    clientThread.Start();
                }
                catch (Exception e)
                {
                    Debug.LogError($"Error accepting client on {GetType().Name}: {e.Message}");
                }
            }
        }

        protected abstract void HandleClientComm(TcpClient client);

        public void StopServer()
        {
            isServerRunning = false;
            server?.Stop();
            CloseAllClients();
        }

        protected void CloseAllClients()
        {
            lock (clients)
            {
                foreach (var client in clients)
                {
                    client.Close();
                }
                clients.Clear();
            }
        }
    }
}
