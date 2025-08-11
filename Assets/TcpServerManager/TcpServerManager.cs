using Assets._1_SelfDrivingCar.Scripts;
using UnityEngine;

namespace Assets.TcpServerManager
{
    public class TcpServerManager : MonoBehaviour
    {
        private static TcpServerManager _instance;
        private bool _initialized = false;

        private CommandServer commandServer;
        private TelemetryServer telemetryServer;
        private EventServer eventServer;
        private OtherObjectsServer otherCarsServer;

        /// <summary>
        /// Gets the singleton instance of TcpServerManager.
        /// </summary>
        public static TcpServerManager Instance
        {
            get
            {
                if (_instance == null)
                {
                    _instance = FindObjectOfType<TcpServerManager>();
                    if (_instance == null)
                    {
                        GameObject serverManager = new GameObject("TcpServerManager");
                        _instance = serverManager.AddComponent<TcpServerManager>();
                        DontDestroyOnLoad(serverManager);
                        Debug.Log("TcpServerManager created.");
                    }
                    _instance.Initialize();
                }
                return _instance;
            }
        }

        private void Awake()
        {
            if (_instance != null && _instance != this)
            {
                Destroy(this.gameObject);
            }
            else
            {
                _instance = this;
                DontDestroyOnLoad(this.gameObject);
                Debug.Log("TcpServerManager initialized.");
                Initialize();
            }
        }

        /// <summary>
        /// Initializes the command, telemetry, event, and other cars servers.
        /// </summary>
        private void Initialize()
        {
            if (_initialized) return;

            int commandPort = Application.isEditor ? 55001 : 55002;
            int telemetryPort = Application.isEditor ? 56001 : 56002;
            int eventPort = Application.isEditor ? 57001 : 57002;
            int otherCarsPort = Application.isEditor ? 58001 : 58002;

            commandServer = new CommandServer(commandPort);
            commandServer.OnCommandReceived += HandleCommandReceived;
            commandServer.StartServer();

            telemetryServer = new TelemetryServer(telemetryPort);
            telemetryServer.StartServer();

            eventServer = new EventServer(eventPort);
            eventServer.OnEventReceived += HandleEventReceived;
            eventServer.StartServer();

            otherCarsServer = new OtherObjectsServer(otherCarsPort);
            otherCarsServer.OnOtherCarsCommandReceived += HandleOtherCarsCommandReceived;
            otherCarsServer.StartServer();

            _initialized = true;
        }

        /// <summary>
        /// Handles incoming command data.
        /// </summary>
        /// <param name="commandData">The received command data.</param>
        private void HandleCommandReceived(CommandData commandData)
        {
            switch (commandData.command)
            {
                case "send_control":
                    Debug.Log("Processing send_control command.");
                    HandleControlCommand(commandData);
                    break;
                default:
                    Debug.Log($"Unknown command: {commandData.command}");
                    break;
            }
        }

        /// <summary>
        /// Handles incoming event data.
        /// </summary>
        /// <param name="commandData">The event command data.</param>
        private void HandleEventReceived(CommandData commandData)
        {
            Debug.Log($"Event command received: {commandData.command}");
        }

        /// <summary>
        /// Handles incoming commands for other cars.
        /// </summary>
        /// <param name="commandData">The command data for other cars.</param>
        private void HandleOtherCarsCommandReceived(CommandData commandData)
        {
            Debug.Log($"Other cars command received: {commandData.command}");
        }

        /// <summary>
        /// Handles control commands by setting car actions via CarManager.
        /// </summary>
        /// <param name="commandData">The control command data.</param>
        private void HandleControlCommand(CommandData commandData)
        {
            CarManager carManager = FindObjectOfType<CarManager>();
            if (carManager != null)
            {
                carManager.SetCarAction(commandData);
            }
            else
            {
                Debug.LogError("CarManager not found");
            }
        }

        /// <summary>
        /// Broadcasts telemetry data to connected clients.
        /// </summary>
        /// <param name="telemetryData">The telemetry data to broadcast.</param>
        public void BroadcastTelemetry(CarTelemetry telemetryData)
        {
            if (telemetryServer != null)
            {
                telemetryServer.BroadcastTelemetry(telemetryData);
            }
        }

        /// <summary>
        /// Sends a spawn response to other cars.
        /// </summary>
        /// <param name="response">The spawn response data.</param>
        public void SendSpawnResponse(SpawnResponse response)
        {
            if (otherCarsServer != null)
            {
                otherCarsServer.SendSpawnResponse(response);
            }
        }

        /// <summary>
        /// Returns the event server instance.
        /// </summary>
        public EventServer GetEventServer()
        {
            return eventServer;
        }

        /// <summary>
        /// Returns the server handling other cars.
        /// </summary>
        public OtherObjectsServer GetOtherCarsServer()
        {
            return otherCarsServer;
        }

        /// <summary>
        /// Returns the command server instance.
        /// </summary>
        public CommandServer GetCommandServer()
        {
            return commandServer;
        }

        /// <summary>
        /// Returns the telemetry server instance.
        /// </summary>
        public TelemetryServer GetTelemetryServer()
        {
            return telemetryServer;
        }

        private void OnApplicationQuit()
        {
            commandServer?.StopServer();
            telemetryServer?.StopServer();
            eventServer?.StopServer();
            otherCarsServer?.StopServer();
        }
    }
}
