using UnityEngine;
using System;
using System.IO;
using UnityEngine.SceneManagement;
using Assets.TcpServerManager;

public class AppConfigurationManager : MonoBehaviour
{
    public AppConfiguration conf = new AppConfiguration();
    private GameObject _app;
    private TcpServerManager _tcpClient;

    /// <summary>
    /// Initializes application configuration from command line arguments and file, sets target frame rate, establishes TCP connection, and loads the next scene.
    /// </summary>
    public void Awake()
    {
        _app = GameObject.Find("Car");
        conf.fps = 60;
        conf.port = 4567;

        string[] args = Environment.GetCommandLineArgs();
        for (int i = 1; i < args.Length - 1; i++)
        {
            if (args[i] == "--config")
            {
                var configFilename = args[i + 1];
                conf = JsonUtility.FromJson<AppConfiguration>(File.ReadAllText(configFilename));
            }
        }

        for (int i = 1; i < args.Length - 1; i++)
        {
            if (args[i] == "--fps")
            {
                conf.fps = int.Parse(args[i + 1]);
            }
            if (args[i] == "--port")
            {
                conf.port = int.Parse(args[i + 1]);
            }
        }

        Debug.Log("Application started with the following configuration: " + JsonUtility.ToJson(conf));
        Application.targetFrameRate = 120;
        _tcpClient = TcpServerManager.Instance;
        SceneManager.LoadScene(1);
    }
}

[System.Serializable]
public class AppConfiguration
{
    [SerializeField]
    public int fps = 60;

    [SerializeField]
    public int port = 4567;

    [SerializeField]
    public int telemetryMinInterval = 33;
}