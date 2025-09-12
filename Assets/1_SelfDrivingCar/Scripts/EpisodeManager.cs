using Assets.TcpServerManager;
using Newtonsoft.Json;
using System;
using System.Collections.Generic;
using Assets.Traffic_Lights_System.Scripts;
using NUnit.Framework;
using Unity.IO.LowLevel.Unsafe;
using UnityEngine;
using UnityEngine.SceneManagement;
using System.Linq;

public class EpisodeManager : MonoBehaviour
{
    public List<EpisodeEvent> eventRecords;
    public EpisodeMetrics metrics;
    private EventServer _eventServer;
    private TcpServerManager _tcpServerManager;

    public void Update()
    {
        if (Input.GetKey(KeyCode.Z))
        {
            ResetTrack(Track.RoadGenerator);
        }
    }

    public void Awake()
    {
        eventRecords = new List<EpisodeEvent>();
        metrics = new EpisodeMetrics();
    }

    public void Start()
    {
        _tcpServerManager = TcpServerManager.Instance;

        _eventServer = _tcpServerManager.GetEventServer();
        if (_eventServer != null)
        {
            _eventServer.OnEventReceived += ProcessEventCommand;
        }
        else
        {
            Debug.LogError("EventServer is null. Cannot register event handlers.");
        }
    }

    private void OnDestroy()
    {
        if (_eventServer != null)
        {
            _eventServer.OnEventReceived -= ProcessEventCommand;
        }

    }

    private void ProcessEventCommand(CommandData commandData)
    {
        switch (commandData.command)
        {
            case "pause_sim":
                PauseSim();
                break;
            case "resume_sim":
                ResumeSim();
                break;
            case "start_episode":
                StartEpisode(commandData);
                break;
            case "end_episode":
                EndEpisode();
                break;
            case "control_traffic_Light":
                ControlTrafficLight(commandData);
                break;
            case "control_traffic_Manager":
                ControlTrafficManager(commandData);
                break;
            default:
                Debug.Log($"Unknown event command: {commandData.command}");
                break;
        }
    }

    public void ResetTrack(
        Track track,
        Weather weather = Weather.Sunny,
        DayTime dayTime = DayTime.Day)
    {
        WeatherController.SetWeather(weather);
        switch (track)
        {
            case Track.Lake:
                switch (dayTime)
                {
                    case DayTime.Day:
                        SceneManager.LoadScene("LakeTrackAutonomousDay");
                        break;
                    case DayTime.DayNightCycle:
                        SceneManager.LoadScene("LakeTrackAutonomousDayNightCycle");
                        break;
                }
                break;
            case Track.Jungle:
                switch (dayTime)
                {
                    case DayTime.Day:
                        SceneManager.LoadScene("JungleTrackAutonomousDay");
                        break;
                    case DayTime.DayNightCycle:
                        SceneManager.LoadScene("JungleTrackAutonomousDayNightCycle");
                        break;
                }
                break;
            case Track.Mountain:
                switch (dayTime)
                {
                    case DayTime.Day:
                        SceneManager.LoadScene("MountainTrackAutonomousDay");
                        break;
                    case DayTime.DayNightCycle:
                        SceneManager.LoadScene("MountainTrackAutonomousDayNightCycle");
                        break;
                }
                break;
            case Track.City:
                SceneManager.LoadScene("CityTrack");
                break;
            case Track.RoadGenerator:
                SceneManager.LoadScene("GeneratedTrack");
                break;
        }
    }

    public void AddEvent(EpisodeEvent e)
    {
        switch (e.key)
        {
            case "collision":
                metrics.collisionCount += 1;
                break;
            case "out_of_track":
                metrics.outOfTrackCount += 1;
                break;
        }
        eventRecords.Add(e);
        string eventJson = JsonConvert.SerializeObject(e);
        _eventServer.SendEventResponse(eventJson);
    }

    public void AddEvent(string key, string value)
    {
        this.AddEvent(new EpisodeEvent(key, value));
    }

    private void PauseSim()
    {
        Time.timeScale = 0;
        _eventServer.SendEventResponse("{\"event\":\"sim_paused\"}");
    }

    private void ResumeSim()
    {
        Time.timeScale = 1;
        _eventServer.SendEventResponse("{\"event\":\"sim_resumed\"}");
    }

    private void EndEpisode()
    {
        Time.timeScale = 0;
        string metricsJson = JsonConvert.SerializeObject(metrics);
        string eventsJson = JsonConvert.SerializeObject(eventRecords);

        _eventServer.SendEventResponse(metricsJson);
        _eventServer.SendEventResponse(eventsJson);
        _eventServer.SendEventResponse("{\"event\":\"episode_ended\"}");
    }

    private void StartEpisode(CommandData commandData)
    {
        string trackName = commandData.track_name;
        string weatherName = commandData.weather_name;
        string daytimeName = commandData.daytime_name;

        this.ResetTrack(
            this.TrackFromString(trackName),
            this.WeatherFromString(weatherName),
            this.DayTimeFromString(daytimeName)
        );

        eventRecords = new List<EpisodeEvent>();
        metrics = new EpisodeMetrics();
        Time.timeScale = 1;
        _eventServer.SendEventResponse("{\"event\":\"episode_started\"}");
    }

    private void ControlTrafficLight(CommandData commandData)
    {
        var trafficLightCommand = commandData.trafficLightCommand;
        GameObject light = GameObject.Find(trafficLightCommand.Name);
        var controller = light.GetComponent<RealTrafficLight>();
        Enum.TryParse(trafficLightCommand.State, out LightState state);
        if (controller != null)
        {
            controller.SetLightState(state);
        }
    }

    public void ControlTrafficManager(CommandData commandData)
    {
        var trafficLightManager = commandData.trafficLightManager;
        GameObject manager = GameObject.Find(trafficLightManager.ManagerName);
        var controller = manager.GetComponent<TrafficLightManager>();
        if (controller != null)
        {
            List<TrafficLightPhase> phases = controller.GetPhaseList();
            foreach (var phase in trafficLightManager.PhaseList)
            {
                List<TrafficLightBase> baseList = new List<TrafficLightBase>();
                foreach (var lightName in phase.TrafficLights)
                {
                    TrafficLightBase[] allTrafficLights = FindObjectsOfType<TrafficLightBase>();

                    TrafficLightBase light = allTrafficLights.FirstOrDefault(t => t.name == lightName);

                    baseList.Add(light);
                }

                var trafficLightPhase = new TrafficLightPhase
                {
                    Name = phase.Name,
                    TrafficLights = baseList.ToArray(),
                    PhaseActiveTime = phase.PhaseActiveTime,
                    PhaseEndTime = phase.PhaseEndTime,
                    PhaseStartTime = phase.PhaseStartTime
                };

                if (phases.Any(x => x.Name == phase.Name))
                {
                    var phaseToUpdate = phases.First(x => x.Name == phase.Name);
                    phases.Remove(phaseToUpdate);
                }
                phases.Add(trafficLightPhase);
            }
            controller.SetPhaseList(phases);
            controller.ChangeProgram(TrafficLightManager.Program.Main);


        }
    }

    private Track TrackFromString(string name)
    {
        switch (name.ToLower())
        {
            case "city":
                return Track.City;
            case "lake":
                return Track.Lake;
            case "jungle":
                return Track.Jungle;
            case "mountain":
                return Track.Mountain;
            case "generator":
                return Track.RoadGenerator;
            default:
                Debug.Log("Track not recognized, returning Track Lake.");
                return Track.Lake;
        }
    }

    private Weather WeatherFromString(string name)
    {
        switch (name.ToLower())
        {
            case "sunny":
                return Weather.Sunny;
            case "rainy":
                return Weather.Rainy;
            case "foggy":
                return Weather.Foggy;
            case "snowy":
                return Weather.Snowy;
            default:
                Debug.Log("Weather not recognized, returning Weather Sunny.");
                return Weather.Sunny;
        }
    }

    private DayTime DayTimeFromString(string name)
    {
        switch (name.ToLower())
        {
            case "day":
                return DayTime.Day;
            case "daynight":
                return DayTime.DayNightCycle;
            default:
                Debug.Log("DayTime not recognized, returning DayTime Day.");
                return DayTime.Day;
        }
    }
}

[Serializable]
public class EpisodeMetrics
{
    public int collisionCount = 0;
    public int outOfTrackCount = 0;
}

[Serializable]
public class EpisodeEvent
{
    public string timestamp;
    public string key;
    public string value;

    public EpisodeEvent(string key, string value)
    {
        this.timestamp = DateTimeOffset.Now.ToUnixTimeMilliseconds().ToString();
        this.key = key;
        this.value = value;
    }
}

public enum Track
{
    City,
    Lake,
    Jungle,
    Mountain,
    RoadGenerator,
}

public enum Weather
{
    Sunny,
    Rainy,
    Snowy,
    Foggy,
}

public enum DayTime
{
    Day,
    DayNightCycle,
}
