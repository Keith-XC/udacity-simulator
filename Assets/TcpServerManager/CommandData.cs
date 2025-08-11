using System;
using System.Collections.Generic;
using Assets.OtherDriver;
using Assets.TcpServerManager;

[Serializable]
public class CommandData
{
    public string command;
    public string name;
    public float speed;
    public float spawn_point;
    public string prefab_name;
    public float[] scale_Vektor;
    public float[] rotation;
    public float[] offset;
    public float humanBehavior;
    public WaitPoint[] waitingPoints;
    public TrafficLightCommand trafficLightCommand;
    public TrafficManager trafficLightManager;
    public List<TrackData> trackDataList;
    public int randomCarAmount;
    public string[] waypoints;
    public string layer;
    public string track_name;
    public string weather_name;
    public string daytime_name;
    public bool autonomous;
    public CarControll carControll;
    public int requestedCarId;

}



