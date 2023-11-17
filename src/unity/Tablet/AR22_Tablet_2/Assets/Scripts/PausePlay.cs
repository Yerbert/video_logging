using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using Unity.Robotics.ROSTCPConnector;
using Bool = RosMessageTypes.Std.BoolMsg;

public class PausePlay : MonoBehaviour
{
    public Text _title;
    private string topic_name = "/pause";
    ROSConnection ros;

    private bool paused = false;

    void Awake() {
        StartConnection();
    }

    public void StartConnection()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<Bool>(topic_name);
    }

    public void PauseOrPlay() {
        if(paused)
        {
            Debug.Log("PLAY");
            _title.text = "Pause";
            ros.Publish(topic_name, new Bool(false));
        }
        else
        {
            Debug.Log("PAUSE");
             _title.text = "Play";
            ros.Publish(topic_name, new Bool(true));
        }
        paused = !paused;
    }
}
