using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using Float32 = RosMessageTypes.Std.Float32Msg;
using Unity.Robotics.ROSTCPConnector;

public class ProgressSubscriber : MonoBehaviour
{   
    private ROSConnection ros;
    public Slider slider;

    public string progress_topic = "/progress";

    void Awake() {
        Debug.Log("Progress bar awake");
        StartConnection();
    }

    public void StartConnection()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<Float32>(progress_topic, UpdateSlider);
    }

    public void UpdateSlider(Float32 msg)
    {  
        slider.value = msg.data;
    }

    public void ClearProgress()
    {  
        slider.value = 0;
    }
}
