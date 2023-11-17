using System.Collections;
using System.Collections.Generic;
using UnityEngine;

using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std;

public class CalibrationSubscriber : MonoBehaviour
{

    public GameObject[] calibrationObjects;

    ROSConnection ros;
    string calTopic = "/calibration";

    void Awake() {
        StartConnection();
    }

    void StartConnection()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<BoolMsg>(calTopic, ToggleCalibration);
    }

    void ToggleCalibration(BoolMsg msg) {
        foreach(GameObject g in calibrationObjects) {
            g.SetActive(msg.data);
        }
    }
}
