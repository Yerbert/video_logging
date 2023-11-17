using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using RosMessageTypes.Geometry;


public class GoalSubscriber : MonoBehaviour
{

    void Awake() {
        StartConnection();
    }

    void StartConnection()
    {
        ClearGoal();

        ROSConnection ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<Vector3Msg>("/goal", SetGoal);
    }

    void SetGoal(Vector3Msg msg) {
        transform.localPosition = msg.From<FLU>();
    }

    public void ClearGoal() {
        transform.localPosition = Vector3.zero;
    }
}