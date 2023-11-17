using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std;
using RosMessageTypes.Assets;

public class SubscriberManager : MonoBehaviour
{
    public TFSubscriber tfSubscriber;
    public PointCloudSubscriber pointCloudSubscriber;
    public CameraSubscriber cameraSubscriber;
    public InfoLogsMRTK infologsSubscriber;
    public ProgressSubscriber progressSubscriber;
    public MapSubscriber mapSubscriber;
    public GoalSubscriber goalSubscriber;

    void Awake() {
        StartConnection();
    }

    void StartConnection()
    {
        ROSConnection.GetOrCreateInstance().Subscribe<ClearScenarioMsg>("/clear_scenario", ClearScenario);
    }

    void ClearScenario(ClearScenarioMsg msg) {

        if (msg.tf) {
            tfSubscriber.ClearTF();
            progressSubscriber.ClearProgress();
            // mapSubscriber.ClearMap();
            goalSubscriber.ClearGoal();
            mapSubscriber.ClearDelocalisation();
        }

        if (msg.point_cloud) {
            pointCloudSubscriber.ClearPointCloud();
        }

        if (msg.camera) {
            cameraSubscriber.ClearVideo();
        }

        if (msg.infologs) {
            infologsSubscriber.ClearInfologs();
        }
        
    }
}
