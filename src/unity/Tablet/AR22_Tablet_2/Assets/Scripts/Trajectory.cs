using System.Collections;
using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;
using System.Linq;
using TFMessage = RosMessageTypes.Tf2.TFMessageMsg;
using RosMessageTypes.Geometry;

public class Trajectory : MonoBehaviour
{
    private ROSConnection ros;

    public string tf_topic = "/tf";
    public string tf_path_topic = "/tf_path";

    public LineRenderer pathLineRenderer;

    void Awake() {
        StartConnection();
    }

    public void StartConnection()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<TFMessage>(tf_path_topic, ReceivePathMsg);
    }

    void CreatePathLine(TFMessage msg) {
        Debug.Log("CREATING PATH LINE ...");
        Vector3[] points = msg.transforms.Select(tf => fromVector3Msg(tf.transform.translation)).ToArray();
        pathLineRenderer.positionCount = points.Length;
        pathLineRenderer.SetPositions(points);
        pathLineRenderer.enabled = true;
        Debug.Log("PATH LINE CREATED");
    }

    public void ReceivePathMsg(TFMessage tf_msg) {
        Debug.Log("RECEIVED PATH MSG");
        CreatePathLine(tf_msg);
    }

    private Vector3 fromVector3Msg(Vector3Msg v) {
        // return new Vector3((float)(v.x), (float)(v.y), (float)(v.z));
        return new Vector3((float)(-v.x), (float)(-v.z) , (float)(-v.y));
    }
}
