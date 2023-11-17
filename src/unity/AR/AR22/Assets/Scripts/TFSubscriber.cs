using System.Collections;
using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;
using System.Linq;
using TFMessage = RosMessageTypes.Tf2.TFMessageMsg;
using RosMessageTypes.Geometry;
using RosMessageTypes.BuiltinInterfaces;

public class TFSubscriber : MonoBehaviour
{
    private ROSConnection ros;
    // public Robot robot;
    public Transform mapFrame;
    public Transform odomFrame;
    public Transform baseLinkFrame;
    public Transform flatOdomFrame;
    public Transform flatBaseLinkFrame;
    public LineRenderer pathLineRenderer;

    public string tfTopic = "/tf";
    public string tfStaticTopic = "/tf_static";
    public string tfPathTopic = "/tf_path";

    private Dictionary<string, Transform> transformLookup;
    private List<string> nonExistentTransforms;
    public Queue<TransformStampedMsg> flatBaseLinkQueue;
    public Queue<TransformStampedMsg> flatOdomQueue;
    private int queueSize = 100;


    void Awake() {
        StartConnection();
    }

    public void StartConnection()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<TFMessage>(tfTopic, ReceivedTFMessage);
        ros.Subscribe<TFMessage>(tfPathTopic, ReceivedPathMessage);
        ros.Subscribe<TFMessage>(tfStaticTopic, UpdateTransforms);

        transformLookup = new Dictionary<string, Transform>() {
            {"base_link", baseLinkFrame},
            {"odom", odomFrame},
            {"map", mapFrame}
            // {"flat_base_link", flatBaseLinkFrame},
            // {"flat_odom", flatOdomFrame}
        };

        nonExistentTransforms = new List<string>();
        nonExistentTransforms.Add("velodyne");

        flatBaseLinkQueue = new Queue<TransformStampedMsg>();
        flatOdomQueue = new Queue<TransformStampedMsg>();

        ClearTF();
    }

    void ReceivedTFMessage(TFMessage msg) {

        UpdateTransforms(msg);
        
    }

    public void ClearTF() {
        zeroTransform(odomFrame, true);
        zeroTransform(baseLinkFrame, true);
        pathLineRenderer.SetPositions(new Vector3[0]);
        pathLineRenderer.positionCount = 0;
        baseLinkFrame.Rotate(new Vector3(0,90,0));
    }

    void UpdateTransforms(TFMessage msg) {

        foreach(TransformStampedMsg tf in msg.transforms) {

            // Update Queues

            if (isFlatBaseLinkFrame(tf)) {
                flatBaseLinkQueue.Enqueue(tf);
                while (flatBaseLinkQueue.Count > queueSize) {
                    flatBaseLinkQueue.Dequeue();
                }
            }

            if (isFlatOdomFrame(tf)) {
                flatOdomQueue.Enqueue(tf);
                while (flatOdomQueue.Count > queueSize) {
                    flatOdomQueue.Dequeue();
                }
            }


            // Transform frames
            
            string frame_id = tf.child_frame_id;

            if (nonExistentTransforms.Contains(frame_id)) {
                continue;
            }

            if (!transformLookup.ContainsKey(frame_id)) {
                Transform scene_transform = FindInAllChildren(baseLinkFrame, frame_id);
                if (scene_transform == null) {
                    nonExistentTransforms.Add(frame_id);
                    Debug.Log("!! " + frame_id + " added to non-existent transforms");
                } else {
                    transformLookup[frame_id] = scene_transform;
                    Debug.Log(frame_id + " added to list of transforms");
                }
            }

            if (transformLookup.ContainsKey(frame_id)) {
                setLocalTransform(transformLookup[frame_id], tf);
            }
            
        }
    }

    public void SetFlatFramesToClosestTimeStamp(TimeMsg otherTimeMsg) {
        TransformStampedMsg closestBaseTF = GetClosestFrame(otherTimeMsg, flatBaseLinkQueue);
        TransformStampedMsg closestOdomTF = GetClosestFrame(otherTimeMsg, flatOdomQueue);

        setLocalTransform(flatBaseLinkFrame, closestBaseTF);
        setLocalTransform(flatOdomFrame, closestOdomTF);
    }

    TransformStampedMsg GetClosestFrame(TimeMsg otherTimeMsg, Queue<TransformStampedMsg> TFQueue) {
        float minError = float.MaxValue;
        TransformStampedMsg closestTF = new TransformStampedMsg();
        foreach(TransformStampedMsg tf in TFQueue) {
            double frameTime = TimeMsgToTime(tf.header.stamp);
            double otherTime = TimeMsgToTime(otherTimeMsg);
            float error = Mathf.Abs((float)(frameTime - otherTime));
            if (error < minError) {
                minError = error;
                closestTF = tf;
            }
        }

        return closestTF;
    }

    double TimeMsgToTime(TimeMsg msg) {
        double nanodiv = 1.0e-9;
        double time = (msg.sec + (msg.nanosec * nanodiv));
        // Debug.Log("sec: "+msg.sec+" , nsec: "+msg.nanosec+", time: "+time.ToString("#,00000.00000"));
        return time;
    }

    Transform FindInAllChildren(Transform parent, string name) {

        /*
        Source: https://gamedev.stackexchange.com/questions/142546/in-unity-how-to-get-reference-of-descendant
        */

        var searchResult = parent.Find(name);

        if (searchResult != null)
            return searchResult;

        foreach (Transform child in parent) {
            searchResult = FindInAllChildren(child, name);
            if (searchResult != null)
                return searchResult;
        }

        return null;
    }

    void ReceivedPathMessage(TFMessage msg) {
        Debug.Log("RECEIVED PATH MESSAGE");
        CreatePathLine(msg);
    }

    void CreatePathLine(TFMessage msg) {
        // Debug.Log("CREATING PATH LINE ...");
        // Vector3[] points = msg.transforms.Select(tf => fromVector3Msg(tf.transform.translation)).ToArray();
        // pathLineRenderer.positionCount = points.Length;
        // pathLineRenderer.SetPositions(points);
        // pathLineRenderer.enabled = true;
        // Debug.Log("PATH LINE CREATED");

        Debug.Log("CREATING PATH LINE ...");

        
        int numPoints = msg.transforms.Where(isBaseLinkFrame).Count();
        List<Vector3> points = new List<Vector3>(numPoints);
        bool hasReadOdomMsg = false;
        Vector3 startOdomPosition = odomFrame.localPosition;
        Quaternion startOdomRotation = odomFrame.localRotation;
        Vector3 startMapPosition = mapFrame.localPosition;
        Quaternion startMapRotation = mapFrame.localRotation;

        zeroTransform(mapFrame, false);
        // mapFrame.position = Vector3.zero;
        // mapFrame.rotation = Quaternion.identity;

        int downsample = 10;
        foreach(TransformStampedMsg tf in msg.transforms.Where((tf, ind) => ind % downsample == 0)) {

            if (isBaseLinkFrame(tf) && hasReadOdomMsg) {
                setLocalTransform(baseLinkFrame, tf);
                points.Add( baseLinkFrame.position - mapFrame.position );
            }
            if (isOdomFrame(tf)) {
                hasReadOdomMsg = true;
                setLocalTransform(odomFrame, tf);
            }
        }

        odomFrame.localPosition= startOdomPosition;
        odomFrame.localRotation = startOdomRotation;

        mapFrame.localPosition= startMapPosition;
        mapFrame.localRotation = startMapRotation;

        pathLineRenderer.positionCount = points.Count;
        pathLineRenderer.SetPositions(points.ToArray());
        pathLineRenderer.enabled = true;

        Debug.Log("PATH CREATED WITH "+points.Count+" POINTS");
    }

    private void zeroTransform(Transform trans, bool local) {

        if (local) {
            trans.localPosition = Vector3.zero;
            trans.localRotation = Quaternion.identity;
        } else {
            trans.position = Vector3.zero;
            trans.rotation = Quaternion.identity;
        }
    }
    

    private bool isBaseLinkFrame(TransformStampedMsg tf) {
        return (tf.child_frame_id == "base_link" && tf.header.frame_id == "odom");
    }

    private bool isOdomFrame(TransformStampedMsg tf) {
        return (tf.child_frame_id == "odom" && tf.header.frame_id == "map");
    }

    private bool isFlatBaseLinkFrame(TransformStampedMsg tf) {
        return (tf.child_frame_id == "flat_base_link" && tf.header.frame_id == "flat_odom");
    }

    private bool isFlatOdomFrame(TransformStampedMsg tf) {
        return (tf.child_frame_id == "flat_odom" && tf.header.frame_id == "map");
    }

    private void setLocalTransform(Transform trans, TransformStampedMsg tf) {
        trans.localPosition = fromVector3Msg(tf.transform.translation);
        trans.localRotation = fromQuaternionMsg(tf.transform.rotation);
    }

    private Vector3 fromVector3Msg(Vector3Msg v) {
        return v.From<FLU>();
    }

    private Quaternion fromQuaternionMsg(QuaternionMsg q) {
        return q.From<FLU>();
    }

    // private Vector3 fromVector3Msg(Vector3Msg v) {
    //     return v.From<FLU>();
    // }

    // private Quaternion fromQuaternionMsg(QuaternionMsg q) {
    //     return q.From<FLU>();
    // }

    // private Vector3 fromVector3Msg(Vector3Msg v) {
    //     // return new Vector3((float)(v.x), (float)(v.y), (float)(v.z));
    //     return new Vector3((float)(-v.x), (float)(-v.z) , (float)(-v.y));
    //     // return new Vector3((float)(-v.x), 0f , (float)(-v.y));
    // } 

    // private Quaternion fromQuaternionMsg(QuaternionMsg q) {
    //     // Angle Correction (Y and Z axis switched)
    //     Quaternion angleCorrection = Quaternion.Euler(0, 90, 0);
    //     Quaternion robotQuaternion = new Quaternion((float)(-q.x), (float)(-q.z), (float)(-q.y), (float)(q.w));
    //     return robotQuaternion * angleCorrection;
    // }
}
