/*
Adapted from:
 - https://github.com/inmo-jang/unity_assets/tree/master/PointCloudStreaming
    (ReceiveMessage, PointCloudRendering)
 - https://assetstore.unity.com/packages/tools/utilities/point-cloud-free-viewer-19811
    (InstantiateMesh, CreateMesh)
*/

using System.Collections;
using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.BuiltinInterfaces;
using RosMessageTypes.Sensor;
using UnityEngine;
using UnityEngine.UI;
using System;
using System.IO;
using UnityEngine.Android;
using TMPro;

public class PointCloudSubscriber : MonoBehaviour
{
    ROSConnection ros;
    private byte[] byteArray;
    private bool isMessageReceived = false;
    // bool readyToProcessMessage = true;
    private int size;

    private Vector3[] pcl;

    [Range(0f, 0.1f)]
    public float pointSize = 0.1f;

    int width;
    int height;
    int row_step;
    int point_step;

    private TimeMsg time;

    private bool isDrawing = false;

    public TFSubscriber tfSubscriber;

    public Material matVertex;
    private GameObject pointCloud;
    private GameObject pointCloudOld;

    // ~65000 is max number of vertices per mesh.
    // Must reduce by factor of 4 for tetrahedron-clouds.
    // This script draws one mesh per frame, so can reduce number of vertices per mesh to smooth out computation for smoother framerate.
    // Although, generally, the more meshes, the slower the computation.
    private int limitPoints = 16000; //21666; // 16250; //65000;
    private int numPoints;
	private int numPointGroups;

    public Transform mapOrigin;
    public Transform velodyneFrame;
    public TMP_InputField topic_name;

    public float PointCloudFrameRate = 3.33f;

    private int pc_frame = 1;
    private float pc_period;
    private bool timeToLoadNewPCFrame = false;
    private string pc_path;

    public Texture2D colourMapTexture;
    private Color[] colourMap;

    // public string topic_name = "/velodyne_points/processed";

    // Downsampling
    private int skipPoints = 0;


    void Awake() {
        pc_period = 1f /PointCloudFrameRate;
        pointCloud = new GameObject("PointCloudMesh");

        ReadColourMap();

        StartConnection();
                
        // StartCoroutine(PlayPointCloudFromFile());
        // InvokeRepeating("PlayPointCloudFromFile", 0f, pc_period);
    }

    void ReadColourMap() {
        colourMap = new Color[colourMapTexture.width];
        for(int i=0; i<colourMapTexture.width; i++) {
            colourMap[i] = colourMapTexture.GetPixel(i,0);
        }
    }

    public void StartConnection()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<PointCloud2Msg>(topic_name.text, ReceiveMessage);
    }

    public void FixedUpdate()
    {
        if (isMessageReceived)
        {
            StartCoroutine(PointCloudRendering());
            isMessageReceived = false;
        }

        if (timeToLoadNewPCFrame) {
            timeToLoadNewPCFrame = false;
            ReadPCDFile();
        }
    }

    public void ClearPointCloud() {

        foreach (Transform child in pointCloud.transform) {
            child.gameObject.GetComponent<MeshFilter>().sharedMesh.Clear();
        }
        Destroy(pointCloudOld);

        pointCloud = new GameObject("PointCloudMesh");

        StopAllCoroutines();
        isDrawing = false;
    }

    protected void ReceiveMessage(PointCloud2Msg message)
    {
        // skip this message if still localising or still drawing point cloud from last message
        if (isDrawing) {
            Debug.Log("Skipped");
            return;
        }

        size = message.data.GetLength(0);

        byteArray = new byte[size];
        byteArray = message.data;


        width = (int)message.width;
        height = (int)message.height;
        row_step = (int)message.row_step;
        point_step = (int)message.point_step;

        size = size / point_step;
        isMessageReceived = true;

        time = message.header.stamp;

    }

    IEnumerator PointCloudRendering()
    {

        isDrawing = true;

        pcl = new Vector3[size];

        int x_posi;
        int y_posi;
        int z_posi;

        float x;
        float y;
        float z;
      
        for (int n = 0; n < size; n++)
        {
            if (n*(1+skipPoints) >= size) { break; }

            x_posi = n * (1+skipPoints) * point_step + 0;
            y_posi = n * (1+skipPoints) * point_step + 4;
            z_posi = n * (1+skipPoints) * point_step + 8;

            x = BitConverter.ToSingle(byteArray, x_posi);
            y = BitConverter.ToSingle(byteArray, y_posi);
            z = BitConverter.ToSingle(byteArray, z_posi);

            pcl[n] = new Vector3(x, z, y);
        }

        numPoints = size;

        // Instantiate Point Groups
		numPointGroups = Mathf.CeilToInt(numPoints*1.0f / limitPoints*1.0f);

        pointCloudOld = pointCloud;

		pointCloud = new GameObject("PointCloudMesh");
        pointCloud.SetActive(false);    // make new mesh invisisble while it is being created

		for (int i = 0; i < numPointGroups-1; i ++) {
			InstantiateMesh(i, limitPoints);
            yield return null;
		}
		InstantiateMesh(numPointGroups-1, numPoints - (numPointGroups-1) * limitPoints);

        // Destroy old mesh once new one is created
        foreach (Transform child in pointCloudOld.transform) {
            child.gameObject.GetComponent<MeshFilter>().sharedMesh.Clear(); // to prevent memory leak
        }
        Destroy(pointCloudOld);

        // make new mesh visisble after deleting old one
        pointCloud.transform.parent = velodyneFrame;
        pointCloud.transform.localPosition = new Vector3(0,0,0);
        pointCloud.transform.localRotation = Quaternion.identity;
        pointCloud.transform.localScale = new Vector3(1,1,1);
        tfSubscriber.SetFlatFramesToClosestTimeStamp(time);
        pointCloud.transform.parent = mapOrigin;
        // pointCloud.transform.localEulerAngles = Vector3.Scale(new Vector3(0f,1f,0f), pointCloud.transform.localEulerAngles); // reduce rocking. assumes flat surface
        pointCloud.SetActive(true);

        // Unblock new messages
        isDrawing = false;

    }

    

    void InstantiateMesh(int meshInd, int nPoints){
		// Create Mesh
		GameObject pointGroup = new GameObject();
		pointGroup.AddComponent<MeshFilter>();
		pointGroup.AddComponent<MeshRenderer>();
		pointGroup.GetComponent<Renderer>().material = matVertex;

		pointGroup.GetComponent<MeshFilter>().mesh = CreateMesh(meshInd, nPoints, limitPoints);
		pointGroup.transform.parent = pointCloud.transform;
        pointGroup.transform.localPosition = new Vector3(0,0,0);
        pointGroup.transform.localRotation = Quaternion.identity;
        pointGroup.transform.localScale = new Vector3(1,1,1);

	}


    Mesh CreateMesh(int id, int nPoints, int limitPoints){

        Mesh mesh = new Mesh();

        mesh.vertices = GetTriangleVerts(pcl, id, nPoints, limitPoints);
        mesh.colors = GetColors(pcl, id, nPoints, limitPoints);
        mesh.triangles = GetTriangles(nPoints);

        return mesh;
	}


    private Vector3[] GetTriangleVerts(Vector3[] points, int id, int nPoints, int limitPoints) {

        Vector3 point;
        Vector3[] trianglePoints = new Vector3[4*nPoints];
        for(int i=0; i<nPoints; i++) {
            point = points[id*limitPoints + i];

            float x = point.x;
            float y = point.y;
            float z = point.z;

            trianglePoints[4*i] = new Vector3(x,y,z);
            trianglePoints[(4*i)+1] = new Vector3(x+2*pointSize,y,z);
            trianglePoints[(4*i)+2] = new Vector3(x+pointSize,y+2*pointSize,z+pointSize);
            trianglePoints[(4*i)+3] = new Vector3(x+pointSize,y,z+2*pointSize);
        }

        return trianglePoints;
    }

    // private Color[] GetColors(Vector3[] points, int id, int nPoints, int limitPoints) {
    //     Color[] colors = new Color[4*nPoints];
    //     for(int i=0; i<4*nPoints; i++) {
    //         colors[i] = Color.cyan;
    //     }
    //     return colors;
    // }





    private Color[] GetColors(Vector3[] points, int id, int nPoints, int limitPoints) {
        // float y_min = -0.5f;
        // float y_max = 1f;
        // float y_range = y_max-y_min;
        float height_offset = 0.3f;

        Vector3 point;
        Color[] colors = new Color[4*nPoints];
        for(int i=0; i<nPoints; i++) {

            point = points[id*limitPoints + i];
            int ind = Math.Min(colourMap.Length-1, Mathf.FloorToInt(Mathf.Lerp(0, colourMap.Length, point.y+height_offset)));

            Color col = colourMap[ind];

            colors[4*i] = col;
            colors[4*i+1] = col;
            colors[4*i+2] = col;
            colors[4*i+3] = col;
        }

        return colors;
    }






    // private Color[] GetColors(Vector3[] points, int id, int nPoints, int limitPoints) {
    //     float y_min = -0.5f;
    //     float y_max = 1f;
    //     float y_range = y_max-y_min;
    //     float height_offset = 0.2f;

        

    //     Vector3 point;
    //     Color[] colors = new Color[4*nPoints];
    //     for(int i=0; i<nPoints; i++) {

    //         Color col;
    //         point = points[id*limitPoints + i];
    //         switch(Mathf.Floor(point.y + height_offset)) {
    //             case -1:
    //                 col = Color.Lerp(Color.blue, Color.green, point.y+1);
    //                 break;
    //             case 0:
    //                 col = Color.Lerp(Color.green, Color.red, point.y);
    //                 break;
    //             default:
    //                 col = Color.Lerp(Color.blue, Color.red, point.y);
    //                 break;
    //         }

    //         // Debug.Log("Point y: "+point.y);

    //         colors[4*i] = col;
    //         colors[4*i+1] = col;
    //         colors[4*i+2] = col;
    //         colors[4*i+3] = col;
    //     }

    //     return colors;
    // }


    private int[] GetTriangles(int nPoints){
        int[] triangles = new int[12*nPoints];  // 4 tris, each tri has 3 points = 12
        for(int i=0; i<nPoints; i++) {
            triangles[(12*i)] = (4*i);
            triangles[(12*i)+1] = (4*i)+1;
            triangles[(12*i)+2] = (4*i)+2;

            triangles[(12*i)+3] = (4*i);
            triangles[(12*i)+4] = (4*i)+3;
            triangles[(12*i)+5] = (4*i)+1;

            triangles[(12*i)+6] = (4*i);
            triangles[(12*i)+7] = (4*i)+2;
            triangles[(12*i)+8] = (4*i)+3;

            triangles[(12*i)+9] = (4*i)+2;
            triangles[(12*i)+10] = (4*i)+1;
            triangles[(12*i)+11] = (4*i)+3;
        }
        return triangles;
    }

    /* // FOR 2D TRIS
    private Vector3[] GetTriangleVerts(Vector3[] points, int id, int nPoints) {

        Vector3 point;
        Vector3[] trianglePoints = new Vector3[3*points.Length];
        for(int i=0; i<nPoints; i++) {
            point = points[id*nPoints + i];

            float x = point.x;
            float y = point.y;
            float z = point.z;

            trianglePoints[3*i] = new Vector3(x,y,z);
            trianglePoints[(3*i)+1] = new Vector3(x+pointSize,y,z);
            trianglePoints[(3*i)+2] = new Vector3(x+pointSize,y+pointSize,z);
        }

        return trianglePoints;
    }
    private int[] SetTriangles(int nPoints){
        int[] triangles = new int[3*nPoints];
        for(int i=0; i<3*nPoints; i++) {
            triangles[i] = i;
        }
        return triangles;
    }
    */

    // IEnumerator PlayPointCloudFromFile() {
    //     while (GetPCDFile()) {
    //         yield return new WaitForSeconds(pc_period);
    //         pc_frame += 1;
    //     }
    // }

    void PlayPointCloudFromFile() {

        if (isDrawing) {
            Debug.Log("SKIPPED PC FRAME: Still drawing previous frame.");
            return;
        }

        Debug.Log("PlayPointCloudFromFile");

        if (!GetPCDFile()) {
            CancelInvoke();
        }
    }

    bool GetPCDFile()
    {
        Debug.Log("GetPCDFile");
        var watch = new System.Diagnostics.Stopwatch();
        watch.Start();

        #if UNITY_ANDROID && !UNITY_EDITOR
            string dir = Application.persistentDataPath + "/PCD";
        #else
            string dir = Application.dataPath + "/ROS/PCD";
        #endif

        string substr = string.Format("{0:000}_*.bin", pc_frame);
        List<string> paths = new List<string>(Directory.GetFiles(dir, substr));

        if (paths.Count == 0) {
            return false;
        } else {
            pc_path = paths[0];
        }
        
        watch.Stop();
        Debug.Log("1. GetPCDFile Execution Time: "+watch.ElapsedMilliseconds+" ms");

        timeToLoadNewPCFrame = true;
        return true;

    }

    void ReadPCDFile() {

        Debug.Log("ReadPCDFile");
        var watch = new System.Diagnostics.Stopwatch();
        watch.Start();

        byteArray = File.ReadAllBytes(pc_path);
        point_step = 12;
        size = byteArray.Length / point_step;

        watch.Stop();
        Debug.Log("2. READ FILE Execution Time: "+watch.ElapsedMilliseconds+" ms");

        StartCoroutine(PointCloudRendering());
        pc_frame += 1;
    }
}