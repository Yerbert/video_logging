using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using RosMessageTypes.Sensor;
using RosMessageTypes.Geometry;
using RosMessageTypes.Nav;
using System.Linq;

public class MapSubscriber : MonoBehaviour
{

    public GameObject mapPlaneOrigin;
    public Material mapMaterial;
    Mesh mapMesh;
    Texture2D displayTexture;
    Material material;

    public Transform DelocalisationFrame;

    public Color Unoccupied = Color.clear;
    public Color Occupied = Color.white;
    public Color Unknown = Color.clear;

    void Awake() {
        StartConnection();
    }

    void StartConnection()
    {
        InitMap();
        
        ROSConnection ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<OccupancyGridMsg>("/map", SetMap);
        ros.Subscribe<TransformMsg>("/delocalisation", SetDelocalisation);
    }

    void InitMap() {

        // Create mesh
        mapMesh = new Mesh();
        mapMesh.vertices = new[]
        { Vector3.zero, new Vector3(0, 0, 1), new Vector3(1, 0, 1), new Vector3(1, 0, 0) };
        mapMesh.uv = new[] { Vector2.zero, Vector2.up, Vector2.one, Vector2.right };
        mapMesh.triangles = new[] { 0, 1, 2, 2, 3, 0 };

        // Assign mesh and material
        mapPlaneOrigin.AddComponent<MeshFilter>();
		mapPlaneOrigin.AddComponent<MeshRenderer>();
		mapPlaneOrigin.GetComponent<Renderer>().material = mapMaterial;
		mapPlaneOrigin.GetComponent<MeshFilter>().mesh = mapMesh;

        // Create map texture
        displayTexture = new Texture2D(1, 1, TextureFormat.RGBA32, true);
        displayTexture.wrapMode = TextureWrapMode.Clamp;
        displayTexture.filterMode = FilterMode.Point;
        material = mapPlaneOrigin.GetComponent<Renderer>().material;
        mapPlaneOrigin.GetComponent<Renderer>().material.mainTexture = displayTexture;
        ClearMap();
    }

    void SetMap(OccupancyGridMsg msg) {

        // Color data
        // Color[] colour_data = msg.data.Select(x => x==-1 ? Unknown : Color.Lerp(Unoccupied, Occupied, ((float)x)/100)).ToArray();
        int numPixels = msg.data.Length;
        Color[] colour_data = new Color[numPixels];
        for(int i=0; i<numPixels; i++) {
            colour_data[i] = msg.data[i]>0 ? Occupied : Unoccupied;
        }
        
        // Map texture
        displayTexture.Resize((int)msg.info.width, (int)msg.info.height);
        displayTexture.SetPixels(colour_data);
        displayTexture.Apply();

        // Map size and position
        mapPlaneOrigin.transform.localScale = new Vector3( ((float)msg.info.width) * msg.info.resolution, 1, ((float)msg.info.height) * msg.info.resolution);
        mapPlaneOrigin.transform.localPosition = msg.info.origin.position.From<FLU>();
        mapPlaneOrigin.transform.localRotation = msg.info.origin.orientation.From<FLU>();
        mapPlaneOrigin.transform.Rotate(new Vector3(0,-90,0)); // Rotation to fix misalignment
    }

    public void SetDelocalisation(TransformMsg msg) {
        DelocalisationFrame.transform.localPosition = msg.translation.From<FLU>();
        DelocalisationFrame.transform.localRotation = msg.rotation.From<FLU>();
    }

    public void ClearMap() {
        displayTexture.Resize(1, 1);
        displayTexture.SetPixels(new Color[] {Color.clear});
        displayTexture.Apply();
    }

    public void ClearDelocalisation() {
        DelocalisationFrame.transform.localPosition = Vector3.zero;
        DelocalisationFrame.transform.localRotation = Quaternion.identity;
    }
}