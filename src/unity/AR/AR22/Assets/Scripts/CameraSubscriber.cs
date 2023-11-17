using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using CompressedImage = RosMessageTypes.Sensor.CompressedImageMsg;
using Image = RosMessageTypes.Sensor.ImageMsg;

public class CameraSubscriber : MonoBehaviour {

    public GameObject display;

    private Texture2D displayTexture;

    private int imageHeight = 480;
    private int imageWidth = 640;

    void Awake() {
        StartConnection();
    }

    void StartConnection()
    {
        ROSConnection ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<CompressedImage>("/camera/color/image_raw/compressed/processed", ShowVideo);
        // ros.Subscribe<CompressedImage>("/usb_cam/image_raw/compressed", ShowVideo); // for home dev

        displayTexture = new Texture2D(imageWidth, imageHeight, TextureFormat.RGBA32, false);
        display.GetComponent<Renderer>().material.mainTexture = displayTexture;
    }

    void ShowVideo(CompressedImage ImgMsg)
    {
        displayTexture.LoadImage(ImgMsg.data);
    }

    public void ClearVideo()
    {
        int numPixels = displayTexture.GetPixels().Length;
        Color[] cols = new Color[numPixels];
        for(int i=0; i<numPixels; i++) {
            cols[i] = Color.black;
        }
        displayTexture.SetPixels(cols);
        displayTexture.Apply();
    }
}