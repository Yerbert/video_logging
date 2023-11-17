using System.IO;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using Unity.Robotics.ROSTCPConnector;
using CompressedImage = RosMessageTypes.Sensor.CompressedImageMsg;
using Bool = RosMessageTypes.Std.BoolMsg;
using TMPro;

public class CameraSubscriber : MonoBehaviour
{
    public Image display;
    public TMP_InputField topic_name;

    private Texture2D displayTexture;

    private bool ready = false;

    void Awake() {
        StartConnection();
    }

    void StartConnection()
    {
        ROSConnection ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<CompressedImage>(topic_name.text, ShowVideo);

        displayTexture = new Texture2D(1, 1, TextureFormat.RGBA32, false);
        display.material.mainTexture = displayTexture;

        ClearVideo();
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