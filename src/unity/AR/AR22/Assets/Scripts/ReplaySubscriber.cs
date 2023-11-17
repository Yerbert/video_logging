// using System.Collections;
// using System.Collections.Generic;
// using UnityEngine;
// using UnityEngine.UI;
// using Unity.Robotics.ROSTCPConnector;

// using CompressedImage = RosMessageTypes.Sensor.CompressedImageMsg;
// using Image = RosMessageTypes.Sensor.ImageMsg;
// using Float32 = RosMessageTypes.Std.Float32Msg;
// using Bool = RosMessageTypes.Std.BoolMsg;

// public class ReplaySubscriber : MonoBehaviour
// {

//     public GameObject display;
//     public Slider progressBar;

//     private Texture2D displayTexture;

//     private int imageHeight = 480;
//     private int imageWidth = 640;

//     private bool paused = false;
//     private ROSConnection ros;


//     public void StartConnection()
//     {
//         ros = ROSConnection.GetOrCreateInstance();
//         ros.Subscribe<CompressedImage>("/replay/camera/color/image_raw/compressed", ShowVideo);
//         ros.Subscribe<Float32>("/replay/progress", UpdateProgressBar);
//         ros.RegisterPublisher<Bool>("/pause");
//         ros.RegisterPublisher<Bool>("/error_resolved");

//         displayTexture = new Texture2D(imageWidth, imageHeight, TextureFormat.RGBA32, false);
//         display.GetComponent<Renderer>().material.mainTexture = displayTexture;
//     }

//     void ShowVideo(CompressedImage ImgMsg)
//     {
//         Destroy(display.GetComponent<Renderer>().material.mainTexture); // to prevent memory leak
//         Texture2D tex = new Texture2D(imageWidth, imageHeight);
//         tex.LoadImage(ImgMsg.data);
//         display.GetComponent<Renderer>().material.mainTexture = tex;
//     }

//     void UpdateProgressBar(Float32 prog) {
//         progressBar.value = prog.data;
//     }

//     public void PausePlay()
//     {
//         paused = !paused;
//         ros.Publish("/pause", new Bool(paused));
//         Debug.Log("Paused: "+paused);
//     }

//     public void Exit()
//     {
//         ros.Publish("/error_resolved", new Bool(true));
//         Debug.Log("Exit");
//     }
// }