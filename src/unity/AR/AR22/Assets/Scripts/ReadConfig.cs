using System;
using System.Collections;
using System.Collections.Generic;
using System.IO;
using UnityEngine;
using Newtonsoft.Json;
using System.Threading.Tasks;
using Unity.Robotics.ROSTCPConnector;

using Microsoft.MixedReality.Toolkit.Utilities;
using Microsoft.MixedReality.Toolkit.UI;
using TMPro;

#if ENABLE_WINMD_SUPPORT
using Windows.Storage;
#endif

using Microsoft.MixedReality.Toolkit.Input;

public class ReadConfig : MonoBehaviour
{
    public string defaultLaptopIP = "160.69.69.3";
    public string defaultJackalIP = "160.69.69.10";
    public GameObject[] ROSGameObjects;
    public GridObjectCollection ipSelectArea;
    public GameObject buttonPrefab;
    public ImageTrackingBehaviour imageTracking;

    public static string laptop_IP = "";
    public static string jackal_IP = "";

    #if ENABLE_WINMD_SUPPORT
    public StorageFolder AR22_folder;
    // public StorageFolder Scene_folder;
    // public StorageFolder PCD_folder;
    #endif
    
    // Start is called before the first frame update
    void Start()
    {
        Debug.Log("1. Running Start()");

        #if ENABLE_WINMD_SUPPORT
            Debug.Log("2. Running readFile()");
            readFile();
            Debug.Log("COMPLETE");
        #else
            InitROS(defaultLaptopIP, defaultJackalIP);
        #endif

        PointerUtils.SetMotionControllerRayPointerBehavior(PointerBehavior.AlwaysOff);
        PointerUtils.SetHandRayPointerBehavior(PointerBehavior.AlwaysOff);
        
    }

    void InitROS(string laptop_ip, string jackal_ip) {

        laptop_IP = laptop_ip;
        jackal_IP = jackal_ip;

        // Create ROS Connection 
        GameObject rosObj = new GameObject("ROSConnection2");
        ROSConnection rosCon = rosObj.AddComponent<ROSConnection>();

        rosCon.RosIPAddress = laptop_ip;
        rosCon.RosPort = 10000;
        rosCon.listenForTFMessages = false;

        // Enable publishers/subscribers
        foreach(GameObject g in ROSGameObjects) {
            g.SetActive(true);

            // Enable infologs clipping boxes
            ClippingBox[] clippingBoxes = g.GetComponentsInChildren<ClippingBox>(true);
            if (clippingBoxes.Length > 0) {
                clippingBoxes[0].enabled = true;
            }
        }

        // Disable tracking in editor
        #if UNITY_EDITOR
        imageTracking.disableTracking();
        #endif

    }

    #if ENABLE_WINMD_SUPPORT
    private async Task readFile() {
        AR22_folder = await KnownFolders.PicturesLibrary.GetFolderAsync("AR22");
        string text = (new StreamReader(AR22_folder.Path + @"\ar22config.json")).ReadToEnd();
        ConfigFile config = JsonConvert.DeserializeObject<ConfigFile>(text);

        InitROS(config.laptop_ip, config.jackal_ip);
    }
    #endif





    // #if ENABLE_WINMD_SUPPORT
    // private async Task readFile() {

    //     AR22_folder = await KnownFolders.PicturesLibrary.GetFolderAsync("AR22");
    //     string text = (new StreamReader(AR22_folder.Path + @"\ar22config.json")).ReadToEnd();
    //     ConfigFile config = JsonConvert.DeserializeObject<ConfigFile>(text);
        
    //     foreach(string ip in config.ip_options) {

    //         GameObject button = Instantiate(buttonPrefab, new Vector3(0,0,0), Quaternion.identity);
    //         button.transform.SetParent(ipSelectArea.transform);
    //         button.transform.localScale = new Vector3(1f,1f,1f);
    //         button.transform.localPosition = new Vector3(0f,0f,0f);
    //         button.transform.localRotation = Quaternion.identity;

    //         button.GetComponent<Interactable>().OnClick.AddListener( () => ButtonPress(ip) );

    //         TextMeshPro tmp = button.GetComponentInChildren<TextMeshPro>();
    //         tmp.SetText(ip);
    //         tmp.fontSize = 0.07f;
    //         tmp.alignment = TextAlignmentOptions.Center;

    //     }
    //     ipSelectArea.UpdateCollection();
    // }
    // #endif

    // public void ButtonPress(string ip) {
    //     InitROS(ip);
    //     ipSelectArea.gameObject.SetActive(false);
    // }



    // #if ENABLE_WINMD_SUPPORT
    // private async Task readFile() {

    //     Debug.Log("3. Opening folders");
    //     AR22_folder = await KnownFolders.PicturesLibrary.GetFolderAsync("AR22");
    //     // Scene_folder = await AR22_folder.GetFolderAsync("Scene");
    //     // PCD_folder = await Scene_folder.GetFolderAsync("PCD");
    //     Debug.Log("done");

    //     Debug.Log("4. Opening file and 5. Reading");
    //     string text = (new StreamReader(AR22_folder.Path + @"\ar22config.json")).ReadToEnd();
    //     // StorageFile file = await AR22_folder.GetFileAsync("ar22config.json");
    //     Debug.Log("done");

    //     // Debug.Log("5. Reading file");
    //     // string text = await Windows.Storage.FileIO.ReadTextAsync(file);
    //     // Debug.Log("done");

    //     Debug.Log("6. Converting to JSON");
    //     ConfigFile config = JsonConvert.DeserializeObject<ConfigFile>(text);
    //     Debug.Log("done");

    //     Debug.Log("7. Returning ...");
    //     Debug.Log("IP ADDRESS: " + config.ip);

    //     InitROS(config.ip);
    // }
    // #endif


    // #if ENABLE_WINMD_SUPPORT
    // private async Task<ConfigFile> pickFile() {

    //     Debug.Log("3. Creating FileOpenPicker()");
    //     FileOpenPicker openPicker = new FileOpenPicker();
    //     openPicker.ViewMode = PickerViewMode.Thumbnail;
    //     openPicker.SuggestedStartLocation = PickerLocationId.PicturesLibrary;
    //     openPicker.FileTypeFilter.Add(".json");
    //     Debug.Log("done");

    //     Debug.Log("4. Opening picker");
    //     StorageFile file = await openPicker.PickSingleFileAsync();
    //     Debug.Log("done");

    //     Debug.Log("5. Reading file");
    //     string text = await Windows.Storage.FileIO.ReadTextAsync(file);
    //     Debug.Log("done");

    //     Debug.Log("6. Converting to JSON picker");
    //     ConfigFile config = JsonConvert.DeserializeObject<ConfigFile>(text);
    //     Debug.Log("done");

    //     Debug.Log("7. Returning ...");
    //     return config;
    // }
    // #endif
}

public class ConfigFile {
    public string laptop_ip {get; set;}
    public string jackal_ip {get; set;}
}