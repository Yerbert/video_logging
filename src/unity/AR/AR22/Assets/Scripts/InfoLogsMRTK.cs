using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using Unity.Robotics.ROSTCPConnector;
using String = RosMessageTypes.Std.StringMsg;
using Int8 = RosMessageTypes.Std.Int8Msg;
using Bool = RosMessageTypes.Std.BoolMsg;
using TMPro;
using Microsoft.MixedReality.Toolkit.UI;
using Microsoft.MixedReality.Toolkit.Utilities;

public class InfoLogsMRTK : MonoBehaviour
{

    public GameObject messageObject;
    public GridObjectCollection gridObjectCollection;
    public ScrollingObjectCollection scrollObjectCollection;

    private List<GameObject> infoMessages;
    private bool isAwaitingData;

    void Awake() {
        StartConnection();
        Debug.Log("INFO LOG AWAKE");
    }

    void StartConnection() {
        ROSConnection ros = ROSConnection.GetOrCreateInstance();
        infoMessages = new List<GameObject>();
        isAwaitingData = true;
        ros.Subscribe<String>("/infologs", AddMessage);
        ros.Subscribe<Bool>("/infologs/start", ResetMessages);
        ros.Subscribe<Int8>("/infologs/activate", ActivateMessage);
    }

    void AddMessage(String msgText) {
        if(isAwaitingData)
        {
            GameObject msgObj = Instantiate(messageObject, new Vector3(0,0,0), Quaternion.identity); // Create new message object
            msgObj.transform.SetParent(gridObjectCollection.transform); // Add to grid object collection
            msgObj.transform.localScale = new Vector3(1f,1f,1f);
            msgObj.transform.localPosition = new Vector3(0f,0f,0f);
            msgObj.transform.localRotation = Quaternion.identity;
            msgObj.GetComponentInChildren<TextMeshPro>().SetText(msgText.data); // set text to message data
            msgObj.GetComponentInChildren<TextMeshPro>().color = new Color32(132,170,220,255);
            msgObj.GetComponentInChildren<TextMeshPro>().fontSize = 0.05f;
            infoMessages.Add(msgObj);
            RefreshLogs();
        }
    }

    public void RefreshLogs() {
        gridObjectCollection.UpdateCollection();
        scrollObjectCollection.UpdateContent();
        scrollObjectCollection.MoveToIndex(1); // scroll to top
    }

    void ResetMessages(Bool msg) {
        foreach(GameObject msgObj in infoMessages)
        {
            msgObj.GetComponentInChildren<TextMeshPro>().color = new Color32(132,170,220,255);
        }

        if(infoMessages.Count > 0)
        {
            isAwaitingData = false;
        }
    }

    void ActivateMessage(Int8 msgNumber) {
        if(!isAwaitingData)
        {
            infoMessages[msgNumber.data].GetComponentInChildren<TextMeshPro>().color = new Color32(255,255,255,255);
        }
    }

    public void ClearInfologs() {
        foreach (Transform child in gridObjectCollection.transform) {
            GameObject.Destroy(child.gameObject);
        }
        infoMessages = new List<GameObject>();
        isAwaitingData = true;
    }

}
