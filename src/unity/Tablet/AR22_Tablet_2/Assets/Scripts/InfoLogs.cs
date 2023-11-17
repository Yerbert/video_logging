using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using Unity.Robotics.ROSTCPConnector;
using String = RosMessageTypes.Std.StringMsg;
using Int8 = RosMessageTypes.Std.Int8Msg;
using Bool = RosMessageTypes.Std.BoolMsg;
using TMPro;

public class InfoLogs : MonoBehaviour
{
    public GameObject messageObject;
    public GameObject contentArea;

    private List<GameObject> infoMessages;
    private bool isAwaitingData;

    void Awake() {
        Debug.Log("Infolog awake");
        StartConnection();
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
        Debug.Log("RECEIVED MESSAGE");
        // Create new message object
        if(isAwaitingData)
        {
            GameObject msgObj = Instantiate(messageObject, new Vector3(0,0,0), Quaternion.identity);
            msgObj.transform.SetParent(contentArea.transform);
            msgObj.GetComponentInChildren<TextMeshProUGUI>().SetText(msgText.data);
            infoMessages.Add(msgObj);
        }
    }

    void ResetMessages(Bool msg) {
        foreach(GameObject msgObj in infoMessages)
        {
            msgObj.GetComponentInChildren<TextMeshProUGUI>().color = new Color32(100,100,100,255);
        }

        if(infoMessages.Count > 0)
        {
            isAwaitingData = false;
        }
    }

    void ActivateMessage(Int8 msgNumber) {
        if(!isAwaitingData)
        {
            infoMessages[msgNumber.data].GetComponentInChildren<TextMeshProUGUI>().color = new Color32(255,255,255,255);
        }
    }

    public void ClearInfologs() {
        foreach (Transform child in contentArea.transform) {
            GameObject.Destroy(child.gameObject);
        }
        infoMessages = new List<GameObject>();
        isAwaitingData = true;
    }
}
