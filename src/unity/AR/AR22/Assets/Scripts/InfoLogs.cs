using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using Unity.Robotics.ROSTCPConnector;
using String = RosMessageTypes.Std.StringMsg;
using TMPro;

public class InfoLogs : MonoBehaviour
{

    public GameObject messageObject;
    public GameObject contentArea;
    public Scrollbar scrollBar;

    Coroutine scrollCoroutine;

    void Awake() {
        StartConnection();
    }

    void StartConnection() {
        ROSConnection ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<String>("/infologs", AddMessage);
    }

    void AddMessage(String msgText) {
        // Create new message object
        GameObject msgObj = Instantiate(messageObject, new Vector3(0,0,0), Quaternion.identity);
        msgObj.transform.SetParent(contentArea.transform);
        msgObj.transform.localScale = new Vector3(1f,0.83f,1f);
        msgObj.transform.localPosition = new Vector3(0f,0f,0f);
        msgObj.transform.localRotation = Quaternion.identity;
        msgObj.GetComponentInChildren<TextMeshProUGUI>().SetText(msgText.data);
        // Smoothly scroll to bottom
        StopAllCoroutines();
        scrollCoroutine = StartCoroutine(ScrollDown());
    }

    // Coroutine to smoothly scroll to the bottom
    IEnumerator ScrollDown() {
        yield return new WaitForSeconds(0.1f);
        while (scrollBar.value > 0f) {
            scrollBar.value = scrollBar.value - (0.2f * (Mathf.Max(scrollBar.value, 0.01f)));
            yield return null;
        }
        scrollBar.value = 0f;
        Debug.Log("DONE SCROLLING");
    }
}
