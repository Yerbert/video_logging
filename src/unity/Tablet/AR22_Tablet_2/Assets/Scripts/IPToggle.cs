using System.Collections;
using System.Collections.Generic;
using UnityEngine;

using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std;


public class IPToggle : MonoBehaviour
{
    public string ipTopic = "/condition";
    private int connectionDelaySeconds = 5;
    ROSConnection ros;

    public GameObject progressBar;

    void Awake() {
        StartConnection();
    }

    void StartConnection()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<StringMsg>(ipTopic, ReceivedIPMessage);
    }

    void ReceivedIPMessage(StringMsg msg) {

        ros.Disconnect();

        string device = msg.data.Split('+')[0];
        string dataStream = msg.data.Split('+')[1];

        if ( (device == "AR " && dataStream == " replay") || (device == "Tablet " && dataStream == " live") ) {
            StartCoroutine(Reconnect(StartUp.jackal_IP));
            progressBar.SetActive(false);
        }

        if ( (device == "AR " && dataStream == " live") || (device == "Tablet " && dataStream == " replay") ) {
            StartCoroutine(Reconnect(StartUp.laptop_IP));
            progressBar.SetActive(true);
        }
        
    }

    IEnumerator Reconnect(string ip) {
        yield return new WaitForSeconds(connectionDelaySeconds);

        ros.Connect(ip, 10000);
    }

}
