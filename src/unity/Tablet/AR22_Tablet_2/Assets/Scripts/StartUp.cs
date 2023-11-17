using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using System.Linq;

using Unity.Robotics.ROSTCPConnector;
using TMPro;


public class StartUp : MonoBehaviour
{
    public GameObject startCanvas;
    public GameObject allDisplays;
    public TMP_InputField laptop_ip;
    public TMP_InputField jackal_ip;
    public ToggleGroup connectionOptions;

    public static string laptop_IP;
    public static int laptop_PORT;
    public static string jackal_IP;
    public static int jackal_PORT;

    public void Connect() {

        GameObject rosObj = new GameObject("ROSConnection");
        ROSConnection rosCon = rosObj.AddComponent<ROSConnection>();

        string initialConnection = connectionOptions.ActiveToggles().FirstOrDefault().gameObject.name;
        if (initialConnection == "LaptopOption") {
            rosCon.RosIPAddress = laptop_ip.text;
        } else if (initialConnection == "JackalOption") {
            rosCon.RosIPAddress = jackal_ip.text;
        } else {
            Debug.LogError("Invalid option name. Check to see if the option names are consistent.");
        }

        rosCon.RosPort = 10000;
        rosCon.listenForTFMessages = false;

        laptop_IP = laptop_ip.text;
        jackal_IP = jackal_ip.text;

        startCanvas.SetActive(false);
        allDisplays.SetActive(true);
    }
}
