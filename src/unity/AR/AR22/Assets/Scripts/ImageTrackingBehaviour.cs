using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Vuforia;
using UnityEngine.UIElements;
using TMPro;

public class ImageTrackingBehaviour : MonoBehaviour
{

    public TextMeshPro markerStatus;

    private bool skippedEnableVuforiaEventFire = true;

    public void disableTracking() {

        if (!skippedEnableVuforiaEventFire) {
            skippedEnableVuforiaEventFire = true;
            return;
        }

        VuforiaBehaviour.Instance.enabled = false;
        skippedEnableVuforiaEventFire = false;

        // re-enable mesh and line renderers after disabling tracking  
        foreach (MeshRenderer renderer in GetComponentsInChildren<MeshRenderer>()) {    
            renderer.enabled = true;
        }
        foreach (LineRenderer renderer in GetComponentsInChildren<LineRenderer>()) {
            renderer.enabled = true;
        }

        markerStatus.text = "Marker: Scanned";

        Debug.Log("TRACKING DISABLED");

    }

    public void enableTracking() {

        VuforiaBehaviour.Instance.enabled = true;

        markerStatus.text = "Marker: Not scanned";
        
        Debug.Log("TRACKING Enabled");
    }

}
