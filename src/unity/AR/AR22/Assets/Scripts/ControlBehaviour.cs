using System.Collections;
using System.Collections.Generic;
using UnityEngine;

using Microsoft.MixedReality.Toolkit;
using Microsoft.MixedReality.Toolkit.Input;
using Microsoft.MixedReality.Toolkit.Utilities.Solvers;

public class ControlBehaviour : MonoBehaviour
{
    void Start() {

        StartCoroutine(GarbageCollection());
    }

    /*
    Every 30s, manually call garbage collector.
    For some reason, there are memory leak issues when running on the
    Hololens 2, even when there are no memory leaks while testing on the PC.
    */
    IEnumerator GarbageCollection() {
        while(true) {
            yield return new WaitForSeconds(30);
            Debug.Log("GC");
            Resources.UnloadUnusedAssets();
            System.GC.Collect();
        }
    }
}
