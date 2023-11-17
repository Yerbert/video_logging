using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ControlBehaviour : MonoBehaviour
{

    // Start is called before the first frame update
    void Start()
    {
        /*
        Every 60s, manually call garbage collector.
        For some reason, there are memory leak issues when running on the
        Hololens 2, even when there are no memory leaks while testing on the PC.
        */
        InvokeRepeating("GarbageCollection", 0f, 60f);
    }

    void GarbageCollection() {
        Debug.Log("GC");
        Resources.UnloadUnusedAssets();
        System.GC.Collect();
    }
}
