using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

// public class PointCloud {
//     public Vector3[] points {get; set;}
// }

public class PointCloudScenario {
    public float hz {get; set;}
    public int point_step {get; set;}
    public List<Vector3[]> pointCloudList {get; set;}

    public PointCloudScenario() {
        this.pointCloudList = new List<Vector3[]>();
    }

    public PointCloudScenario(float hz, int point_step) {
        this.hz = hz;
        this.point_step = point_step;
        this.pointCloudList = new List<Vector3[]>();
    }

    public void AddPointCloud(Vector3[] pc) {
        pointCloudList.Add(pc);
    }

    public int FrameCount() {
        return pointCloudList.Count;
    }
}

// public class PointCloudScenarioPlayer {
//     public int frame {get; set;}
//     public PointCloudScenario scenario {get; set;}
//     private float frame_period;

//     // public delegate IEnumerator DrawFunction(Vector3[] pointList);
//     // public DrawFunction drawFunction;

//     private Coroutine clock;

//     public PointCloudScenarioPlayer(PointCloudScenario scenario) {
//         this.frame = 0;
//         this.scenario = scenario;
//         this.frame_period = 1 / scenario.hz;
//     }

//     public void Start() {
//         clock = StartCoroutine(FrameClock());
//     }

//     public void Stop() {
//         if (clock != null) {
//             StopCoroutine(clock);
//         }
//     }

//     public void ScrubTo(float duration) {
//         this.frame = Mathf.FloorToInt(duration * scenario.pointCloudList.Count);
//     }

//     private IEnumerator FrameClock() {
//         while(frame < scenario.pointCloudList.Count)
//         {
//             StartCoroutine(scenario.pointCloudList[frame]);
//             frame += 1;
//             yield return new WaitForSeconds(frame_period);
//         }
//         Stop();
//     }
// }