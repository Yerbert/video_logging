// using System.Collections;
// using System.Collections.Generic;
// using System.IO;
// using UnityEngine;

// public class PCDReader : MonoBehaviour
// {
//     // Start is called before the first frame update
//     void ReadPCD()
//     {
//         string path = Application.dataPath + "/ROS/SceneRecording_PointCloud/1673914665.692276239.pcd";
//         Debug.Log(path);
        
//         FileStream fs = new FileStream (path, FileMode.Open, FileAccess.Read);
//         StreamReader sr = new StreamReader(fs);
//         BinaryReader br = new BinaryReader(fs);

//         sr.ReadLine();
//         sr.ReadLine();

//         string[] fields = sr.ReadLine().Split();

//         string[] sizes = sr.ReadLine().Split();

//         int point_step = 0;
//         for(int i=1; i<sizes.Length; i++) {
//             point_step += int.Parse(sizes[i]);
//         }


//         sr.ReadLine();
//         sr.ReadLine();

//         int width = int.Parse((sr.ReadLine().Split())[1]);
        
//         sr.ReadLine();
//         sr.ReadLine();
//         sr.ReadLine();
//         sr.ReadLine();

//         byte[] data = br.ReadBytes(width*point_step);
//         // string data = sr.ReadToEnd();

//         foreach(string field in fields) {
//             Debug.Log("Field: " + field);
//         }
//         Debug.Log("Width: " + width);
//         Debug.Log("Point step: " + point_step);
//         Debug.Log("data length: " + data.Length);

        
//     }
// }
