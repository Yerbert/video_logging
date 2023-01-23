import rosbag
from sensor_msgs.msg import PointCloud2
import sys
import os
from rospy_message_converter import json_message_converter
import json
import shutil

POINT_CLOUD_HZ = 3.33

# bag_name = 'SceneRecordingCorrectMapped2.bag'
assert len(sys.argv) >= 3, "Must give arguments <in-rosbag> and <destination-folder>"
bag_name = sys.argv[1]
dest_folder = sys.argv[2]
pcd_folder = dest_folder+"/PCD"

os.mkdir(dest_folder)
os.mkdir(pcd_folder)
bag = rosbag.Bag(bag_name)
tf_json = open("{}/tf_msgs.json".format(dest_folder), 'w')
tf_json.write('{\n"messages": [')
first_tf = True
created_pcd_json = False

for topic, msg, t in bag.read_messages():
    if topic == '/velodyne_points/processed':
        # f = open("./PCD/{}_{}.bin".format("{0:03d}".format(counter), t), 'w')
        f = open("{}/{}.bin".format(pcd_folder, t), 'w')
        f.write(msg.data)
        f.close()
        if not created_pcd_json:
            pcd_json = open("{}/pcd_info.json".format(pcd_folder), 'w')
            pcd_json.write(json.dumps({"point_step": msg.point_step, "hz": POINT_CLOUD_HZ}))
            pcd_json.close()
            created_pcd_json = True
    if topic == '/tf' and msg.transforms[0].child_frame_id == 'base_link':
        if not first_tf:
            tf_json.write(',')  # to remove trailing comma
        else:
            first_tf = False
        tf_json.write("\n{}".format(json_message_converter.convert_ros_message_to_json(msg)))

tf_json.write("\n]\n}")
tf_json.close()

#Video file
os.system("python ./rosbag2video/rosbag2video.py {}".format(bag_name))
videofile = [l for l in os.listdir("./") if ".mp4" in l][0]
shutil.move("./{}".format(videofile), "./{}/VideoFeed.mp4".format(dest_folder))

print("\nCONVERSION COMPLETE\nResulting files in '{}'".format(dest_folder))