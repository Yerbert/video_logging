from rospy import rostime
import rosbag
from std_msgs.msg import String, Bool, Int8
from video_logging.msg import ClearScenario
from geometry_msgs.msg import Vector3
import sys
import shutil
import os
import json
import time

def insert_infologs(bag_name, new_bag_name):
    bag = rosbag.Bag(new_bag_name, 'a')
    start_time = bag.get_start_time()
    messages = load_infologs(bag_name)
    for i in range(len(messages)):
        msg = "[ Timestamp: {:.2f} secs ]\n".format(messages[i]["time"]) + messages[i]["text"]
        bag.write("/infologs", String(msg), rostime.Time.from_seconds(start_time))
        bag.write("/infologs/activate", Int8(i), rostime.Time.from_seconds(start_time + messages[i]["time"]))
    bag.close()

def load_infologs(bag_name):
    f = open('infologs.json')
    data = json.load(f)
    for bag in data:
        if(bag_name.endswith(bag["bag_name"])):
            f.close()
            return bag["infologs"]
    print("[ERROR] Infologs for this bag are undefined in infologs.json file!")
    f.close()
    sys.exit(0)

def insert_goal(bag_name):
    bag = rosbag.Bag(new_bag_name, 'a')
    start_time = bag.get_start_time()
    messages = load_goal(bag_name)
    bag.write("/goal", Vector3(messages[0],messages[1],messages[2]), rostime.Time.from_seconds(start_time))
    bag.close()

def load_goal(bag_name):
    f = open('infologs.json')
    data = json.load(f)
    for bag in data:
        if(bag["bag_name"][:-4] in bag_name):
            f.close()
            return bag["goal"]
    print("[ERROR] Goal Coordinates for this bag are undefined in infologs.json file!")
    f.close()
    sys.exit(0)

def insert_fake_object_signal(new_bag_name):
    bag = rosbag.Bag(new_bag_name, 'a')
    start_time = bag.get_start_time()
    messages = load_fake_object_signal(bag_name)
    for i in range(len(messages)):
        bag.write("/fake_object", Bool(messages[i]["bool"]), rostime.Time.from_seconds(start_time + messages[i]["time"]))
    bag.close()

def load_fake_object_signal(bag_name):
    f = open('infologs.json')
    data = json.load(f)
    for bag in data:
        if(bag["bag_name"][:-4] in bag_name):
            f.close()
            return bag["fakeObjectSignal"]
    print("[ERROR] Fake Object Signals for this bag are undefined in infologs.json file!")
    f.close()
    sys.exit(0)

def insert_velodyne_flicker(bag_name):
    new_bag_name = bag_name[:-4] + "_temp.bag"
    bag = rosbag.Bag(bag_name, 'a')
    start_time = bag.get_start_time()
    bag.close()
    cond1 = "(topic != '/tf' and topic != '/velodyne_points')"
    cond2 = "(t.to_sec() <= {})".format((start_time + 2))
    cond3 = "(t.to_sec() >= {} and t.to_sec() <= {})".format((start_time + 4), (start_time + 6.3))
    cond4 = "(t.to_sec() >= {} and t.to_sec() <= {})".format((start_time + 8.3), (start_time + 10.1))
    cond5 = "(t.to_sec() >= {} and t.to_sec() <= {})".format((start_time + 13.1), (start_time + 14.4))
    os.system("rosbag filter " + bag_name + " " + new_bag_name + " \"{} or {} or {} or {} or {}\"".format(cond1, cond2, cond3, cond4, cond5))
    
    new_bag = rosbag.Bag(new_bag_name, 'a')
    new_bag.write("/clear_scenario", ClearScenario(point_cloud = True), rostime.Time.from_seconds(start_time + 2.1))
    new_bag.write("/clear_scenario", ClearScenario(point_cloud = True), rostime.Time.from_seconds(start_time + 6.4))
    new_bag.write("/clear_scenario", ClearScenario(point_cloud = True), rostime.Time.from_seconds(start_time + 10.2))
    new_bag.write("/clear_scenario", ClearScenario(point_cloud = True), rostime.Time.from_seconds(start_time + 14.5))
    new_bag.close()
    os.system("mv " + new_bag_name + " " + bag_name)

def insert_camera_failure(bag_name):
    new_bag_name = bag_name[:-4] + "_temp.bag"
    bag = rosbag.Bag(bag_name, 'a')
    start_time = bag.get_start_time()
    bag.close()
    cond1 = "(topic != '/camera/color/image_raw/compressed')"
    cond2 = "(t.to_sec() <= {})".format((start_time + 8))
    os.system("rosbag filter " + bag_name + " " + new_bag_name + " \"{} or {}\"".format(cond1, cond2))
    os.system("mv " + new_bag_name + " " + bag_name)

def insert_start_message(bag_name):
    bag = rosbag.Bag(bag_name, 'a')
    start_time = bag.get_start_time()
    bag.write("/infologs/start", Bool(True), rostime.Time.from_seconds(start_time + 0.1))
    bag.close()

if __name__ == "__main__":

    bag_name = sys.argv[1]
    new_bag_name = bag_name[:-4] + "_with_infologs.bag"
    shutil.copyfile(bag_name, new_bag_name)
    insert_infologs(bag_name, new_bag_name)
    insert_start_message(new_bag_name)
    insert_goal(new_bag_name)

    if(bag_name.endswith("FakeObstruction.bag")):
        insert_fake_object_signal(new_bag_name)

    if(bag_name.endswith("VelodyneError.bag")):
        insert_velodyne_flicker(new_bag_name)

    if(bag_name.endswith("CameraFailure.bag")):
        insert_camera_failure(new_bag_name)

    # Fix rosbag timings due to inserting messages asynchronously
    os.system("rosbag fix " + new_bag_name + " " + new_bag_name)
