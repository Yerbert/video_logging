from rospy import rostime
import rosbag
from std_msgs.msg import String, Bool, Int8
import sys
import shutil
import os
import json

def insert_infologs(bag_name, new_bag_name, messages):
    bag = rosbag.Bag(new_bag_name, 'a')
    start_time = bag.get_start_time()
    for i in range(len(messages)):
        msg = "[ Timestamp: {:.2f} secs ]\n".format(messages[i]["time"]) + messages[i]["text"]
        bag.write("/infologs", String(msg), rostime.Time.from_seconds(start_time))
        bag.write("/infologs/activate", Int8(i), rostime.Time.from_seconds(start_time + messages[i]["time"]))
    bag.close()

def insert_start_message(bag_name):
    bag = rosbag.Bag(bag_name, 'a')
    start_time = bag.get_start_time()
    bag.write("/infologs/start", Bool(True), rostime.Time.from_seconds(start_time + 0.1))
    bag.close()

def load_messages(bag_name):
    f = open('infologs.json')
    data = json.load(f)
    for bag in data:
        if(bag_name.endswith(bag["bag_name"])):
            f.close()
            return bag["infologs"]
    print("[ERROR] Infologs for this bag are undefined in infologs.json file!")
    f.close()
    sys.exit(0)

if __name__ == "__main__":

    bag_name = sys.argv[1]
    messages = load_messages(bag_name)
    new_bag_name = bag_name[:-4] + "_with_infologs.bag"
    shutil.copyfile(bag_name, new_bag_name)
    insert_infologs(bag_name, new_bag_name, messages)
    insert_start_message(new_bag_name)

    # Fix rosbag timings due to inserting messages asynchronously
    os.system("rosbag fix " + new_bag_name + " " + new_bag_name)
