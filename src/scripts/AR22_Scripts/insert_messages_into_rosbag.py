from rospy import rostime
import rosbag
from collections import namedtuple
from std_msgs.msg import String, Bool
import sys
import shutil
import os

InfoLog = namedtuple("InfoLog", ["time", "message"])

def insert_infologs(bag_name, messages):
    bag = rosbag.Bag(bag_name, 'a')
    start_time = bag.get_start_time()
    for infolog in messages:
        bag.write("/infologs", String(infolog.message), rostime.Time.from_seconds(infolog.time + start_time))
    bag.close()

def insert_start_message(bag_name):
    bag = rosbag.Bag(bag_name, 'a')
    start_time = bag.get_start_time()
    bag.write("/start", Bool(True), rostime.Time.from_seconds(start_time))
    bag.close()

if __name__ == "__main__":

    messages = [
        InfoLog(1,  "[INFO] Navigating to Goal A."),
        InfoLog(7,  "[INFO] Rerouting path."),
        InfoLog(8,  "[INFO] Rerouting path."),
        InfoLog(10,  "[IERROR] Failed to navigate to Goal A."),
    ]

    bag_name = sys.argv[1]
    new_bag_name = bag_name[:-4] + "_with_infologs.bag"
    shutil.copyfile(bag_name, new_bag_name)
    insert_infologs(new_bag_name, messages)
    insert_start_message(new_bag_name)

    # Fix rosbag timings due to inserting messages asynchronously
    os.system("rosbag fix " + new_bag_name + " " + new_bag_name)

