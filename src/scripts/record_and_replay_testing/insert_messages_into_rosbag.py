from rospy import rostime
import rosbag
from collections import namedtuple
from std_msgs.msg import String, Bool
import sys
import shutil

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
        InfoLog(1,  "1) Payload on board: YES"),
        InfoLog(5,  "2) Payload on board: YES"),
        InfoLog(14, "3) Arriving at destination..."),
        InfoLog(17, "4) Payload on board: NO")
    ]

    bag_name = sys.argv[1]
    new_bag_name = bag_name[:-4] + "_with_infologs.bag"
    shutil.copyfile(bag_name, new_bag_name)
    insert_infologs(new_bag_name, messages)
    insert_start_message(new_bag_name)

