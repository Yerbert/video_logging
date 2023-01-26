from rospy import rostime
import rosbag
from collections import namedtuple
from std_msgs.msg import String
import sys
import shutil

InfoLog = namedtuple("InfoLog", ["time", "message"])

def insert_infologs(bag_name, messages):
    bag = rosbag.Bag(bag_name, 'a')
    start_time = bag.get_start_time()
    for infolog in messages:
        bag.write("/infologs", String(infolog.message), rostime.Time.from_seconds(infolog.time + start_time))
    bag.close()


if __name__ == "__main__":

    messages = [
        InfoLog(0.5, "1. Heading to Point B..."),
        InfoLog(15,  "2. done"),
        InfoLog(16,  "3. Exiting arena..."),
        InfoLog(25,  "4. ERROR [Navigation] - No suitable path to destination")
    ]

    bag_name = sys.argv[1]
    new_bag_name = bag_name[:-4] + "_with_infologs.bag"
    shutil.copyfile(bag_name, new_bag_name)
    insert_infologs(new_bag_name, messages)

