#!/usr/bin/env python

import rospy as rp
import rospkg
import json
import sys
from std_msgs.msg import String


class InfoLogsPublisher:
    def __init__(self, rosbag_name):
        self.pub = rp.Publisher("/infologs", String, queue_size=10)
        self.bag_name = rosbag_name
        self.publish_infologs()

    def publish_infologs(self):
        fname = rospkg.RosPack().get_path('process_messages') + '/src/infologs.json'
        f = open(fname)
        data = json.load(f)
        messages = None
        for bag in data:
            if(self.bag_name.endswith(bag["bag_name"])):
                f.close()
                messages = bag["infologs"]
                break
        if(messages == None):
            print("[ERROR] Infologs for this bag are undefined in infologs.json file!")
            f.close()
            sys.exit(0)
        for i in range(len(messages)):
            msg = "[ Timestamp: {:.2f} secs ]\n".format(messages[i]["time"]) + messages[i]["text"]
            self.pub.publish(String(msg))
            rp.sleep(0.1) 

if __name__ == '__main__':
    rp.init_node('live_infologs')
    infologs = InfoLogsPublisher(rosbag_name = sys.argv[1])
    rp.sleep(5)
    sys.exit(0)
