#!/usr/bin/env python

import rospy as rp
import rospkg
import json
import sys
from std_msgs.msg import String
from geometry_msgs.msg import Vector3


class InfoLogsPublisher:
    def __init__(self, rosbag_name):
        self.pub1 = rp.Publisher("/infologs", String, queue_size=10)
        self.pub2 = rp.Publisher("/goal", String, queue_size=10)
        self.bag_name = rosbag_name
        self.publish_infologs_and_goal()

    def publish_infologs_and_goal(self):
        fname = rospkg.RosPack().get_path('process_messages') + '/src/infologs.json'
        f = open(fname)
        data = json.load(f)
        messages = None
        goal = None
        for bag in data:
            if(self.bag_name.endswith(bag["bag_name"])):
                messages = bag["infologs"]
                goal = bag["goal"]
                f.close()
                break
        if(messages == None or goal == None):
            print("[ERROR] Infologs and/or Goal for this bag are undefined in infologs.json file!")
            f.close()
            sys.exit(0)
        for i in range(len(messages)):
            rp.sleep(0.1) 
            msg = "[ Timestamp: {:.2f} secs ]\n".format(messages[i]["time"]) + messages[i]["text"]
            self.pub1.publish(String(msg))
        self.pub2.publish(Vector3(goal[0],goal[1],goal[2]))       

if __name__ == '__main__':
    rp.init_node('live_infologs')
    infologs = InfoLogsPublisher(rosbag_name = sys.argv[1])
    sys.exit(0)
