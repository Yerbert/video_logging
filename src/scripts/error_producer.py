#!/usr/bin/env python3

import rospy
import subprocess
import os
import signal

import numpy as np
import time
from datetime import datetime
import rospkg
from std_msgs.msg import Bool

if __name__ == '__main__':
    #parser = argparse.ArgumentParser(description='Code to produce an "error" for testing functionality')
    #parser.add_argument("--z-offset", type=float, nargs='?', const=True, default=0.0, help="Height Offset for Debugging")
    #args = parser.parse_args()

    #Initialise ROS node to enable communication with other nodes
    rospy.init_node('error_producer')

    #Open a publisher which errors can be published to
    error_pub = rospy.Publisher('/error_detected', Bool, queue_size=10)

    rospy.sleep(10.)
    error_pub.publish(1)
    rospy.sleep(1.)
