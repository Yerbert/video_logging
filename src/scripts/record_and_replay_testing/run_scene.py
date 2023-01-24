#!/usr/bin/env python

import rospy
import subprocess
import os
import signal
import sys
import argparse

import numpy as np
import time
from datetime import datetime
import rospkg
import rosbag

from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import Int32, Float32, String, Bool
from tf2_msgs.msg import TFMessage
from time import sleep

p1 = None
p2 = None

def signal_handler(sig, frame):
    os.killpg(os.getpgid(p1.pid), signal.SIGTERM)
    os.killpg(os.getpgid(p2.pid), signal.SIGTERM) 
    sys.exit(0)

if __name__ == '__main__':
    
    # Parse Arguments
    argParser = argparse.ArgumentParser()
    argParser.add_argument("--rosbag", help="Rosbag to be played back")
    argParser.add_argument("--infologs", help="Info logs python file to be played back")
    args = argParser.parse_args()

    # Signal Handler to kill subprocesses
    signal.signal(signal.SIGINT, signal_handler)

    # Working Directory (CHANGE THIS)
    cwd_path = "/home/kelvin/ar_vr_ws/recorded_bag_files"

    #Initialise ROS node to enable communication with other nodes
    rospy.init_node('rosbag_replay')

    #Initialize Rostopics publisher for /tf offset
    tf_offset_pub = rospy.Publisher("/tf/offset", TFMessage, queue_size=10)

    # Find the last /tf rostopic and publish it
    tf_offset_msg = None
    for topic, msg, t in rosbag.Bag(args.rosbag).read_messages():
        if topic == "tf":
            tf_offset_msg = msg
    tf_offset_pub.publish(tf_offset_msg)

    # Sleep 2 seconds
    sleep(2.0)

    # Start subprocess to play rosbag
    command = "rosbag play -l " + args.rosbag
    p1 = subprocess.Popen(command, stdin=subprocess.PIPE, shell=True, cwd=cwd_path, executable='/bin/bash', preexec_fn=os.setsid)

    # Start the infologs file
    command = "python3 " + args.infologs
    p2 = subprocess.Popen(command, stdin=subprocess.PIPE, shell=True, cwd=cwd_path, executable='/bin/bash', preexec_fn=os.setsid)

    # Infinite loop to keep parent process running
    while True:
        print("Running...")
        sleep(2.0)
    

    

    