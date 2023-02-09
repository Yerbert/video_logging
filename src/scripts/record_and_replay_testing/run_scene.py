#!/usr/bin/env python

import rospy
import subprocess
import os
import signal
import psutil
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
# from time import sleep

# Global Variables
p1 = None
p2 = None
p2Process = None
pause = False
service = ""

# Signal Handler to kill all processes
def signal_handler(sig, frame):
    os.killpg(os.getpgid(p1.pid), signal.SIGTERM)
    os.killpg(os.getpgid(p2.pid), signal.SIGTERM) 
    sys.exit(0)

# Pause flipflop mechanism
def pause_callback(msg):

    global pause,service,p2Process

    # Skip repeated/unsynced pauses
    if (msg.data == pause):
        return

    pause = not pause
    if pause:
        rospy.loginfo("Paused")
        command = "rosservice call " + service + " true"
        os.system(command)
        p2Process.suspend()
    else:
        rospy.loginfo("Resumed")
        command = "rosservice call " + service + " false"
        os.system(command)
        p2Process.resume()


if __name__ == '__main__':
    
    # Parse Arguments
    argParser = argparse.ArgumentParser()
    argParser.add_argument("--rosbag", help="Rosbag to be played back")
    argParser.add_argument("--infologs", help="Info logs python file to be played back")
    args = argParser.parse_args()

    # Signal Handler to kill subprocesses
    signal.signal(signal.SIGINT, signal_handler)

    # Working Directory (CHANGE THIS)
    cwd_path = os.getcwd()

    #Initialise ROS node to enable communication with other nodes
    rospy.init_node('rosbag_replay')

    #Initialize Rostopics publisher and subscribers
    tf_offset_pub = rospy.Publisher("/tf/path", TFMessage, queue_size=10)
    pause_sub = rospy.Subscriber("/pause", Bool, pause_callback)

    # Find the last /tf rostopic and publish it
    tf_offset_transforms = []
    for topic, msg, t in rosbag.Bag(args.rosbag).read_messages():
        if topic == "/tf":
            for tf in msg.transforms:
                if tf.child_frame_id == "base_link" and tf.header.frame_id == "odom":
                    # print(msg)
                    tf_offset_transforms.append(tf)
    tf_offset_pub.publish(TFMessage(tf_offset_transforms))

    # Sleep 2 seconds
    rospy.sleep(2.0)

    # Start subprocess to play rosbag
    command = "rosbag play -l " + args.rosbag
    p1 = subprocess.Popen(command, stdin=subprocess.PIPE, shell=True, cwd=cwd_path, executable='/bin/bash', preexec_fn=os.setsid)
    rospy.sleep(0.5)

    # Find the rosservice for pausing/playing rosbag
    list_cmd = subprocess.Popen("rosservice list", shell=True, stdout=subprocess.PIPE)
    list_output = list_cmd.stdout.read()
    retcode = list_cmd.wait()
    assert retcode == 0, "List command returned %d" % retcode
    for string in list_output.decode('ascii').split("\n"):
        if (string.endswith('pause_playback')):
            service = string
            print("\nPausing service: " + service)

    # Start the infologs file
    command = "python3 " + args.infologs
    p2 = subprocess.Popen(command, stdin=subprocess.PIPE, shell=True, cwd=cwd_path, executable='/bin/bash', preexec_fn=os.setsid)
    p2Process = psutil.Process(pid=p2.pid)

    # Infinite loop to keep parent process running and listen in to pause/play functionality
    rospy.spin()
    