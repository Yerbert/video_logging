#!/usr/bin/env python3

import rospy
import subprocess
import os
import signal
import sys

import numpy as np
import time
from datetime import datetime
import rospkg

import rosbag

from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import Int32, String, Bool

#Allow keyboard key to be set
from pynput import keyboard

class RosbagRecord:
    def __init__(self):
        #Set the recording folder relative to where the video_logging package is
        self.record_folder = rospkg.RosPack().get_path('video_logging') + '/bag_files'
        self.name = ""

    def start_ros_node(self):

        #Specify the topics and name (set to current rostime) to be recorded to the rosbag inside a shell command to record to a rosbag
        self.name = str(rospy.Time.now().secs)
        command = "rosbag record -O " + self.name + " /camera/color/image_raw/compressed"

        #Start subprocess to record
        self.p = subprocess.Popen(command, stdin=subprocess.PIPE, shell=True, cwd=self.record_folder,
                                      executable='/bin/bash')

    def terminate_ros_node(self, s):
        # Adapted from http://answers.ros.org/question/10714/start-and-stop-rosbag-within-a-python-script/
        list_cmd = subprocess.Popen("rosnode list", shell=True, stdout=subprocess.PIPE)
        list_output = list_cmd.stdout.read()
        retcode = list_cmd.wait()
        assert retcode == 0, "List command returned %d" % retcode

        name2 = int(self.name) + 1
        string2 = s + "_" + str(name2)
        for string in list_output.split("\n"):
            print(string)
            if (string.startswith(s+"_"+self.name) | string.startswith(string2)):
                os.system("rosnode kill " + string)

def stop_recording_handler():
    rospy.loginfo(rospy.get_name() + ' stop recording.')
    # Adapted from http://answers.ros.org/question/10714/start-and-stop-rosbag-within-a-python-script/
    list_cmd = subprocess.Popen("rosnode list", shell=True, stdout=subprocess.PIPE)
    list_output = list_cmd.stdout.read()
    retcode = list_cmd.wait()
    assert retcode == 0, "List command returned %d" % retcode

    for string in list_output.split("\n"):
        if (string.startswith("/record")):
            os.system("rosnode kill " + string)

def rosbag_extract(data):
    print("hi")
    return

def signal_handler(sig, frame):
    stop_recording_handler()
    sys.exit(0)

def error_callback(data):
    #Wait to record some data after the event
    rospy.sleep(5.)

    #Stop recording to rosbag
    stop_recording_handler()

    #small pause 
    rospy.sleep(0.1)

    #Fetch ROS time
    current_time = rospy.Time.now().secs

    #Find bag files
    usable_bag_files = []
    for entry in os.listdir(rospkg.RosPack().get_path('video_logging') + '/bag_files'):
        if entry.endswith('.bag'):
            name = entry.split('.')[0]
            try:
                float(name)
                if float(name) + 15 < current_time:
                    usable_bag_files.append(name)
            except ValueError:
                print("skipped " + entry)

    bag_file = usable_bag_files[0]

    #Specify the topics and name (set to current rostime) to be recorded to the rosbag inside a shell command to record to a rosbag
    command = 'rosbag filter ' + bag_file + '.bag replay.bag "t.to_sec() >= ' + str(float(current_time - 15)) + '"'

    record_folder = rospkg.RosPack().get_path('video_logging') + '/bag_files'

    #Start subprocess to filter a bag to get the replay data
    p = subprocess.Popen(command, stdin=subprocess.PIPE, shell=True, cwd=record_folder,
                                      executable='/bin/bash')
    
    p.wait()
    
    #FOLLOWING CODE MIGHT BE ALTERED
    #Adjust topic to replay topic
    with rosbag.Bag(rospkg.RosPack().get_path('video_logging') + '/bag_files/replay_altered.bag', 'w') as Y:
        for topic, msg, t in rosbag.Bag(rospkg.RosPack().get_path('video_logging') + '/bag_files/replay.bag'):
            if topic == "/camera/color/image_raw/compressed":
                Y.write("replay/camera/color/image_raw/compressed", msg, t)
            else:
                Y.write(topic, msg, t)

    #play rosbag
    command = 'rosbag play -l replay_altered.bag'

    #Start subprocess to play the replay data on loop
    p = subprocess.Popen(command, stdin=subprocess.PIPE, shell=True, cwd=record_folder,
                                      executable='/bin/bash')
    rospy.sleep(5.)
    return

if __name__ == '__main__':
    #parser = argparse.ArgumentParser(description='Code to record video footage of panda for up to 5 minutes')
    #parser.add_argument("--z-offset", type=float, nargs='?', const=True, default=0.0, help="Height Offset for Debugging")
    #args = parser.parse_args()

    #Initialise ROS node to enable communication with other nodes
    rospy.init_node('rosbag_record')

    #Open a subscriber which listens to the error publisher and calls a callback function if error detected
    error_sub = rospy.Subscriber("/error_detected", Bool, error_callback)

    #Initialize RosbagRecord Class
    rosbag_record = RosbagRecord()
    rosbag_record2 = RosbagRecord()

    signal.signal(signal.SIGINT, signal_handler)

    while(1):
        #Start recording to a bag file
        rosbag_record.start_ros_node()

        #Record to that bag file for 60 seconds
        rospy.sleep(30.)

        #Check existing rosbags to see if any have existed for too long
        duration_time = rospy.Time.now().secs - 300 #5 minutes
        for entry in os.listdir(rospkg.RosPack().get_path('video_logging') + '/bag_files'):
            if entry.endswith('.bag'):
                name = entry.split('.')[0]
                try:
                    float(name)
                    if float(name) < duration_time:
                        os.remove(rospkg.RosPack().get_path('video_logging') + '/bag_files/' + entry)
                except ValueError:
                    print("skipped " + entry + " as created elsewhere")
                
        if rosbag_record2.name != "":
            rosbag_record2.terminate_ros_node("/record")
        rosbag_record2.start_ros_node()        
        rospy.sleep(30.)

        #Stop recording to that bag file
        rosbag_record.terminate_ros_node("/record")

        
