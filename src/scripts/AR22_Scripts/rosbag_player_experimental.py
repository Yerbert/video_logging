#!/usr/bin/env python3

import rospy as rp
import rosbag
import rostopic
import rosnode
import pyrosbag
from pynput.keyboard import Listener
import rospkg
import subprocess
import os
import sys
import signal
from std_msgs.msg import Bool, Float32, String, Empty
from tf2_msgs.msg import TFMessage
from time import sleep, time
from rosgraph_msgs.msg import Clock
from termios import tcflush, TCIFLUSH
from video_logging.msg import FilterSwitch
from JackalSSH import JackalSSH

class Timer:
    start = 0
    diagnoseTime = 0
    ConfidencePercentageResponseTime = 0
    ConfidenceExplanationTime = 0
    total = 0

class ProgressClock():
    def __init__(self, rosbag_name):
        bag = rosbag.Bag(rosbag_name, 'r')
        self.start_time = bag.get_start_time()
        self.end_time = bag.get_end_time()
        self.duration = self.end_time - self.start_time
        self.clock_sub = rp.Subscriber("/clock", Clock, self.callback)
        self.progress_pub = rp.Publisher("/progress", Float32, queue_size=10)
        bag.close()

    def callback(self, msg):
        progress = ((msg.clock.secs + msg.clock.nsecs / 1e9) - self.start_time) / self.duration
        self.progress_pub.publish(Float32(progress))
    
    def finishPlayback(self):
        self.progress_pub.unregister()
        self.clock_sub.unregister()
        

        
class Run_Condition():
    def __init__(self):
        self.rate = rp.Rate(10.0) #10Hz
        self.clear_scenario_pub = rp.Publisher('/clear_scenario', Empty, queue_size=10)
        self.tf_pub = rp.Publisher("/tf_path", TFMessage, queue_size=10)
        self.filter_pub = rp.Publisher("/filters", FilterSwitch, queue_size=10)
        self.pause = 1

    def on_press(self, key):  # The function that's called when a key is pressed
        if self.listenToKeypress == 1:
            self.listenToKeypress = 0
            if str(key) == "Key.enter":  # If enter was pressed, write a new line
                self.pause = 0
                self.listener.stop()

    def on_release(self, key):  # The function that's called when a key is released
        return

    def publish_infologs(self, rosbag_name):
        f = open(rospkg.RosPack().get_path('video_logging') + '/src/scripts/AR22_Scripts/infologs.json')
        data = json.load(f)
        messages = None
        for bag in data:
            if(rosbag_name.endswith(bag["bag_name"])):
                f.close()
                messages = bag["infologs"]
                break
        if(messages == None):
            print("[ERROR] Infologs for this bag are undefined in infologs.json file!")
            f.close()
            sys.exit(0)
        for i in range(len(messages)):
            msg = "[ Timestamp: {:.2f} secs ]\n".format(messages[i]["time"]) + messages[i]["text"]
            self.infologs_pub.publish(String(msg))
        
    def publish_tfs(self, rosbag_name):

        tf_path = []
        for topic, msg, t in rosbag.Bag(rosbag_name).read_messages(topics=["/tf"]):
            for tf in msg.transforms:
                if (tf.child_frame_id == "base_link" and tf.header.frame_id == "odom") or (tf.child_frame_id == "odom" and tf.header.frame_id == "map"):
                    tf_path.append(tf)
        self.tf_pub.publish(TFMessage(tf_path))

    
    def signal_handler(self, sig, frame):
        self.clear_scenario_pub.publish()
        
        print("\n\nshutting down from rosbag_player_experimental\n\n")
        sys.exit(0)

    def run_condition(self,rosbag_name,error,condition):
        # Signal Handler to kill subprocesses
        signal.signal(signal.SIGINT, self.signal_handler)

        # self.set_device_connections(condition)
        splitCondition = condition.split(" + ")

        data_to_write = []
        if splitCondition[1] == "live":
            #Live
            self.start_livestream(error, rosbag_name)
            data_to_write = self.record_data(error,condition)
            self.stop_livestream()
        if splitCondition[1] == "replay":
            #Replay
            rosbag_player, clock = self.play_rosbag(rosbag_name)
            data_to_write = self.record_data(error,condition)
            self.stop_rosbag(rosbag_player,clock)
        
        # Re-block all live filters data
        filters = FilterSwitch(
            velodyne_blocked=True,
            camera_blocked=True
        )
        self.filter_pub.publish(filters)
        JackalSSH().ros_pub_filterswitch(filters).kill()

        return data_to_write



    def start_livestream(self,error,rosbag_name):

        # Set filters
        print("Setting filters...")
        filters = FilterSwitch(
            velodyne_flicker = error in ["Velodyne LIDAR Failure"],
            velodyne_blocked = error in [],
            camera_flicker   = error in [],
            camera_blocked   = error in ["Camera Sensor Failure"]
        )
        self.filter_pub.publish(filters)
        j1 = JackalSSH().ros_pub_filterswitch(filters)
        
        print("Sending infologs...")
        j2 = JackalSSH().send_cmd('python live_infologs_publisher.py ' + rosbag_name)

        # Localise Jackal to origin
        wait_seconds = 5
        print("Localising Jackal to map origin...")
        j3 = JackalSSH().send_cmd("roslaunch jackal_navigation amcl_demo.launch map_file:=/home/administrator/G10Map.yaml scan_topic:=/scan")
        print("Allowing {} seconds...".format(wait_seconds))
        rp.sleep(wait_seconds)
        
        # Kill ssh's
        j1.kill(0)
        j2.kill(0)
        j3.kill(0)
        

    def stop_livestream(self):
        print("placeholder")

    def play_rosbag(self,rosbag_name):

        rospkg.RosPack().get_path('video_logging') + '/bag_files'
        
        rosbag_name = rospkg.RosPack().get_path('video_logging') + '/bag_files/' + rosbag_name

        # Disable all live-condition filters
        self.filter_pub.publish(FilterSwitch(
            velodyne_blocked=False,
            velodyne_flicker=False,
            camera_flicker=False,
            camera_blocked=False
        ))

        # Publish all TFs for path visualisation
        self.publish_tfs(rosbag_name)

        # Start Progress Clock
        clock = ProgressClock(rosbag_name)

        # Start rosbag
        rosbag_player = pyrosbag.BagPlayer(rosbag_name)
        rosbag_player.play(loop=True, publish_clock=True, quiet=True)
        return rosbag_player, clock

    def stop_rosbag(self,rosbag_player,clock):
        #Shut down rosbag
        rosbag_player.process.send_signal(signal.SIGINT)
        rp.sleep(1.) # to allow process to end
        rosbag_player.stop()
        rp.sleep(1.) # to allow process to end
        clock.finishPlayback()
        self.clear_scenario_pub.publish()


    def record_data(self,error,condition):
        #Start timer
        if error == "Training Scenario":
            rp.sleep(1.)
            splitCondition = condition.split(" + ")
            if splitCondition[0] == "AR":
                device = "Augmented Reality Headset"
            else:
                device = "Tablet"
            input("\nProvide participant with instructions on how to use the " + device + " and then press enter     ")
            data_to_write = ["training", "training", "training"]
        else:
            Timer.start = time()
            rp.sleep(1.)
            
            wait = input("\nPress enter when participant provides their diagnosis")
            
            #Wait for participant to respond, then the operator presses enter
            current_time = time()
            Timer.diagnoseTime = current_time - Timer.start

            print("Recorded Diagnosis time")

            #Did participant get the correct error/s?
            rp.sleep(0.5)
            correct_error = input("Did participant get the correct error/s? Y/N:  ")
            cont = 0
            if correct_error == "Y" or correct_error == "N":
                cont = 1
            while cont == 0:
                correct_error = input("Error. Invalid Input. Please enter Y or N:  ")
                if correct_error == "Y" or correct_error == "N":
                    cont = 1
            if correct_error == "N":
                errors_guessed = input("Which errors did the participant guess?  ")
            else:
                errors_guessed = error

            #Wait for participant to respond to second question
            #response = input("Press enter when participant has told you their confidence ")
            #current_time = time()
            #Timer.ConfidencePercentageResponseTime = current_time - Timer.start
            
            #Wait for participant to respond to third question
            #response = input("Press enter when participant has told you their reasoning for confidence ")
            #current_time = time()
            #Timer.ConfidenceExplanationTime = current_time - Timer.start

            #Wait for participant to respond to follow up questions
            response = input("\nPress enter when participant has responded to all follow up questions:  ")

            #Return data
            data_to_write = [correct_error, errors_guessed, Timer.diagnoseTime]
        return data_to_write

    
     

    