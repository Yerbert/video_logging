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
from std_msgs.msg import Bool, Float32
from tf2_msgs.msg import TFMessage
from time import sleep, time
from rosgraph_msgs.msg import Clock
from termios import tcflush, TCIFLUSH

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

    def callback(self, msg):
        progress = ((msg.clock.secs + msg.clock.nsecs / 1e9) - self.start_time) / self.duration
        self.progress_pub.publish(Float32(progress))
    
    def finishPlayback(self):
        self.progress_pub.unregister()
        self.clock_sub.unregister()
        
        
        
class Play_Rosbag():
    def __init__(self):
        self.rate = rp.Rate(10.0) #10Hz
        self.infologs_pub = rp.Publisher('/infologs/end', Bool, queue_size=10)
        self.pause = 1
        self.listenToKeypress = 0
        self.listener = Listener(on_press=self.on_press, on_release=self.on_release)
        self.listener.start()  # Create an instance of Listener

    def on_press(self, key):  # The function that's called when a key is pressed
        if self.listenToKeypress == 1:
            self.listenToKeypress = 0
            if str(key) == "Key.enter":  # If enter was pressed, write a new line
                self.pause = 0
                self.listener.stop()

    def on_release(self, key):  # The function that's called when a key is released
        return

    def publish_tfs(self, rosbag_name):
        tf_pub = rp.Publisher("/tf_path", TFMessage, queue_size=10)

        tf_path = []
        for topic, msg, t in rosbag.Bag(rosbag_name).read_messages(topics=["/tf"]):
            for tf in msg.transforms:
                if (tf.child_frame_id == "base_link" and tf.header.frame_id == "odom") or (tf.child_frame_id == "odom" and tf.header.frame_id == "map"):
                    tf_path.append(tf)
        tf_pub.publish(TFMessage(tf_path))

    
    def signal_handler(self, sig, frame):
        self.infologs_pub.publish(1)
        self.listener.stop()
        print("\nshutting down")
        sys.exit(0)

    def play_rosbag(self,rosbag_name,error):
        rospkg.RosPack().get_path('video_logging') + '/bag_files'
        # Signal Handler to kill subprocesses
        signal.signal(signal.SIGINT, self.signal_handler)
        
        rosbag_name = rospkg.RosPack().get_path('video_logging') + '/bag_files/' + rosbag_name

        # Publish all TFs for path visualisation
        self.publish_tfs(rosbag_name)

        # Start Progress Clock
        clock = ProgressClock(rosbag_name)

        # Start rosbag
        rosbag_player = pyrosbag.BagPlayer(rosbag_name)
        rosbag_player.play(loop=True, publish_clock=True, quiet=True)

        #Start timer
        Timer.start = time()
        rp.sleep(1.)
        self.listenToKeypress = 1
        while self.pause == 1:
            rp.sleep(0.1)
        #rp.spin()
        #Wait for participant to respond, then the operator presses enter
        current_time = time()
        Timer.diagnoseTime = current_time - Timer.start
        rp.sleep(0.2)
        print("\n\nRecorded Diagnosis time")
        tcflush(sys.stdin, TCIFLUSH) # flush input stream
        #Did participant get the correct error/s?
        rp.sleep(0.5)
        correct_error = input("\n\nDid participant get the correct error/s? Y/N:  ")
        cont = 0
        if correct_error == "Y" or correct_error == "N":
            cont = 1
        while cont == 0:
            correct_error = input("\nPlease enter Y or N:  ")
            if correct_error == "Y" or correct_error == "N":
                cont = 1
        if correct_error == "N":
            errors_guessed = input("\n\nWhich errors did the participant guess?    ")
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
        tcflush(sys.stdin, TCIFLUSH)
        response = input("\n\nPress enter when participant has responded to all follow up questions:  ")
        
        #Shut down rosbag
        rosbag_player.stop()
        clock.finishPlayback()

        #Return data
        data_to_write = [correct_error, errors_guessed, Timer.diagnoseTime]
        return data_to_write

    
     

    