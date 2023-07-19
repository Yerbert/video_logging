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
from geometry_msgs.msg import Transform, Vector3, Quaternion
from time import sleep, time
from rosgraph_msgs.msg import Clock
from termios import tcflush, TCIFLUSH
from video_logging.msg import FilterSwitch
from video_logging.msg import ClearScenario
from JackalSSH import JackalSSH

class Timer:
    start = 0
    diagnoseTime = 0
    followupQn1Time = 0
    followupQn2Time = 0

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
        self.clear_scenario_pub = rp.Publisher('/clear_scenario', ClearScenario, queue_size=10)
        self.tf_pub = rp.Publisher("/tf_path", TFMessage, queue_size=10)
        self.filter_pub = rp.Publisher("/filters", FilterSwitch, queue_size=10)
        self.fake_obj_pub = rp.Publisher("/fake_object", Bool, queue_size=10)
        self.camera_smudge_pub = rp.Publisher("/camera_smudge", Bool, queue_size=10)
        self.deloc_pub = rp.Publisher("/delocalisation", Transform, queue_size=10)
        self.deloc_tf = Transform(translation=Vector3(0.5, 0.5, 0), rotation=Quaternion(0, 0, 0, 1))
        self.pause = 1
        self.map_name = "G10Map2xCropped"

    def on_press(self, key):  # The function that's called when a key is pressed
        if self.listenToKeypress == 1:
            self.listenToKeypress = 0
            if str(key) == "Key.enter":  # If enter was pressed, write a new line
                self.pause = 0
                self.listener.stop()

    def on_release(self, key):  # The function that's called when a key is released
        return
        
    def publish_tfs(self, rosbag_name):

        tf_path = []
        for topic, msg, t in rosbag.Bag(rosbag_name).read_messages(topics=["/tf"]):
            for tf in msg.transforms:
                if (tf.child_frame_id == "base_link" and tf.header.frame_id == "odom") or (tf.child_frame_id == "odom" and tf.header.frame_id == "map"):
                    tf_path.append(tf)
        self.tf_pub.publish(TFMessage(tf_path))
    
    def publish_delocalisation(self, error):
        if "localisation" in error.lower():
            self.deloc_pub.publish(self.deloc_tf)
        else:
            self.deloc_pub.publish(Transform())
    
    def publish_camera_smudge(self, error):
        if "smudge" in error.lower():
            self.camera_smudge_pub.publish(Bool(True))
        else:
            self.camera_smudge_pub.publish(Bool(False))

    # def publish_map(self, error):
    #     print("Starting map server...")
    #     # map_yaml = rospkg.RosPack().get_path('video_logging') + "/maps/" + (self.map_name+"Delocalised.yaml" if "Localisation Error" in error else self.map_name+".yaml")
    #     map_yaml = rospkg.RosPack().get_path('video_logging') + "/maps/" + self.map_name+".yaml"
    #     map_proc = subprocess.Popen(
    #         "rosrun map_server map_server " + map_yaml,
    #         shell=True,
    #         stdin=subprocess.PIPE,
    #         stdout=subprocess.PIPE,
    #         preexec_fn=os.setsid
    #     )
    #     rp.sleep(2)
    #     print("Killing map server...")
    #     os.killpg(os.getpgid(map_proc.pid), signal.SIGTERM)

    
    def signal_handler(self, sig, frame):
        # Clear
        self.clear_scenario_pub.publish(ClearScenario(True, True, True, True))
        
        print("\n\nshutting down from rosbag_player_experimental\n\n")
        sys.exit(0)

    def run_condition(self,rosbag_name,error,condition):
        # Signal Handler to kill subprocesses
        signal.signal(signal.SIGINT, self.signal_handler)

        splitCondition = condition.split(" + ")

        data_to_write = []
        if splitCondition[1] == "live":
            #Live
            self.start_livestream(error, rosbag_name)
            data_to_write = self.record_data(error, condition)
            self.stop_livestream()
        if splitCondition[1] == "replay":
            #Replay
            rosbag_player, clock = self.play_rosbag(error, rosbag_name)
            data_to_write = self.record_data(error, condition)
            self.stop_rosbag(rosbag_player, clock)


        # End of scenario...

        ssh_lst = []
        print("  Blocking data stream (enabling filters)...")
        # Block all data
        filters = FilterSwitch(
            velodyne_blocked=True,
            camera_blocked=True
        )
        self.filter_pub.publish(filters)
        j = JackalSSH().ros_pub_msg("/filters", "video_logging/FilterSwitch", filters)
        ssh_lst.append(j)

        # Clear fake object
        print("  Clearing fake object...")
        self.fake_obj_pub.publish(Bool(False))
        j = JackalSSH().ros_pub("/fake_object", "std_msgs/Bool", "false")
        ssh_lst.append(j)

        # Clear camera smudge
        print("  Clearing camera smudge...")
        self.camera_smudge_pub.publish(Bool(False))
        j = JackalSSH().ros_pub("/camera_smudge", "std_msgs/Bool", "false")
        ssh_lst.append(j)

        # Clear scenario
        print("  Clearing scenario...")
        clear_all = ClearScenario(True, True, True, True)
        self.clear_scenario_pub.publish(clear_all)
        j = JackalSSH().ros_pub_msg("/clear_scenario", "video_logging/ClearScenario", clear_all)
        ssh_lst.append(j)

        wait_seconds = 4
        print("  Allowing {} seconds...".format(wait_seconds))
        rp.sleep(wait_seconds)
        
        # Kill ssh's
        for ssh in ssh_lst:
            ssh.kill(0)

        return data_to_write



    def start_livestream(self,error,rosbag_name):

        # Set filters
        # Uses multiple JackalSSH's so that they are all done in parallel
        print("\n  Livestream started.")
        print("  Setting filters...")
        filters = FilterSwitch(
            velodyne_flicker = error in ["LaserScanner Flicker"],
            velodyne_blocked = error in [],
            camera_flicker   = error in [],
            camera_blocked   = error in ["Camera Issue"]
        )
        self.filter_pub.publish(filters)

        ssh_lst = []
        # j1 = JackalSSH().ros_pub_filterswitch(filters)
        j = JackalSSH().ros_pub_msg("/filters", "video_logging/FilterSwitch", filters)
        ssh_lst.append(j)
        
        print("  Sending infologs...")
        j = JackalSSH().send_cmd('python /home/administrator/catkin_ws/src/video_logging/src/live_infologs_publisher.py ' + rosbag_name)
        ssh_lst.append(j)

        # Localise Jackal to origin
        print("  Localising Jackal to map origin...")
        j = JackalSSH().send_cmd("roslaunch jackal_navigation my_amcl_demo.launch map_file:=/home/administrator/{}.yaml scan_topic:=/scan update_min_a:=0.1 init_a:=-1.5708".format(self.map_name))
        ssh_lst.append(j)
        rp.sleep(0.1)

        # Turn on effect non-existent object in point cloud
        fake_obj = (error == "LaserScanner Issue (Fake Obstruction)")
        fake_obj_str = str(fake_obj).lower()    # bool to string
        print("  Setting fake object effect in point cloud to {}".format(fake_obj_str))
        j = JackalSSH().ros_pub("/fake_object", "std_msgs/Bool", fake_obj_str)
        ssh_lst.append(j)

        # Turn on camera smudge
        camera_smudge = ("smudge" in error.lower())
        camera_smudge_str = str(camera_smudge).lower()    # bool to string
        print("  Setting camera smudge effect to {}".format(camera_smudge_str))
        j = JackalSSH().ros_pub("/camera_smudge", "std_msgs/Bool", camera_smudge_str)
        ssh_lst.append(j)

        # # Send single point cloud frame for lidar/localisation error
        # if error == "Velodyne LIDAR Failure and Localisation Error":
        #     j = JackalSSH().send_cmd('rosbag play /home/administrator/catkin_ws/src/video_logging/src/SingleVelodyneLive.bag')
        #     ssh_lst.append(j)
        
        # Send single camera frame for camera failure
        if error == "Camera Issue":
            j = JackalSSH().send_cmd('rosbag play /home/administrator/catkin_ws/src/video_logging/src/SingleCameraLive.bag')
            ssh_lst.append(j)
        
        # Publish all TFs for path visualisation
        print("  Sending path from rosbag "+rosbag_name)
        j = JackalSSH().send_cmd('python /home/administrator/catkin_ws/src/video_logging/src/path_publisher.py ' + rosbag_name)
        ssh_lst.append(j)

        # # Map server
        # print("  Sending map...")
        # # map_yaml = rospkg.RosPack().get_path('video_logging') + "/maps/" + (self.map_name+"Delocalised.yaml" if "Localisation Error" in error else self.map_name+".yaml")
        # map_yaml = rospkg.RosPack().get_path('video_logging') + "/maps/" + self.map_name+".yaml"
        # j = JackalSSH().send_cmd("rosrun map_server map_server " + map_yaml)
        # ssh_lst.append(j)

        # Delocalisation
        if "localisation" in error.lower():
            j = JackalSSH().ros_pub_msg("/delocalisation", "geometry_msgs/Transform", self.deloc_tf)
            ssh_lst.append(j)
        else:
            j = JackalSSH().ros_pub_msg("/delocalisation", "geometry_msgs/Transform", Transform())
            ssh_lst.append(j)
        
        #Start timer
        Timer.start = time()
        rp.sleep(1.)
        
        wait_seconds = 4
        print("  Allowing {} seconds...".format(wait_seconds))
        rp.sleep(wait_seconds)
        
        # Kill ssh's
        for ssh in ssh_lst:
            ssh.kill(0)
        

    def stop_livestream(self):
        print("  Livestream stopped")

    def play_rosbag(self, error, rosbag_name):

        rospkg.RosPack().get_path('video_logging') + '/bag_files'
        
        rosbag_name = rospkg.RosPack().get_path('video_logging') + '/bag_files/' + rosbag_name

        # Disable all filters for replay (laptop only)
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

        # Set delocalisation
        self.publish_delocalisation(error)

        # Set camera smudge
        self.publish_camera_smudge(error)

        #Start timer
        Timer.start = time()
        rp.sleep(1.)

        return rosbag_player, clock

    def stop_rosbag(self,rosbag_player,clock):
        #Shut down rosbag
        rosbag_player.process.send_signal(signal.SIGINT)
        rp.sleep(1.) # to allow process to end
        rosbag_player.stop()
        rp.sleep(1.) # to allow process to end
        clock.finishPlayback()

    def record_data(self,error,condition):
        
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

            # Timer is started in previous function
            
            wait = input("\nPress enter when participant provides their diagnosis")
            
            #Wait for participant to respond, then the operator presses enter
            current_time = time()
            Timer.diagnoseTime = current_time - Timer.start

            print("Recorded Diagnosis time")

            #Did participant get the correct error/s?
            rp.sleep(0.5)
            correct_error = input("Did participant guess the correct error/s? (Y/N):  ")
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

            # Record confidence
            rp.sleep(0.5)
            confidence = input("How confident? (give a percentage):  ")
            cont = 0
            while cont == 0:
                try:
                    co = int(confidence)
                    assert 0 <= co <= 100
                except:
                    confidence = input("Error. Invalid Input. Please enter a percentage between 0 and 100: ")
                else:
                    cont = 1
            

            # Wait for participant to respond to follow up questions
            
            # First

            Timer.start = time()
            
            response = input("\nPress enter when participant has answered FIRST follow up question:  ")
            
            current_time = time()
            Timer.followupQn1Time = current_time - Timer.start

            # Second

            Timer.start = time()
            
            response = input("\nPress enter when participant has answered SECOND follow up question:  ")
            
            current_time = time()
            Timer.followupQn2Time = current_time - Timer.start

            

            
            # Question answered correctly

            correct_qn = "n/a"
            if error in ["Dropped Payload", "Obstruction in Robot's Path"]:
                rp.sleep(0.5)
                correct_qn = input("\nDid the participant answer the follow-up question correctly? (Y/N):  ")
                cont = 0
                if correct_qn == "Y" or correct_qn == "N":
                    cont = 1
                while cont == 0:
                    correct_qn = input("Error. Invalid Input. Please enter Y or N:  ")
                    if correct_qn == "Y" or correct_qn == "N":
                        cont = 1
            
            #Wait for participant to respond to second question
            #response = input("Press enter when participant has told you their confidence ")
            #current_time = time()
            #Timer.ConfidencePercentageResponseTime = current_time - Timer.start
            
            #Wait for participant to respond to third question
            #response = input("Press enter when participant has told you their reasoning for confidence ")
            #current_time = time()
            #Timer.ConfidenceExplanationTime = current_time - Timer.start

            

            #Return data
            data_to_write = [correct_error, errors_guessed, round(Timer.diagnoseTime,3), confidence, correct_qn, round(Timer.followupQn1Time,3), round(Timer.followupQn2Time,3)]
        return data_to_write

    
     

    