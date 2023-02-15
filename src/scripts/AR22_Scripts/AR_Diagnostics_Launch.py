#!/usr/bin/env python3

from fileinput import filename
import rospy
from std_msgs.msg import String
from video_logging.msg import FilterSwitch
import numpy as np
import time
from datetime import datetime
import random
import os
import subprocess
import signal
import pickle
import argparse
import rospkg
import copy
import sys
import signal
try:
    from builtins import input
except ImportError:
    from __builtin__ import input
#from Constants import Constants
from geometry_msgs.msg import Point, Vector3, WrenchStamped, PoseStamped
from openpyxl import load_workbook, Workbook

import rosbag_player_experimental
from JackalSSH import JackalSSH

# For printing formatting
class Color:
   PURPLE = '\033[95m'
   MAGENTA = '\033[35m'
   CYAN = '\033[96m'
   DARKCYAN = '\033[36m'
   BLUE = '\033[94m'
   GREEN = '\033[92m'
   YELLOW = '\033[93m'
   RED = '\033[91m'
   BOLD = '\033[1m'
   UNDERLINE = '\033[4m'
   END = '\033[0m'

class Errors:
    name = None
    types = {
                        "A": "Flat tyre",
                        "B": "Motor Failure",
                        "C": "Camera Sensor Failure",
                        "D": "Velodyne LIDAR Failure",
                        "E": "Obstacle in the Way",
                        "F": "Robot Senses Non-existent Object",
                        "G": "Dropped Payload",
                        "H": "Velodyne LIDAR Failure and Localisation Error",
                        "T": "Training Scenario"
    }
    rosbags = {
                        "A": "FlatTyre.bag",
                        "B": "MotorFailure.bag",
                        "C": "CameraFailure.bag",
                        "D": "VelodyneError.bag",
                        "E": "HumanObstruction.bag",
                        "F": "HumanObstruction.bag",
                        "G": "DroppedPayload.bag",
                        "H": "LocalisationError.bag",
                        "T": "TrainingRecording.bag"
                        }
    jackal_locations = {
                        "A"     : "Point A (near desk)",
                        "B"     : "Point C (middle)",
                        "C"     : "Point A (near desk)",
                        "D"     : "Point A (near desk)",
                        "E"     : "Point A (near desk), slightly towards gap in barriers",
                        "F"     : "Point A (near desk), slightly towards gap in barriers",
                        "G"     : "Point A (near desk)",
                        "H"     : "Point A (near desk)",
                        "T"     : "Point A (near desk)",
                        "live"  : "Point B (map origin)",
    }

class Conditions:
    name = None
    conditions = {
                        "1": "AR + replay",
                        "2": "AR + live",
                        "3": "Tablet + replay",
                        "4": "Tablet + live"
                        }

class MyWorkbook:
    def __init__(self):
        self.conditionCol = 3
        self.errorCol = 4
        self.correctErrorCol = 5
        self.guessedErrorCol = 6
        self.diagnoseTimeCol = 7
        #ConfidenceTimeCol = 6
        self.dateCol = 1
        self.partIdCol = 2
    
        self.file_path = rospkg.RosPack().get_path('video_logging') + '/Sheets'
        self.excel_filename = "data.xlsx"
        self.fullpath = os.path.join(self.file_path, self.excel_filename)
        self.wb = load_workbook(self.fullpath)
        self.ws = self.wb.active
    
    def write_next_row(self, participant_no, condition, error, correct_error, guessed_error, diagnosis_time):
        row = self.ws.max_row + 1
        self.ws.cell(row=row, column=self.dateCol).value = datetime.now()
        self.ws.cell(row=row, column=self.partIdCol).value = participant_no
        self.ws.cell(row=row, column=self.conditionCol).value = condition
        self.ws.cell(row=row, column=self.errorCol).value = error
        self.ws.cell(row=row, column=self.correctErrorCol).value = correct_error
        self.ws.cell(row=row, column=self.guessedErrorCol).value = guessed_error
        self.ws.cell(row=row, column=self.diagnoseTimeCol).value = diagnosis_time
        self.wb.save(self.fullpath)


class AR_error_diagnostics:
    def __init__(self, participant_no):
        self.participant_no = int(participant_no)
        self.conditions = ["","","","","","","","","",""]
        self.errors = ["","","","","","","","","",""]
        self.condition_pub = rospy.Publisher("/condition", String, queue_size=10)

    def signal_handler(self, sig, frame):
        print("\n\nshutting down from AR_error_diagnostics\n\n")
        rospy.sleep(.5)
        sys.exit(0)

    def all_loop(self):

        # Block live data stream initially
        print("  Blocking all data streams initially...")
        filters = FilterSwitch(
            velodyne_blocked=True,
            camera_blocked=True
        )
        JackalSSH().ros_pub_filterswitch(filters).kill()

        #Find and open file with conditions and errors used by each participant
        signal.signal(signal.SIGINT, self.signal_handler)
        pol_file_path = rospkg.RosPack().get_path('video_logging') + '/Sheets'
        pol_filename = "pol.xlsx"
        pol_wb = load_workbook(os.path.join(pol_file_path, pol_filename))
        pol_sheet = pol_wb.active
        pol_rows = 31
        pol_columns = pol_sheet.max_column
        participant_row = 0

        #Find row relating to current participant
        for i in range(2,pol_rows):
            number = int(pol_sheet.cell(row = i, column = 1).value)
            if number == self.participant_no:
                participant_row = i
        
        #Find order of conditions for current participant
        j = 2
        for i in range(2,6):
            condition = int(pol_sheet.cell(row = participant_row, column = i).value)
            self.conditions[j] = str(condition)
            j = j + 1
            self.conditions[j] = str(condition)
            j = j + 1


        #Find order of errors for current participant
        for i in range(2,10):
            col = i+6
            self.errors[i] = pol_sheet.cell(row = participant_row, column = col).value
        
        #Add static training scenarios to lists
        self.errors[0] = "T"
        self.errors[1] = "T"
        self.conditions[0] = "1"
        self.conditions[1] = "3"
        workbook = MyWorkbook()

        input("Run through first page of instructions if not already done. Press enter to continue once completed    ")

        #Go through each error
        for l, error in enumerate(self.errors):
            #Print error information here
            print("\nThe error will be " + Color.BOLD + Color.CYAN + Errors.types[self.errors[l]] + Color.END)
            #Print condition for operator to know which method is being used
            print("The condition required for this error is " + Color.BOLD + Color.RED + Conditions.conditions[self.conditions[l]] + Color.END)
            #Print location where Jackal needs to be
            location = Errors.jackal_locations["live"] if Conditions.conditions[self.conditions[l]].split(" + ")[1] == "live" else Errors.jackal_locations[self.errors[l]]
            print("The Jackal will be located at: " + Color.BOLD + Color.YELLOW + location + Color.END)
            cont = input("Do you want to perform this Error/condition? (Y/N)  ")
            check = 0
            if cont == "Y" or cont == "N":
                check = 1
            while check != 1:
                cont = input("Error. Invalid Input. Do you want to perform this Error/condition? (Y/N)  ")
                if cont == "Y" or cont == "N":
                    check = 1
            if cont == "N":
                print("skipping error " + Errors.types[self.errors[l]] + "\n\n\n")
                continue

            self.configure_device_connections(Conditions.conditions[self.conditions[l]], Errors.rosbags[self.errors[l]])

            wait = input("\nPrepare condition now and ensure devices are connected.\nPress any key to begin streaming data   ")
            print("\n")
            #Once user has confirmed ready, switch to other file to start condition
            
            
            play_condition = rosbag_player_experimental.Run_Condition()
            data_to_write = play_condition.run_condition(Errors.rosbags[self.errors[l]],Errors.types[self.errors[l]],Conditions.conditions[self.conditions[l]])
            
            if l == 1:
                input("Provide instructions on difference between live and replay and then press enter    ")
                print("\n\nTraining complete. About to move onto study.... \n\n")
            if l > 1:
                MyWorkbook().write_next_row(participant_no,Conditions.conditions[self.conditions[l]],Errors.types[self.errors[l]],data_to_write[0],data_to_write[1],data_to_write[2])
                print("Error " + str(l+1) + " completed\n\n\n")
        print("All error conditions complete, exiting...\n\n")
    

    def configure_device_connections(self, new_condition, rosbag_name):

        sleep_seconds = 6
        print("\n  Signalling devices to configure connections...")
        
        self.condition_pub.publish(String(new_condition))
        JackalSSH().ros_pub_condition(new_condition).kill()
        print("  Sleeping for {} more seconds to allow connections...".format(sleep_seconds))
        rospy.sleep(sleep_seconds) # to allow reconnections to occur

if __name__ == '__main__':
    rospy.init_node('error_diagnostics_user_study')
    participant_no = input("Enter Participant Number: ")
    
    AR_error_diagnostics_obj = AR_error_diagnostics(participant_no)
    AR_error_diagnostics_obj.all_loop() 
        # filename = 'participant_' + args.participant_ID + '.pkl'
        # pickle.dump(handover_obj.seq_list, open(filename, "wb"))
