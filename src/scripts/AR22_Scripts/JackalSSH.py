import subprocess
import signal
import rospy
import os
import json
from rospy_message_converter import json_message_converter

"""
Wrapper class for easy SSH'ing into Jackal robot.
Always call the .kill() method when you no longer need it.
"""

class JackalSSH:
    def __init__(self):
        self.process = subprocess.Popen(
            "sshpass -p clearpath ssh -tt administrator@160.69.69.100\n",
            shell=True,
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            preexec_fn=os.setsid
        )
    
    def send_cmd(self, cmd):
        self.process.stdin.write((cmd+"\n").encode())
        self.process.stdin.flush()
        return self

    def ros_pub(self, topic_name, msg_type, data_str):
        self.send_cmd("rostopic pub -1 {} {} {}".format(
            topic_name,
            msg_type,
            '"' + data_str + '"'
        ))
        return self
    
    def ros_pub_msg(self, topic_name, msg_type, msg):
        # Msg is a msg object
        # Also assumes no uppercase characters
        # msg_data_text = ('{' + str(msg).replace('\n',', ') + '}').lower()
        # msg_data = message_converter.convert_ros_message_to_dictionary(msg)
        msg_data = json_message_converter.convert_ros_message_to_json(msg).replace('"','')
        self.ros_pub(topic_name, msg_type, msg_data)
        return self

    def ros_pub_condition(self, condition):
        self.ros_pub("/condition", "std_msgs/String", condition)
        return self
    
    def kill(self, wait=4):
        # in most cases, publishing from cmd line "latches" 3 seconds
        print("  <killing SSH, waiting {} seconds...>".format(wait))
        rospy.sleep(wait)
        os.killpg(os.getpgid(self.process.pid), signal.SIGTERM)
        del self.process