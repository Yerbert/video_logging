import subprocess
import signal
import rospy
import os
import json
from video_logging.msg import FilterSwitch

class JackalSSH:
    def __init__(self):
        self.process = subprocess.Popen(
            "sshpass -p clearpath ssh -tt administrator@160.69.69.10\n",
            shell=True,
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            preexec_fn=os.setsid
        )
    
    def send_cmd(self, cmd):
        self.process.stdin.write((cmd+"\n").encode())
        self.process.stdin.flush()
        return self

    def ros_pub(self, topic_name, msg_type, data):
        self.send_cmd("rostopic pub -1 {} {} {}".format(
            topic_name,
            msg_type,
            json.dumps(data)
        ))
        return self
    
    def ros_pub_filterswitch(self, msg):
        # msg must be a FilterSwitch
        msg_data_text = ('{' + str(msg).replace('\n',', ') + '}').lower()
        self.ros_pub("/filters", "video_logging/FilterSwitch", msg_data_text)
        return self

    def ros_pub_condition(self, condition):
        self.ros_pub("/condition", "std_msgs/String", condition)
        return self
    
    def kill(self, wait=4):
        # in most cases, publishing from cmd line "latches" 3 seconds
        rospy.sleep(wait)
        os.killpg(os.getpgid(self.process.pid), signal.SIGTERM)
        del self.process