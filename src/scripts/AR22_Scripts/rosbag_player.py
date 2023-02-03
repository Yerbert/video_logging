import rospy as rp
import rosbag
import rostopic
import rosnode
import pyrosbag
import subprocess
import os
import sys
import signal
from std_msgs.msg import Bool
from tf2_msgs.msg import TFMessage
from time import sleep

#Global
tcp = False

class PauseSubscriber():
    def __init__(self, rosbag_process):
        self.sub = rp.Subscriber("/pause", Bool, self.callback)
        self.rosbag = rosbag_process
        self.paused = False
    
    def callback(self, msg):
        if msg.data == self.paused:
            return

        if self.paused:
            self.rosbag.resume()
        else:
            self.rosbag.pause()
        self.paused = not self.paused

def publish_tfs(rosbag_name):
    tf_pub = rp.Publisher("/tf_path", TFMessage, queue_size=10)

    tf_path = []
    for topic, msg, t in rosbag.Bag(rosbag_name).read_messages(topics=["/tf"]):
        for tf in msg.transforms:
            if (tf.child_frame_id == "base_link" and tf.header.frame_id == "odom") or (tf.child_frame_id == "odom" and tf.header.frame_id == "map"):
                tf_path.append(tf)
    tf_pub.publish(TFMessage(tf_path))

def signal_handler(sig, frame):
    if (tcp):
        os.killpg(os.getpgid(p.pid), signal.SIGTERM) 
    sys.exit(0)

if __name__ == "__main__":


    # Get rosbag name and tcp option
    rosbag_name = sys.argv[1]
    if len(sys.argv) == 3:
        tcp = sys.argv[2]

    # Signal Handler to kill subprocesses
    signal.signal(signal.SIGINT, signal_handler)

    # Start subprocess to play start tcp
    if (tcp):
        command = "roslaunch ros_tcp_endpoint endpoint.launch"
        p = subprocess.Popen(command, stdin=subprocess.PIPE, shell=True, cwd=os.getcwd(), executable='/bin/bash', preexec_fn=os.setsid)
        sleep(2.0)

    # Check if tcp endpoint is initialized
    try:
        rostopic.get_topic_class('/rosout')
        rosnode_list = rosnode.get_node_names()
        if '/unity_endpoint' not in rosnode_list:
            print("[ERROR] Please start a tcp endpoint before playing rosbag.")
            sys.exit(0)
    except:
        print("[ERROR] Roscore and tcp endpoint must be active before playing rosbag!")
        sys.exit(0)
    
    # Initialize node
    rp.init_node("rosbag_replay")

    # Get rosbag name
    rosbag_name = sys.argv[1]

    # Publish all TFs for path visualisation
    publish_tfs(rosbag_name)

    # Start rosbag
    rosbag_player = pyrosbag.BagPlayer(rosbag_name)
    pause = PauseSubscriber(rosbag_player)
    rosbag_player.play(loop=True)
    rp.spin()
    