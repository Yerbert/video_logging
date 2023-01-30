import rospy as rp
import rosbag
import pyrosbag
import sys
from std_msgs.msg import Bool
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import Vector3, PoseArray, Pose
import time

class PauseSubscriber():
    def __init__(self, rosbag_process):
        self.sub = rp.Subscriber("/pause", Bool, self.callback)
        self.rosbag = rosbag_process
        self.paused = False
        self.last_msg_time = time.time()
        self.msg_filter_delay_seconds = 0.5
    
    def callback(self, msg):

        # "Debounce" filter
        if time.time() - self.last_msg_time < self.msg_filter_delay_seconds:
            return

        # Ignore repeated messages
        if msg.data == self.paused:
            return

        if self.paused:
            self.rosbag.resume()
        else:
            self.rosbag.pause()
        self.paused = not self.paused

# def publish_tfs(rosbag_name):
#     tf_pub = rp.Publisher('/path/full', Vector3, queue_size=10)
#     tf_pub.publish(Vector3(0,0,0))
#     rp.sleep(2)

def publish_tfs(rosbag_name):
    tf_pub = rp.Publisher("/tf/path", TFMessage, queue_size=10)

    tf_path = []
    for topic, msg, t in rosbag.Bag(rosbag_name).read_messages(topics=["/tf"]):
        for tf in msg.transforms:
            if tf.child_frame_id == "base_link" and tf.header.frame_id == "odom":
                tf_path.append(tf)
                break
    print(len(tf_path))
    tf_pub.publish(TFMessage(tf_path))

if __name__ == "__main__":

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
    