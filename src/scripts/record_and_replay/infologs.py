import rospy as rp
from std_msgs.msg import String

def infologs():
    pub = rp.Publisher('/infologs', String, queue_size=10)
    rp.sleep(0.5)
    pub.publish(String("1. Heading to Point B..."))
    rp.sleep(14.5)
    pub.publish(String("2. done"))
    rp.sleep(1)
    pub.publish(String("3. Exiting arena..."))
    rp.sleep(9)
    pub.publish(String("4. ERROR [Navigation] - No suitable path to destination"))
    rp.spin()

if __name__ == "__main__":
    rp.init_node("infologger", anonymous=True)
    infologs()