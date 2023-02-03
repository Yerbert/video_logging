import rospy as rp
from sensor_msgs.msg import PointCloud2, CompressedImage
from sensor_msgs import point_cloud2 as pc2
from tf2_msgs.msg import TFMessage
from PIL import Image
import io


class PointCloudProcessor:
    def __init__(self):
        self.sub = rp.Subscriber('/velodyne_points', PointCloud2, self.callback)
        self.pub = rp.Publisher('/velodyne_points/processed', PointCloud2, queue_size=1)
        self.counter = 0
        self.accept_every_nth_msg = 2 # any lower than 3 and point cloud lags
        self.ds = 1     # point downsample factor
    
    def callback(self, msg):
        self.counter = (self.counter + 1) % self.accept_every_nth_msg
        if (self.counter!=0):
            return

        new_points = [point for point in pc2.read_points_list(msg, ['x','y','z']) if self.is_acceptable_point(point)][self.ds-1::self.ds]
        new_msg = pc2.create_cloud_xyz32(msg.header, new_points)
        self.pub.publish(new_msg)
    
    def is_acceptable_point(self, point):
        """
        Culls points further than 'max_dist' and lower than 'floor_y'
        param p: (x,y,z)
        """
        accept_all_points = False
        max_dist = 4
        floor_y = -0.38

        if (accept_all_points):
            return True

        # drop point if lies outside max_dist^3 cube
        if max([abs(p) for p in point]) > max_dist:
            return False
        
        # drop point if at/below floor level
        if point[2] <= floor_y:
            return False
        
        return True


class VideoStreamProcessor:
    def __init__(self):
        self.sub = rp.Subscriber('/camera/color/image_raw/compressed/', CompressedImage, self.callback)
        self.pub = rp.Publisher('/camera/color/image_raw/compressed/processed', CompressedImage, queue_size=1)
        self.counter = 0
        self.accept_every_nth_msg = 2
        self.old_msg_size = (640,480)
        self.new_image_size = (640, 480)
    
    def callback(self, msg):

        # Reduce frame rate
        self.counter = (self.counter + 1) % self.accept_every_nth_msg
        if (self.counter!=0):
            return
        
        # Don't resize if new res = old res
        if (self.new_image_size == self.old_msg_size):
            self.pub.publish(msg)
            return
        
        # Resize image
        img = Image.open(io.BytesIO(msg.data))
        img_resized = img.resize(self.new_image_size, Image.ANTIALIAS)
        img_data = io.BytesIO()
        img_resized.save(img_data, format='JPEG')
        msg.data = img_data.getvalue()
        img.close()
        self.pub.publish(msg)


class TFProcessor:
    def __init__(self):
        self.pub = rp.Publisher('/tf/processed', TFMessage, queue_size=1)
        self.sub = rp.Subscriber('/tf', TFMessage, self.callback)
        self.counter = 0
        self.accept_every_nth_msg = 2
    
    def callback(self, msg):
        self.counter = (self.counter + 1) % self.accept_every_nth_msg
        if self.counter==0:
            self.pub.publish(msg)



if __name__ == "__main__":
    rp.init_node("ros_message_processor", anonymous=True)
    processors = [ PointCloudProcessor(), VideoStreamProcessor() ]
    rp.spin()