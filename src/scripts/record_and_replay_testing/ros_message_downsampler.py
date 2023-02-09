import rospy as rp
from sensor_msgs.msg import PointCloud2, CompressedImage
from sensor_msgs import point_cloud2 as pc2
from PIL import Image
import io

# class Counter:
#     def __init__(self, skip_every_nth=None, accept_every_nth=None):
#         assert not (skip_every_nth is not None and accept_every_nth is not None), "Must only provide value for one parameter"
#         assert (skip_every_nth != 0 and accept_every_nth != 0), "Must give nonzero value"

#         self.value = 0
#         self.skip_every_nth = skip_every_nth
#         self.accept_every_nth = accept_every_nth
    
#     def tick(self, num=1):
#         # Skip every nth
#         if self.skip_every_nth is not None:
#             self.value = (self.value + num) % self.skip_every_nth
#             return self.value != 0
        
#         if self.accept_every_nth is not None:
#             self.value = (self.value + num) % self.accept_every_nth
#             return self.value == 0



class PointCloudProcessor:
    def __init__(self):
        self.sub = rp.Subscriber('/velodyne_points/processed', PointCloud2, self.callback)
        self.pub = rp.Publisher('/velodyne_points/processed/ds', PointCloud2, queue_size=10)
    
    def callback(self, msg):
        ds = 2 # downsample factor
        new_msg = pc2.create_cloud_xyz32(msg.header, [point for point in pc2.read_points(msg) if self.is_acceptable_point(point)][ds-1::ds])
        # new_msg = pc2.create_cloud_xyz32(msg.header, [p for p in pc2.read_points_list(msg, ['x','y','z']) if max(p) < max_dist and counter.tick()])
        self.pub.publish(new_msg)
    
    def is_acceptable_point(self, point):
        """
        Culls points further than max_dist and lower than floor_z
        param p: (x,y,z)
        """
        accept_all_points = False
        max_dist = 4
        floor_z = -0.38

        if (accept_all_points):
            return True

        if max([abs(p) for p in point]) > max_dist:
            return False
        
        if point[2] <= floor_z:
            return False
        
        return True


class VideoStreamProcessor:
    def __init__(self):
        self.sub = rp.Subscriber('/camera/color/image_raw/compressed/processed', CompressedImage, self.callback)
        self.pub = rp.Publisher('/camera/color/image_raw/compressed/processed/ds', CompressedImage, queue_size=10)
    
    def callback(self, msg):
        out_resolution = (480,360)
        img = Image.open(io.BytesIO(msg.data))
        img_resized = img.resize(out_resolution, Image.ANTIALIAS)
        img_data = io.BytesIO()
        img_resized.save(img_data, format='JPEG')
        img_data_bytearray = img_data.getvalue()
        msg.data = img_data_bytearray
        img.close()
        self.pub.publish(msg)
        



if __name__ == "__main__":
    rp.init_node("ros_message_downsampler", anonymous=True)
    p = PointCloudProcessor()
    v = VideoStreamProcessor()
    rp.spin()