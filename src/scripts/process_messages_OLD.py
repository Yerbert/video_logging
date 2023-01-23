#!/usr/bin/env python

import rospy as rp
# import numpy as np
# import open3d as o3d
from sensor_msgs.msg import PointCloud2, CompressedImage
from sensor_msgs import point_cloud2 as pc2
from tf2_msgs.msg import TFMessage


class VideoProcessor:
    def __init__(self):
        self.counter = 0
        self.pub = rp.Publisher('/camera/color/image_raw/compressed/processed', CompressedImage, queue_size=10)
        self.sub = rp.Subscriber('/camera/color/image_raw/compressed', CompressedImage, self.callback)
    
    def callback(self, msg):
        # Reduce from 30fps to 15fps
        self.counter = (self.counter + 1) % 2
        if (self.counter==0):
            self.pub.publish(msg)
        

class PointCloudProcessor:
    def __init__(self):
        rp.init_node("point_processor", anonymous=True)
        self.counter = 0
        self.pub = rp.Publisher('/velodyne_points/processed', PointCloud2, queue_size=10)
        self.sub = rp.Subscriber('/velodyne_points', PointCloud2, self.callback)
    
    def callback(self, msg):
        self.counter = (self.counter + 1) % 3
        if self.counter==0:
            new_msg = pc2.create_cloud_xyz32(msg.header, pc2.read_points_list(msg, ['x','y','z'])) # extract only x,y,z data
            self.pub.publish(new_msg)
            print("Message Received...\n")
            print("  Number of points:  {}".format(len(msg.data)/msg.point_step))
            print("  Original size:     {}".format(len(msg.data)))
            print("  New size:          {}".format(len(new_msg.data)))

class TransformProcessor:
    def __init__(self):
        self.counter = 0
        self.pub = rp.Publisher('/tf/processed', TFMessage, queue_size=10)
        self.sub = rp.Subscriber('/tf', TFMessage, self.callback)
    
    def callback(self, msg):
        self.counter = (self.counter + 1) % 4
        if self.counter==0:
            self.pub.publish(msg)


def process_messages():
    p = PointCloudProcessor()
    v = VideoProcessor()
    t = TransformProcessor()
    rp.spin()


if __name__ == "__main__":
    process_messages()




            # print(pc2.read_points_list(msg, ['x','y','z'])[0])
            # print(type(pc2.read_points_list(msg, ['x','y','z'])[0]))
            # print(pc2.read_points_list(msg, ['x','y','z'])[0].x)
            # print(self.get_xyz_separately(pc2.read_points_list(msg, ['x','y','z'])))

            # self.pub.publish(msg)
            # print("Message Received...\n")
            # print("  Number of points: {}".format(len(msg.data)/msg.point_step))
            # print("  Data size:        {}".format(len(msg.data)))
            # print("  Fields:           {}".format((msg.fields[0].name)))
            # print("  Points\n:{}".format(point_cloud2.read_points_list(msg, ['x','y','z'])))
    
    # def get_xyz_separately(self, lst):
    #     """
    #     Converts a Point(x,y,z) array to a 3xN array
    #     """
    #     # x = [p.x for p in lst]
    #     # y = [p.y for p in lst]
    #     # z = [p.z for p in lst]
    #     pnts = [[p.x,p.y,p.z] for p in lst]
    #     mat = np.asmatrix(pnts)
    #     pcd = o3d.geometry.PointCloud()
    #     pcd.points = o3d.utility.Vector3dVector(mat)
    #     # o3d.visualization.draw_geometries([pcd])

    #     # mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_alpha_shape(pcd, 0.03)
    #     # mesh.compute_vertex_normals()
    #     # o3d.visualization.draw_geometries([mesh], mesh_show_back_face=True)

    #     pcd.normals = o3d.utility.Vector3dVector(np.zeros((1, 3)))  # invalidate existing normals
    #     pcd.estimate_normals()
    #     mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(pcd)
    #     # tetra_mesh, pt_map = o3d.geometry.TetraMesh.create_from_point_cloud(pcd)
    #     # mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(pcd, 0.03, tetra_mesh, pt_map)
    #     print(type(mesh))
    #     o3d.visualization.draw_geometries([mesh])
        
    #     return mat
