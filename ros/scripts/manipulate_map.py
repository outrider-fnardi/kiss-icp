#!/usr/bin/env python

import rospy
import numpy as np
from sensor_msgs.msg import PointCloud2
from message_filters import ApproximateTimeSynchronizer, Subscriber
import ros_numpy
import open3d as o3d
from sensor_msgs import point_cloud2
import tf2_ros
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
from tf import transformations
from std_msgs.msg import Header

# Specify the path to the point cloud file
pcd_path = "/home/fnardi/dataset/lidar_trailer_tracking/complete_lidar_map.pcd"

# Load the point cloud from disk
pcd = o3d.io.read_point_cloud(pcd_path)

# Define the 3 axes of the reference frame
axes = o3d.geometry.TriangleMesh.create_coordinate_frame(size=1.0)

# Define the translation vector for the transform
translation = np.array([-11.5, 4.672, -1.377])

# Define the 4x4 transformation matrix
transformation = [[0,1,0,4.672],
                  [-1,0,0,11.5],
                  [0,0,1,-1.377],
                  [0,0,0,1]]

# Apply the transform to the point cloud and the axes
pcd.transform(transformation)

x_min=-15; x_max=5; y_min=-2; y_max=2; z_min=-50; z_max=50
filtered_points = []
for point in pcd.points:
    if(point[0] > x_min and point[0] < x_max \
    and point[1] > y_min and point[1] < y_max \
    and point[2] > z_min and point[2] < z_max):    
        filtered_points.append(point)

filtered_cloud = o3d.geometry.PointCloud()
filtered_cloud.points = o3d.utility.Vector3dVector(
    np.array(filtered_points)
    )

# Visualize the point cloud using Open3D's built-in viewer
o3d.visualization.draw_geometries([filtered_cloud,
                                   axes, 
                                   ])

# store map on disk
o3d.io.write_point_cloud("/home/fnardi/dataset/lidar_trailer_tracking/lidar_map.pcd", filtered_cloud)
