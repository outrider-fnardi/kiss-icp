#!/usr/bin/env python

import rospy
import open3d as o3d
from sensor_msgs.msg import PointCloud2
import ros_numpy


def ros_pointcloud_to_open3d(msg):
    """Convert ros pointcloud message to open3d format."""
    pcd = o3d.geometry.PointCloud()
    array = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(msg)
    pcd.points = o3d.utility.Vector3dVector(array)
    return pcd

def callback(msg):
    # Convert the sensor_msgs::PointCloud2 message to an Open3D point cloud
    o3d_pc = ros_pointcloud_to_open3d(msg)

    # Save the Open3D point cloud to a file in Open3D format
    o3d.io.write_point_cloud("/home/fnardi/dataset/lidar_trailer_tracking/my_point_cloud.pcd", o3d_pc)

rospy.init_node('save_map')
rospy.Subscriber('/odometry_node/local_map', PointCloud2, callback)
rospy.spin()
