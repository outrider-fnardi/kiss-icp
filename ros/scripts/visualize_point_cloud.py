#!/usr/bin/env python

import open3d as o3d

# Load the point cloud from a PCD file
pcd = o3d.io.read_point_cloud("/home/fnardi/dataset/lidar_trailer_tracking/my_point_cloud.pcd")

# Visualize the point cloud
o3d.visualization.draw_geometries([pcd])
