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


def ros_pointcloud_to_open3d(msg):
    """Convert ros pointcloud message to open3d format."""
    pcd = o3d.geometry.PointCloud()
    array = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(msg)
    pcd.points = o3d.utility.Vector3dVector(array)
    return pcd

def ros_transform_to_matrix(tf):

    # Get the translation and rotation as numpy arrays
    translation = np.array([tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z])
    rotation_quat = np.array([tf.transform.rotation.x, tf.transform.rotation.y, tf.transform.rotation.z, tf.transform.rotation.w])

    # Convert the rotation quaternion to a 3x3 rotation matrix
    rotation_matrix = transformations.quaternion_matrix(rotation_quat)[:3, :3]

    # Convert the transform to a 4x4 transformation matrix
    T = np.identity(4)
    T[:3, :3] = rotation_matrix
    T[:3, 3] = translation
    
    return T

class PointCloudAggregator:
    def __init__(self):
        self.aggregated_pc = o3d.geometry.PointCloud()
        self.got_transforms = False
        
        # Subscribers
        sub_pc1 = Subscriber('/ak/lidar_cab/point_cloud', PointCloud2)
        sub_pc2 = Subscriber('/ak/lidar_front_left/point_cloud', PointCloud2)
        sub_pc3 = Subscriber('/ak/lidar_front_right/point_cloud', PointCloud2)

        # Message filter to synchronize the messages
        ts = ApproximateTimeSynchronizer([sub_pc1, sub_pc2, sub_pc3], queue_size=1, slop=0.1)
        ts.registerCallback(self.callback)

        # Publisher
        self.pub = rospy.Publisher('/aggregated_point_cloud', PointCloud2, queue_size=10)

        # TF buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # Wait for transforms
        while not rospy.is_shutdown():
            try:
                tf1 = self.tf_buffer.lookup_transform('base_link', 'lidar_cab/hc', rospy.Time(), rospy.Duration(2.0))
                tf2 = self.tf_buffer.lookup_transform('base_link', 'lidar_front_left/hc', rospy.Time(), rospy.Duration(2.0))
                tf3 = self.tf_buffer.lookup_transform('base_link', 'lidar_front_right/hc', rospy.Time(), rospy.Duration(2.0))
                break
            except Exception as e:
               rospy.loginfo(f"Failed to lookup transform: {str(e)}. Trying again...")
                
        self.got_transforms = True
        rospy.loginfo("got transforms")

        self.transform_pc1 = ros_transform_to_matrix(tf1)
        self.transform_pc2 = ros_transform_to_matrix(tf2)
        self.transform_pc3 = ros_transform_to_matrix(tf3)

        # define cropping bounding box
        self.bbox = o3d.geometry.AxisAlignedBoundingBox(min_bound=(-1.5, -1.75, -2), max_bound=(5, 1.75, 2))

        



    def callback(self, pc1_msg, pc2_msg, pc3_msg):
        
        if not self.got_transforms:
            return

        # Convert point clouds to open3d
        pc1 = ros_pointcloud_to_open3d(pc1_msg)
        pc2 = ros_pointcloud_to_open3d(pc2_msg)
        pc3 = ros_pointcloud_to_open3d(pc3_msg)
        
        # transform point clouds in base_link frame
        pc1.transform(self.transform_pc1)
        pc2.transform(self.transform_pc2)
        pc3.transform(self.transform_pc3)

        # Concatenate the point clouds into a single point cloud
        self.aggregated_pc.clear()
        self.aggregated_pc = pc1
        self.aggregated_pc += pc2
        self.aggregated_pc += pc3

    def run(self):
        r = rospy.Rate(10)  # 10Hz
        while not rospy.is_shutdown():
            if self.aggregated_pc is not None:
            
                # Get indices of points inside the bounding box
                inside_indices = np.array(self.aggregated_pc.points)[:,0] < self.bbox.max_bound[0]
                inside_indices &= np.array(self.aggregated_pc.points)[:,0] > self.bbox.min_bound[0]
                inside_indices &= np.array(self.aggregated_pc.points)[:,1] < self.bbox.max_bound[1]
                inside_indices &= np.array(self.aggregated_pc.points)[:,1] > self.bbox.min_bound[1]
                inside_indices &= np.array(self.aggregated_pc.points)[:,2] < self.bbox.max_bound[2]
                inside_indices &= np.array(self.aggregated_pc.points)[:,2] > self.bbox.min_bound[2]

                # Remove points inside the bounding box from point cloud
                self.aggregated_pc = self.aggregated_pc.select_by_index(np.where(inside_indices == False)[0])

                # Downsample the point cloud
                self.aggregated_pc = self.aggregated_pc.voxel_down_sample(voxel_size=0.1)

    
                # Create the PointCloud2 message
                header = Header()
                header.stamp = rospy.Time.now()
                header.frame_id = "base_link"
                pc_msg = point_cloud2.create_cloud_xyz32(header, self.aggregated_pc.points)
                self.pub.publish(pc_msg)

            r.sleep()

if __name__ == '__main__':
    rospy.init_node('point_cloud_aggregator')
    node = PointCloudAggregator()
    node.run()

