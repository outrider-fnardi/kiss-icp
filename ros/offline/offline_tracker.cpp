#include <iostream>
#include <gflags/gflags.h>
#include <open3d/Open3D.h>
#include <open3d/utility/Eigen.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/PointCloud2.h>
#include "kiss_icp/pipeline/KissICP.hpp"
#include "../ros1/Utils.hpp"

DEFINE_string(map_filename, "", "Filename of the lidar map.");
DEFINE_string(bag_filename, "", "Filename of the rosbag.");

std::shared_ptr<open3d::geometry::Geometry> createReferenceFrame(const Eigen::Isometry3d& pose)
{
  // Create an empty point cloud
  auto point_cloud = std::make_shared<open3d::geometry::PointCloud>();

  // Add points to the point cloud
  point_cloud->points_.push_back(Eigen::Vector3d(0, 0, 0));  // origin
  point_cloud->points_.push_back(Eigen::Vector3d(1, 0, 0));  // x axis
  point_cloud->points_.push_back(Eigen::Vector3d(0, 1, 0));  // y axis
  point_cloud->points_.push_back(Eigen::Vector3d(0, 0, 1));  // z axis

  // Transform all points in the point cloud by the pose
  for (auto& point : point_cloud->points_)
  {
    point = pose * point;
  }

  // Create lines for the reference frame
  auto lines = std::make_shared<open3d::geometry::LineSet>();
  lines->points_ = point_cloud->points_;
  lines->lines_.push_back(Eigen::Vector2i(0, 1));  // origin to x axis
  lines->lines_.push_back(Eigen::Vector2i(0, 2));  // origin to y axis
  lines->lines_.push_back(Eigen::Vector2i(0, 3));  // origin to z axis

  // Set the colors for the lines
  lines->colors_ = std::vector<Eigen::Vector3d>{
    { 1, 0, 0 },  // red x axis
    { 0, 1, 0 },  // green y axis
    { 0, 0, 1 }   // blue z axis
  };

  // Return the drawable object
  return std::static_pointer_cast<open3d::geometry::Geometry>(lines);
}

bool play = false;
bool forward = false;

bool playCallback(open3d::visualization::Visualizer *vis)
{
  std::cerr << "play" << std::endl;
  play ^= 1;
  return true;
}

bool forwardCallback(open3d::visualization::Visualizer *vis)
{
  std::cerr << "forward" << std::endl;
  forward ^= 1;
  return true;
}


int main(int argc, char** argv)
{
  gflags::SetUsageMessage("Perform lidar tracking on rosbags");
  google::ParseCommandLineFlags(&argc, &argv, true);

  // Read the lidar map from disk
  //  std::cerr << "loading map: " << FLAGS_map_filename << std::endl;
  //  auto lidar_map = open3d::io::CreatePointCloudFromFile(FLAGS_map_filename);

  // Visualize the lidar map
  //  open3d::visualization::DrawGeometries({ lidar_map });

  // KISS-ICP
  kiss_icp::pipeline::KISSConfig config;
  config.voxel_size = 1.0;
  config.max_range = 100.0;
  config.min_range = 5.0;
  config.max_points_per_voxel = 20;
  config.min_motion_th = 0.1;
  config.initial_threshold = 2.0;
  config.deskew = false;
  kiss_icp::pipeline::KissICP odometry(config);

  // Global/map coordinate frame.
  std::string odom_frame{ "odom" };
  std::string child_frame{ "base_link" };

  // create bag object
  rosbag::Bag bag;
  bag.open(FLAGS_bag_filename, rosbag::bagmode::Read);

  // create bag view
  std::string topic = "aggregated_point_cloud";
  std::vector<std::string> topics{ { topic } };
  rosbag::View view(bag, rosbag::TopicQuery(topics));

  open3d::visualization::VisualizerWithKeyCallback visualizer;
  visualizer.CreateVisualizerWindow("Open3D VisualizerWithKeyCallback", 800, 600);
  visualizer.RegisterKeyCallback(80, playCallback);
  visualizer.RegisterKeyCallback(70, forwardCallback);


  for (auto m : view)
  {
    // check topic
    if (m.getTopic() != topic)
    {
      continue;
    }

    // try to instantiate message
    sensor_msgs::PointCloud2::ConstPtr msg = m.instantiate<sensor_msgs::PointCloud2>();
    if (msg == NULL)
    {
      continue;
    }

    // skip empty messages
    if (msg->data.empty())
    {
      continue;
    }

    // convert PointCloud2 to std::vector<Eigen::Vector3d> to make KISS-ICP happy
    const auto points = kiss_icp_ros::utils::PointCloud2ToEigen(*msg);

    // Register frame, main entry point to KISS-ICP pipeline
    const auto& [frame, keypoints] = odometry.RegisterFrame(points, {});

    const auto pose = odometry.poses().back();
    const Eigen::Vector3d t_current = pose.translation();
    const Eigen::Quaterniond q_current = pose.unit_quaternion();
    Eigen::Isometry3d transformation = Eigen::Isometry3d::Identity();
    transformation.translation() = t_current;
    transformation.linear() = q_current.toRotationMatrix();

    auto local_map = std::make_shared<open3d::geometry::PointCloud>();
    local_map->points_ = odometry.LocalMap();

    visualizer.ClearGeometries();
    visualizer.AddGeometry(createReferenceFrame(transformation));
    visualizer.AddGeometry(local_map);
    visualizer.UpdateGeometry();

    while (!play)
    {
      visualizer.PollEvents();
      visualizer.UpdateRender();

      if (forward)
      {
        forward = false;
        break;
      }
    }
    visualizer.PollEvents();
    visualizer.UpdateRender();
  }


  // Destroy the visualizer window
  visualizer.DestroyVisualizerWindow();

  return 0;
}
