#include <iostream>
#include <gflags/gflags.h>
#include <open3d/Open3D.h>
#include <open3d/utility/Eigen.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/PointCloud2.h>
#include "kiss_icp/pipeline/KissICP.hpp"
#include "../ros1/Utils.hpp"
#include <tf2/buffer_core.h>
#include <tf2_msgs/TFMessage.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <sensor_msgs/LaserScan.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sstream>
#include <string>

DEFINE_string(map_filename, "", "Filename of the lidar map.");
DEFINE_string(bag_filename, "", "Filename of the rosbag.");

template <class M>
class BagSubscriber : public message_filters::SimpleFilter<M>
{
public:
  void newMessage(const boost::shared_ptr<M const>& msg)
  {
    this->signalMessage(msg);
  }
};


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
bool stop = false;

bool playCallback(open3d::visualization::Visualizer* vis)
{
  std::cerr << "play" << std::endl;
  play ^= 1;
  return true;
}

bool forwardCallback(open3d::visualization::Visualizer* vis)
{
  std::cerr << "forward" << std::endl;
  forward ^= 1;
  return true;
}

bool stopCallback(open3d::visualization::Visualizer* vis)
{
  std::cerr << "stop" << std::endl;
  stop = true;
  return true;
}

std::shared_ptr<open3d::geometry::PointCloud> fromROStoOpen3d(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
  auto pcd = std::make_shared<open3d::geometry::PointCloud>();
  pcd->points_ = kiss_icp_ros::utils::PointCloud2ToEigen(*msg);
  return std::static_pointer_cast<open3d::geometry::PointCloud>(pcd);
}

void fillTfBuffer(const rosbag::Bag& bag, tf2_ros::Buffer& tf_buffer)
{
  std::vector<std::string> topics;
  topics.push_back("/tf");
  topics.push_back("/tf_static");
  rosbag::View view(bag, rosbag::TopicQuery(topics));

  // fill tf buffer
  for (const rosbag::MessageInstance& msg : rosbag::View(bag))
  {
    tf2_msgs::TFMessageConstPtr tf{ msg.instantiate<tf2_msgs::TFMessage>() };
    if (tf != nullptr)
    {
      for (size_t i = 0; i < tf->transforms.size(); ++i)
      {
        tf_buffer.setTransform(tf->transforms[i], "rosbag",
                               msg.getTopic() == "/tf_static" || msg.getTopic() == "tf_static");
      }
    }
  }
}

bool getPose(const tf2_ros::Buffer& tf_buffer, const std::string& target_frame,
             const std::string& source_frame, const ros::Time& t, tf2::Transform& pose)
{
  geometry_msgs::TransformStamped tranform;
  try
  {
    tranform = tf_buffer.lookupTransform(target_frame, source_frame, t);
  }
  catch (tf2::TransformException& ex)
  {
    ROS_WARN("%s", ex.what());
    return false;
  }
  tf2::convert(tranform.transform, pose);

  return true;
}

Eigen::Isometry3d tf2Eigen(const tf2::Transform& pose)
{
  Eigen::Isometry3d iso = Eigen::Isometry3d::Identity();
  iso.translation() = Eigen::Vector3d(pose.getOrigin().getX(), pose.getOrigin().getY(),
                                      pose.getOrigin().getZ());
  iso.linear() = Eigen::Quaterniond(pose.getRotation().getW(), pose.getRotation().getX(),
                                    pose.getRotation().getY(), pose.getRotation().getZ())
                     .toRotationMatrix();
  return iso;
}

tf2_ros::Buffer tf_buffer;
std::vector<tf2::Transform> poses(3);

// KISS-ICP
kiss_icp::pipeline::KissICP tracker(kiss_icp::pipeline::KISSConfig{ 0.1, 100.0, 0, 20,
                                                                    0.1, 2.0, false });
open3d::visualization::VisualizerWithKeyCallback visualizer;

auto trailer_pcd = std::make_shared<open3d::geometry::PointCloud>();
std::vector<Eigen::Vector3d> trailer_cloud;

Sophus::SE3d initial_pose;

void msgCallback(const sensor_msgs::PointCloud2::ConstPtr& msg_1,
                 const sensor_msgs::PointCloud2::ConstPtr& msg_2,
                 const sensor_msgs::PointCloud2::ConstPtr& msg_3)
{
  std::shared_ptr<open3d::geometry::PointCloud> cab_pcd = fromROStoOpen3d(msg_1);
  std::shared_ptr<open3d::geometry::PointCloud> left_pcd = fromROStoOpen3d(msg_2);
  std::shared_ptr<open3d::geometry::PointCloud> right_pcd = fromROStoOpen3d(msg_3);

  cab_pcd->Transform(tf2Eigen(poses[0]).matrix());
  left_pcd->Transform(tf2Eigen(poses[1]).matrix());
  right_pcd->Transform(tf2Eigen(poses[2]).matrix());

  auto merged_pcd = std::make_shared<open3d::geometry::PointCloud>();
  *merged_pcd = *cab_pcd;
  *merged_pcd += *left_pcd;
  *merged_pcd += *right_pcd;

  double voxel_size = 0.5;
  auto reference_pcd = merged_pcd->VoxelDownSample(voxel_size);
  reference_pcd->PaintUniformColor(Eigen::Vector3d(0.0, 0.0, 1.0));

  tracker.local_map_.Update(reference_pcd->points_, Sophus::SE3d());

  // Register frame, main entry point to KISS-ICP pipeline
  const auto& [frame, keypoints] = tracker.RegisterFrame(trailer_cloud, {});

  const auto kicp_pose = tracker.poses().back();

  std::cerr << kicp_pose.translation().transpose() << std::endl;

  visualizer.ClearGeometries();

  visualizer.AddGeometry(open3d::geometry::TriangleMesh::CreateCoordinateFrame());
  visualizer.AddGeometry(reference_pcd);

  auto trailer_frame = open3d::geometry::TriangleMesh::CreateCoordinateFrame();
  trailer_frame->Transform(kicp_pose.matrix());
  visualizer.AddGeometry(trailer_frame);

  auto visualization_pcd = std::make_shared<open3d::geometry::PointCloud>();
  *visualization_pcd = *trailer_pcd;
  visualization_pcd->Transform(kicp_pose.matrix());
  visualization_pcd->PaintUniformColor(Eigen::Vector3d(1.0, 0.0, 0.0));
  visualizer.AddGeometry(visualization_pcd);

  visualizer.UpdateGeometry();

  while (!play)
  {
    visualizer.PollEvents();
    visualizer.UpdateRender();

    if (forward or stop)
    {
      forward = false;
      break;
    }
  }
  visualizer.PollEvents();
  visualizer.UpdateRender();

}

int main(int argc, char** argv)
{
  gflags::SetUsageMessage("Perform lidar tracking on rosbags");
  google::ParseCommandLineFlags(&argc, &argv, true);

  // set initial pose
  Eigen::Matrix3d R = Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  initial_pose.setRotationMatrix(R);
  initial_pose.translation() << 11.5, -4.672, 1.377;
  tracker.poses_.push_back(initial_pose);

  trailer_pcd = open3d::io::CreatePointCloudFromFile(FLAGS_map_filename);
  trailer_cloud = trailer_pcd->points_;
  trailer_pcd->PaintUniformColor(Eigen::Vector3d(0.4, 0.4, 0.4));

  // Read the lidar map from disk
  //  std::cerr << "loading map: " << FLAGS_map_filename << std::endl;

  ros::Time::init();

  // Global/map coordinate frame.
  std::string target_frame{ "odom" };
  std::string source_frame{ "base_link" };

  // create bag object
  rosbag::Bag bag;
  bag.open(FLAGS_bag_filename, rosbag::bagmode::Read);

  // create bag view
  std::vector<std::string> topics;
  topics.push_back("/ak/lidar_cab/point_cloud");
  topics.push_back("/ak/lidar_front_left/point_cloud");
  topics.push_back("/ak/lidar_front_right/point_cloud");
  rosbag::View view(bag, rosbag::TopicQuery(topics));

  // lookup lidar extrinsics
  fillTfBuffer(bag, tf_buffer);
  for (int i = 0, num_topics = topics.size(); i < num_topics; ++i)
  {
    const std::string& topic = topics[i];
    std::smatch matches;
    if (std::regex_search(topic, matches, std::regex(R"(/([^/]+)/([^/]+)/)")))
    {
      if (!getPose(tf_buffer, source_frame, matches[2].str() + "/hc", ros::Time(0), poses[i]))
      {
        throw std::runtime_error("No transform!!!");
      }
    }
  }

  // create message filter synchronizer
  std::vector<BagSubscriber<sensor_msgs::PointCloud2>> subs(3);
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2,
                                                          sensor_msgs::PointCloud2, sensor_msgs::PointCloud2>
      sync_pol;
  message_filters::Synchronizer<sync_pol> sync(sync_pol(10), subs[0], subs[1], subs[2]);
  sync.registerCallback(boost::bind(msgCallback, _1, _2, _3));

  // create viewer
  visualizer.CreateVisualizerWindow("Open3D VisualizerWithKeyCallback", 800, 600);
  visualizer.RegisterKeyCallback(80, playCallback);
  visualizer.RegisterKeyCallback(70, forwardCallback);
  visualizer.RegisterKeyCallback(256, stopCallback);

  // process bag
  for (auto m : view)
  {
    for (int i = 0; i < 3; ++i)
    {
      if (m.getTopic() == topics[i] || ("/" + m.getTopic() == topics[i]))
      {
        sensor_msgs::PointCloud2::ConstPtr msg = m.instantiate<sensor_msgs::PointCloud2>();
        if (msg != NULL)
        {
          subs[i].newMessage(msg);
        }
      }
    }
    if (stop)
    {
      break;
    }
  }

  //  open3d::geometry::PointCloud local_map;
  //  local_map.points_ = odometry.LocalMap();
  //  open3d::io::WritePointCloud(FLAGS_map_filename, local_map);


  // Destroy the visualizer window
  while (!stop)
  {
    visualizer.PollEvents();
    visualizer.UpdateRender();
  }
  visualizer.DestroyVisualizerWindow();

  // close bag
  bag.close();

  return 0;
}
