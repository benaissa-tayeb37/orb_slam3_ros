#ifndef ORB_SLAM3_ROS_COMMON_HPP
#define ORB_SLAM3_ROS_COMMON_HPP

#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <vector>
#include <queue>
#include <thread>
#include <mutex>
#include <memory>
#include <Eigen/Dense>

// ROS 2 headers
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time.hpp"
#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2/LinearMath/Transform.h"

// Standard message headers
#include "std_msgs/msg/header.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <visualization_msgs/msg/marker.hpp>

// Message filter headers
#include "message_filters/subscriber.h"
#include "message_filters/time_synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"

// OpenCV headers
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>

// Custom service definition
#include "orb_slam3_ros/srv/save_map.hpp"

// ORB-SLAM3-specific libraries
#include "System.h"
#include "ImuTypes.h"

// Add extern declarations for all shared global variables
extern ORB_SLAM3::System* pSLAM;
extern ORB_SLAM3::System::eSensor sensor_type;
extern std::string world_frame_id, cam_frame_id, imu_frame_id;
extern rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub;
extern rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;
extern rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr kf_markers_pub;
extern rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr tracked_mappoints_pub;
extern rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr all_mappoints_pub;
extern rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr tracked_keypoints_pub;
extern image_transport::Publisher tracking_img_pub;

// Function declarations
void setup_services(rclcpp::Node::SharedPtr node, std::string node_name);
void setup_publishers(rclcpp::Node::SharedPtr node, image_transport::ImageTransport& image_transport, std::string node_name);
void publish_topics(rclcpp::Time msg_time, Eigen::Vector3f Wbb = Eigen::Vector3f::Zero());

void publish_camera_pose(Sophus::SE3f Tcw_SE3f, rclcpp::Time msg_time);
void publish_tracking_img(cv::Mat image, rclcpp::Time msg_time);
void publish_tracked_points(std::vector<ORB_SLAM3::MapPoint*> tracked_points, rclcpp::Time msg_time);
void publish_keypoints(std::vector<ORB_SLAM3::MapPoint*> tracked_map_points, std::vector<cv::KeyPoint> tracked_keypoints, rclcpp::Time msg_time);
sensor_msgs::msg::PointCloud2 keypoints_to_pointcloud(std::vector<cv::KeyPoint>& keypoints, rclcpp::Time msg_time);

void publish_all_points(std::vector<ORB_SLAM3::MapPoint*> map_points, rclcpp::Time msg_time);
void publish_tf_transform(Sophus::SE3f T_SE3f, std::string frame_id, std::string child_frame_id, rclcpp::Time msg_time);
void publish_body_odom(Sophus::SE3f Twb_SE3f, Eigen::Vector3f Vwb_E3f, Eigen::Vector3f ang_vel_body, rclcpp::Time msg_time);
void publish_kf_markers(std::vector<Sophus::SE3f> vKFposes, rclcpp::Time msg_time);

// Helper function for converting Sophus::SE3f to geometry_msgs::msg::Pose
geometry_msgs::msg::Pose se3fToPoseMsg(const Sophus::SE3f& se3);

bool save_map_srv(const std::shared_ptr<orb_slam3_ros::srv::SaveMap::Request> req, 
                 std::shared_ptr<orb_slam3_ros::srv::SaveMap::Response> res);
bool save_traj_srv(const std::shared_ptr<orb_slam3_ros::srv::SaveMap::Request> req, 
                  std::shared_ptr<orb_slam3_ros::srv::SaveMap::Response> res);

cv::Mat SE3f_to_cvMat(Sophus::SE3f T_SE3f);
sensor_msgs::msg::PointCloud2 mappoint_to_pointcloud(std::vector<ORB_SLAM3::MapPoint*> map_points, rclcpp::Time msg_time);

#endif // ORB_SLAM3_ROS_COMMON_HPP