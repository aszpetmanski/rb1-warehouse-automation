#pragma once

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2_ros/buffer.h>

#include <optional>
#include <string>
#include <vector>

#include "rb1_nav2_bt_nodes/patrol_types.hpp"

namespace rb1_bt::scan_utils {

// Builds a PoseStamped from a SimplePose2D waypoint.
geometry_msgs::msg::PoseStamped
buildPoseStampedFromWaypoint(const SimplePose2D &waypoint,
                             const std::string &frame_id,
                             const rclcpp::Time &stamp);

// Converts a polar LiDAR measurement into a 2D Cartesian point.
Point2D polarToCartesian(double range, double angle_rad);

// Clamps a scan index window into valid bounds.
bool normalizeWindow(int requested_start_idx, int requested_end_idx,
                     int scan_size, int &normalized_start_idx,
                     int &normalized_end_idx);

double normalizeAngle(double angle);

// Returns the 2D distance between two Point2D values.
double distance2D(const Point2D &a, const Point2D &b);

// Returns the 2D distance between two geometry points.
double distance2D(const geometry_msgs::msg::Point &a,
                  const geometry_msgs::msg::Point &b);

double distance2D(const SimplePose2D &a, const SimplePose2D &b);
double distance2D(const SimplePose2D &a, const geometry_msgs::msg::Point &b);
double distance2D(const geometry_msgs::msg::Point &a, const SimplePose2D &b);

// Computes the average of a list of geometry points.
geometry_msgs::msg::Point
averageGeometryPoints(const std::vector<geometry_msgs::msg::Point> &points);

// Computes midpoint of two geometry points.
geometry_msgs::msg::Point
midpointGeometryPoints(const geometry_msgs::msg::Point &a,
                       const geometry_msgs::msg::Point &b);

// Builds a quaternion from yaw angle.
geometry_msgs::msg::Quaternion yawToQuaternion(double yaw);

// Gets current robot origin position in target_frame by transforming
// (0,0,0) from robot_frame.
bool getRobotPoseInFrame(const tf2_ros::Buffer &tf_buffer,
                         const std::string &target_frame,
                         SimplePose2D &robot_pose_out,
                         const rclcpp::Logger &logger,
                         const std::string &robot_frame = "robot_base_link");

// Extracts contiguous point clusters from a scan index window.
std::vector<ScanCluster>
extractClustersFromIndexWindow(const sensor_msgs::msg::LaserScan &scan,
                               int start_idx, int end_idx,
                               const ShelfDetectorParams &params);

// Convenience overload that transforms a 2D point using default TF options.
bool transformPointToFrame(const tf2_ros::Buffer &tf_buffer,
                           const std::string &source_frame,
                           const std::string &target_frame,
                           const rclcpp::Time &stamp,
                           const Point2D &input_point,
                           geometry_msgs::msg::Point &output_point,
                           const rclcpp::Logger &logger);

// Transforms a 2D point from one frame into another using the latest
// available TF. No exact-time lookup is attempted.
bool transformPointToFrame(const tf2_ros::Buffer &tf_buffer,
                           const std::string &source_frame,
                           const std::string &target_frame,
                           const Point2D &input_point,
                           geometry_msgs::msg::Point &output_point,
                           const rclcpp::Logger &logger);

// Legacy compatibility overload: stamp is ignored and latest TF is used.
bool transformPointToFrame(const tf2_ros::Buffer &tf_buffer,
                           const std::string &source_frame,
                           const std::string &target_frame,
                           const rclcpp::Time &stamp,
                           const Point2D &input_point,
                           geometry_msgs::msg::Point &output_point,
                           const rclcpp::Logger &logger);

// Legacy compatibility overload: stamp/options are ignored and latest TF is
// used.
bool transformPointToFrame(
    const tf2_ros::Buffer &tf_buffer, const std::string &source_frame,
    const std::string &target_frame, const rclcpp::Time &stamp,
    const Point2D &input_point, geometry_msgs::msg::Point &output_point,
    const rclcpp::Logger &logger, const TransformPointOptions &options);
std::optional<ShelfCandidateDetection> detectShelfCandidateInWindow(
    const sensor_msgs::msg::LaserScan &scan, int start_idx, int end_idx,
    const ShelfDetectorParams &params, const tf2_ros::Buffer &tf_buffer,
    const std::string &target_frame, const rclcpp::Time &transform_time,
    const rclcpp::Logger &logger);

// Evaluates which side of the robot looks freer in the laser frame.
// Windows are centered around +90deg (left) and -90deg (right).
// preferred_turn_sign:
//   +1 => left side is freer
//   -1 => right side is freer
//    0 => no clear preference
SideClearanceDecision
detectPreferredBackupTurnSide(const sensor_msgs::msg::LaserScan &scan,
                              int side_window_size, double side_range_cap,
                              double min_preference_diff);

} // namespace rb1_bt::scan_utils