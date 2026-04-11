#pragma once

#include "rb1_nav2_bt_nodes/patrol_types.hpp"

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2_ros/buffer.h>

#include <optional>
#include <string>
#include <vector>

namespace rb1_bt::scan_utils {

Point2D polarToCartesian(double range, double angle_rad);

bool normalizeWindow(int requested_start_idx, int requested_end_idx,
                     int scan_size, int &normalized_start_idx,
                     int &normalized_end_idx);

double distance2D(const Point2D &a, const Point2D &b);

double distance2D(const geometry_msgs::msg::Point &a,
                  const geometry_msgs::msg::Point &b);

geometry_msgs::msg::Point
averageGeometryPoints(const std::vector<geometry_msgs::msg::Point> &points);

geometry_msgs::msg::PoseStamped
buildPoseStampedFromWaypoint(const SimplePose2D &waypoint,
                             const std::string &frame_id,
                             const rclcpp::Time &stamp);

std::vector<ScanCluster>
extractClustersFromIndexWindow(const sensor_msgs::msg::LaserScan &scan,
                               int start_idx, int end_idx,
                               const ShelfDetectorParams &params);

bool transformPointToFrame(const tf2_ros::Buffer &tf_buffer,
                           const std::string &source_frame,
                           const std::string &target_frame,
                           const rclcpp::Time &stamp,
                           const Point2D &input_point,
                           geometry_msgs::msg::Point &output_point,
                           const rclcpp::Logger &logger);

bool transformPointToFrame(
    const tf2_ros::Buffer &tf_buffer, const std::string &source_frame,
    const std::string &target_frame, const rclcpp::Time &stamp,
    const Point2D &input_point, geometry_msgs::msg::Point &output_point,
    const rclcpp::Logger &logger, const TransformPointOptions &options);

/*
bool findClosestTransformedClusterInWindow(
    const sensor_msgs::msg::LaserScan &scan, int start_idx, int end_idx,
    const ShelfDetectorParams &params, const tf2_ros::Buffer &tf_buffer,
    const std::string &target_frame, const rclcpp::Time &transform_time,
    const geometry_msgs::msg::Point &reference_point_target_frame,
    double max_match_distance, TransformedClusterMatch &match_out,
    const rclcpp::Logger &logger, const TransformPointOptions &tf_options);
*/
std::optional<ShelfCandidateDetection> detectShelfCandidateInWindow(
    const sensor_msgs::msg::LaserScan &scan, int start_idx, int end_idx,
    const ShelfDetectorParams &params, const tf2_ros::Buffer &tf_buffer,
    const std::string &target_frame, const rclcpp::Time &transform_time,
    const rclcpp::Logger &logger);

} // namespace rb1_bt::scan_utils