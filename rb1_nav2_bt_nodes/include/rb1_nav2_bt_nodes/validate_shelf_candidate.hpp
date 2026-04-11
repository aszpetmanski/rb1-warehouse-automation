#pragma once

#include <behaviortree_cpp_v3/action_node.h>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <memory>
#include <mutex>
#include <string>

namespace rb1_bt {

class ValidateShelfCandidate : public BT::StatefulActionNode {
public:
  ValidateShelfCandidate(const std::string &name,
                         const BT::NodeConfiguration &config);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  struct Detection {
    geometry_msgs::msg::Point center;
    geometry_msgs::msg::Point left_leg;
    geometry_msgs::msg::Point right_leg;
    double spacing{0.0};
  };

  bool initializeRosNode();
  void configureInterfaces();
  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);

  void publishZeroVelocity() const;
  bool fetchLatestScan(sensor_msgs::msg::LaserScan &scan) const;

  bool validateFromScan(const sensor_msgs::msg::LaserScan &scan,
                        Detection &detection_out) const;

  bool isStableAgainstPrevious(const Detection &current) const;

  void logRejectThrottled(const char *fmt, ...) const;
  void resetConsecutiveValidation();
  void accumulateDetection(const Detection &detection);
  Detection averagedDetection() const;

  rclcpp::Node::SharedPtr node_;

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  mutable std::mutex scan_mutex_;
  sensor_msgs::msg::LaserScan latest_scan_;
  bool has_scan_{false};

  std::string scan_topic_{"/scan"};
  std::string cmd_vel_topic_{"/cmd_vel"};
  std::string target_frame_{"map"};

  int left_window_start_idx_{55};
  int left_window_end_idx_{305};
  int right_window_start_idx_{775};
  int right_window_end_idx_{1025};

  double expected_leg_spacing_{0.60};
  double leg_spacing_tolerance_{0.14};
  double cluster_gap_tolerance_{0.10};
  double intensity_threshold_{7500.0};
  double max_detection_range_{2.5};
  int min_cluster_points_{2};
  int min_consecutive_hits_{2};
  double stable_hit_distance_tol_{0.18};
  double stop_settle_time_{0.40};
  double wait_duration_sec_{10.0};
  double max_scan_age_sec_{0.90};

  double tf_exact_timeout_sec_{0.03};
  double tf_fallback_timeout_sec_{0.05};

  rclcpp::Time start_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_processed_scan_stamp_{0, 0, RCL_ROS_TIME};
  bool has_processed_scan_{false};

  int consecutive_valid_hits_{0};
  bool has_last_valid_detection_{false};
  Detection last_valid_detection_;

  geometry_msgs::msg::Point accumulated_center_;
  geometry_msgs::msg::Point accumulated_left_leg_;
  geometry_msgs::msg::Point accumulated_right_leg_;
  double accumulated_spacing_{0.0};
  int accumulated_count_{0};
  std::string candidate_window_;
};

} // namespace rb1_bt