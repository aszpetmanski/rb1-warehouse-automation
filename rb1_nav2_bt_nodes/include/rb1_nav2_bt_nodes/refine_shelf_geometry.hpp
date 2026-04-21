#pragma once

#include <memory>
#include <mutex>
#include <string>

#include "behaviortree_cpp_v3/action_node.h"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2_ros/buffer.h"

namespace rb1_bt {

class RefineShelfGeometry : public BT::StatefulActionNode {
public:
  RefineShelfGeometry(const std::string &name,
                      const BT::NodeConfiguration &config);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  bool initializeRosNode();
  void ensureInterfaces();

  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void publishCmd(double linear_x, double angular_z = 0.0);
  void stopRobot();

  bool detectBestShelfGeometry(const sensor_msgs::msg::LaserScan &scan,
                               geometry_msgs::msg::Point &center_out,
                               geometry_msgs::msg::Point &left_leg_out,
                               geometry_msgs::msg::Point &right_leg_out,
                               double &confidence_out, double &spacing_out,
                               std::string &reason_out);

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;

  std::string scan_topic_;
  std::string cmd_vel_topic_;

  std::mutex scan_mutex_;
  sensor_msgs::msg::LaserScan latest_scan_;
  bool have_scan_{false};

  int front_window_start_idx_{430};
  int front_window_end_idx_{650};

  double expected_leg_spacing_{0.60};
  double leg_spacing_tolerance_{0.07};
  double cluster_gap_tolerance_{0.05};
  double intensity_threshold_{3000.0};
  double max_detection_range_{1.8};
  int min_cluster_points_{5};
  int min_consecutive_hits_{3};
  double stable_hit_distance_tol_{0.08};
  double stop_settle_time_{0.30};
  double wait_duration_{4.0};
  double max_scan_age_{0.40};

  int side_window_size_{100};
  double side_range_cap_{2.0};
  double side_free_min_diff_{0.10};

  std::string target_frame_{"map"};

  rclcpp::Time start_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time settle_deadline_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_processed_scan_stamp_{0, 0, RCL_ROS_TIME};

  bool have_last_processed_scan_{false};
  bool have_last_accepted_center_{false};
  geometry_msgs::msg::Point last_accepted_center_;
  int accepted_hits_{0};
};

} // namespace rb1_bt