#pragma once

#include <memory>
#include <mutex>
#include <string>

#include "behaviortree_cpp_v3/action_node.h"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace rb1_bt {

class CustomDriveOnHeading : public BT::StatefulActionNode {
public:
  CustomDriveOnHeading(const std::string &name,
                       const BT::NodeConfiguration &config);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  void ensureInterfaces();
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void publishCmd(double linear_x);
  void stopRobot();

  rclcpp::Node::SharedPtr node_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

  std::string cmd_vel_topic_;
  std::string odom_topic_;

  std::mutex odom_mutex_;
  nav_msgs::msg::Odometry latest_odom_;
  bool have_odom_{false};

  double target_distance_{0.0};
  double speed_{0.0};
  double time_allowance_{0.0};

  double start_x_{0.0};
  double start_y_{0.0};
  double start_yaw_{0.0};
  bool initialized_from_odom_{false};

  rclcpp::Time start_time_;
};

} // namespace rb1_bt