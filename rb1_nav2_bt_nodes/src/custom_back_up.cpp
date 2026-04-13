#include "rb1_nav2_bt_nodes/custom_back_up.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"

namespace rb1_bt {

CustomBackUp::CustomBackUp(const std::string &name,
                           const BT::NodeConfiguration &config)
    : BT::StatefulActionNode(name, config) {
  node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
}

BT::PortsList CustomBackUp::providedPorts() {
  return {
      BT::InputPort<double>("distance", 0.5,
                            "Distance to move backward in meters (positive)"),
      BT::InputPort<double>("speed", 0.1,
                            "Backward speed magnitude in m/s (positive)"),
      BT::InputPort<double>("time_allowance", 0.0,
                            "Timeout in seconds; <= 0 disables timeout"),
      BT::InputPort<std::string>("cmd_vel_topic", "/cmd_vel", "cmd_vel topic"),
      BT::InputPort<std::string>("odom_topic", "/odom", "Odometry topic")};
}

void CustomBackUp::ensureInterfaces() {
  std::string new_cmd_vel_topic;
  std::string new_odom_topic;

  getInput("cmd_vel_topic", new_cmd_vel_topic);
  getInput("odom_topic", new_odom_topic);

  if (!cmd_pub_ || new_cmd_vel_topic != cmd_vel_topic_) {
    cmd_vel_topic_ = new_cmd_vel_topic;
    cmd_pub_ =
        node_->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);
  }

  if (!odom_sub_ || new_odom_topic != odom_topic_) {
    odom_topic_ = new_odom_topic;
    odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
        odom_topic_, rclcpp::SensorDataQoS(),
        std::bind(&CustomBackUp::odomCallback, this, std::placeholders::_1));
  }
}

void CustomBackUp::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(odom_mutex_);
  latest_odom_ = *msg;
  have_odom_ = true;
}

void CustomBackUp::publishCmd(double linear_x) {
  geometry_msgs::msg::Twist cmd;
  cmd.linear.x = linear_x;
  cmd.angular.z = 0.0;
  cmd_pub_->publish(cmd);
}

void CustomBackUp::stopRobot() {
  if (cmd_pub_) {
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = 0.0;
    cmd.angular.z = 0.0;
    cmd_pub_->publish(cmd);
  }
}

BT::NodeStatus CustomBackUp::onStart() {
  ensureInterfaces();

  if (!getInput("distance", target_distance_)) {
    RCLCPP_ERROR(node_->get_logger(), "CustomBackUp: missing input [distance]");
    return BT::NodeStatus::FAILURE;
  }
  if (!getInput("speed", speed_)) {
    RCLCPP_ERROR(node_->get_logger(), "CustomBackUp: missing input [speed]");
    return BT::NodeStatus::FAILURE;
  }
  getInput("time_allowance", time_allowance_);

  target_distance_ = std::abs(target_distance_);
  speed_ = std::abs(speed_);

  if (target_distance_ < 1e-4) {
    RCLCPP_WARN(node_->get_logger(),
                "CustomBackUp: distance ~ 0, returning SUCCESS");
    return BT::NodeStatus::SUCCESS;
  }

  if (speed_ < 1e-4) {
    RCLCPP_ERROR(node_->get_logger(), "CustomBackUp: speed must be > 0");
    return BT::NodeStatus::FAILURE;
  }

  initialized_from_odom_ = false;
  start_time_ = node_->now();

  RCLCPP_INFO(
      node_->get_logger(),
      "CustomBackUp: START distance=%.3f m speed=%.3f m/s cmd_vel=%s odom=%s",
      target_distance_, speed_, cmd_vel_topic_.c_str(), odom_topic_.c_str());

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus CustomBackUp::onRunning() {
  nav_msgs::msg::Odometry odom;
  {
    std::lock_guard<std::mutex> lock(odom_mutex_);
    if (!have_odom_) {
      if (time_allowance_ > 0.0) {
        const double elapsed = (node_->now() - start_time_).seconds();
        if (elapsed > time_allowance_) {
          stopRobot();
          RCLCPP_ERROR(
              node_->get_logger(),
              "CustomBackUp: TIMEOUT waiting for first odom after %.3f s",
              elapsed);
          return BT::NodeStatus::FAILURE;
        }
      }
      return BT::NodeStatus::RUNNING;
    }
    odom = latest_odom_;
  }

  if (!initialized_from_odom_) {
    start_x_ = odom.pose.pose.position.x;
    start_y_ = odom.pose.pose.position.y;
    start_yaw_ = tf2::getYaw(odom.pose.pose.orientation);
    initialized_from_odom_ = true;

    publishCmd(-speed_);

    RCLCPP_INFO(node_->get_logger(),
                "CustomBackUp: first odom received, starting motion from "
                "(%.3f, %.3f, yaw=%.3f)",
                start_x_, start_y_, start_yaw_);

    return BT::NodeStatus::RUNNING;
  }

  const double dx = odom.pose.pose.position.x - start_x_;
  const double dy = odom.pose.pose.position.y - start_y_;

  const double progress =
      -(dx * std::cos(start_yaw_) + dy * std::sin(start_yaw_));

  if (time_allowance_ > 0.0) {
    const double elapsed = (node_->now() - start_time_).seconds();
    if (elapsed > time_allowance_) {
      stopRobot();
      RCLCPP_ERROR(node_->get_logger(),
                   "CustomBackUp: TIMEOUT after %.3f s, progress=%.3f / %.3f",
                   elapsed, progress, target_distance_);
      return BT::NodeStatus::FAILURE;
    }
  }

  if (progress >= target_distance_) {
    stopRobot();
    RCLCPP_INFO(node_->get_logger(),
                "CustomBackUp: SUCCESS progress=%.3f / %.3f", progress,
                target_distance_);
    return BT::NodeStatus::SUCCESS;
  }

  publishCmd(-speed_);
  return BT::NodeStatus::RUNNING;
}

void CustomBackUp::onHalted() {
  stopRobot();
  RCLCPP_WARN(node_->get_logger(), "CustomBackUp: HALTED");
}

} // namespace rb1_bt

BT_REGISTER_NODES(factory) {
  factory.registerNodeType<rb1_bt::CustomBackUp>("CustomBackUp");
}