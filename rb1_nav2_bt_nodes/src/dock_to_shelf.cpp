#include "rb1_nav2_bt_nodes/dock_to_shelf.hpp"

#include "rb1_nav2_bt_nodes/shelf_scan_utils.hpp"

#include <behaviortree_cpp_v3/bt_factory.h>
#include <geometry_msgs/msg/point_stamped.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdexcept>

namespace rb1_bt {

DockToShelf::DockToShelf(const std::string &name,
                         const BT::NodeConfiguration &config)
    : BT::SyncActionNode(name, config) {}

bool DockToShelf::initializeRosNode() {
  if (!node_) {
    config().blackboard->get<rclcpp::Node::SharedPtr>("node", node_);
    if (!node_) {
      throw std::runtime_error("DockToShelf: missing ROS node on blackboard");
    }
  }

  if (!tf_buffer_) {
    config().blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer",
                                                               tf_buffer_);
    if (!tf_buffer_) {
      throw std::runtime_error("DockToShelf: missing tf_buffer on blackboard");
    }
  }

  if (!cmd_vel_pub_) {
    cmd_vel_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>(
        "/diffbot_base_controller/cmd_vel_unstamped", 10);
  }

  if (!elevator_pub_) {
    elevator_pub_ =
        node_->create_publisher<std_msgs::msg::String>("/elevator_up", 10);
  }

  return true;
}

BT::PortsList DockToShelf::providedPorts() {
  return {BT::InputPort<geometry_msgs::msg::Point>("shelf_center",
                                                   "Validated shelf center"),
          BT::InputPort<std::string>("target_frame", std::string("map"),
                                     "Frame for docking geometry"),
          BT::InputPort<double>("drive_distance", 0.95,
                                "How far to drive forward during docking [m]"),
          BT::InputPort<double>("forward_speed", 0.15,
                                "Forward speed during docking [m/s]"),
          BT::InputPort<double>("yaw_gain", 1.5, "Yaw correction gain"),
          BT::InputPort<double>("max_angular_speed", 0.35,
                                "Maximum angular speed [rad/s]"),
          BT::InputPort<double>("control_rate", 20.0, "Control loop rate [Hz]"),
          BT::InputPort<double>("straight_when_center_dist_below", 0.10,
                                "Latch straight drive when robot is this close "
                                "to shelf center [m]")};
}

void DockToShelf::publishStop() const {
  geometry_msgs::msg::Twist cmd;
  cmd_vel_pub_->publish(cmd);
}

BT::NodeStatus DockToShelf::tick() {
  initializeRosNode();

  geometry_msgs::msg::Point shelf_center;
  std::string target_frame = "map";
  double drive_distance = 0.95;
  double forward_speed = 0.15;
  double yaw_gain = 1.5;
  double max_angular_speed = 0.35;
  double control_rate = 20.0;
  double straight_when_center_dist_below = 0.10;

  if (!getInput("shelf_center", shelf_center)) {
    RCLCPP_ERROR(node_->get_logger(),
                 "DockToShelf: missing input [shelf_center]");
    return BT::NodeStatus::FAILURE;
  }

  getInput("target_frame", target_frame);
  getInput("drive_distance", drive_distance);
  getInput("forward_speed", forward_speed);
  getInput("yaw_gain", yaw_gain);
  getInput("max_angular_speed", max_angular_speed);
  getInput("control_rate", control_rate);
  getInput("straight_when_center_dist_below", straight_when_center_dist_below);

  if (drive_distance <= 0.0 || forward_speed <= 0.0 || control_rate <= 0.0 ||
      straight_when_center_dist_below < 0.0) {
    RCLCPP_ERROR(node_->get_logger(),
                 "DockToShelf: invalid params drive_distance=%.3f "
                 "forward_speed=%.3f control_rate=%.3f "
                 "straight_when_center_dist_below=%.3f",
                 drive_distance, forward_speed, control_rate,
                 straight_when_center_dist_below);
    return BT::NodeStatus::FAILURE;
  }

  SimplePose2D start_pose;
  if (!scan_utils::getRobotPoseInFrame(*tf_buffer_, target_frame, start_pose,
                                       node_->get_logger())) {
    RCLCPP_ERROR(node_->get_logger(),
                 "DockToShelf: failed to get robot start position");
    return BT::NodeStatus::FAILURE;
  }

  RCLCPP_INFO(node_->get_logger(),
              "DockToShelf: start docking toward shelf center [%.3f, %.3f], "
              "drive_distance=%.3f, straight_when_center_dist_below=%.3f",
              shelf_center.x, shelf_center.y, drive_distance,
              straight_when_center_dist_below);

  rclcpp::Rate rate(control_rate);
  const auto start_time = node_->now();
  const double max_time_sec = 25.0;

  bool straight_mode = false;

  while (rclcpp::ok()) {
    SimplePose2D robot_pose;
    if (!scan_utils::getRobotPoseInFrame(*tf_buffer_, target_frame, robot_pose,
                                         node_->get_logger())) {
      publishStop();
      RCLCPP_ERROR(node_->get_logger(),
                   "DockToShelf: failed to refresh robot position");
      return BT::NodeStatus::FAILURE;
    }

    const double traveled = scan_utils::distance2D(robot_pose, start_pose);

    if (traveled >= drive_distance) {
      break;
    }

    const double dist_to_center =
        scan_utils::distance2D(robot_pose, shelf_center);

    if (!straight_mode && dist_to_center <= straight_when_center_dist_below) {
      straight_mode = true;
      RCLCPP_INFO(node_->get_logger(),
                  "DockToShelf: switching to straight mode at "
                  "dist_to_center=%.3f m, traveled=%.3f m",
                  dist_to_center, traveled);
    }

    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = forward_speed;

    if (straight_mode) {
      cmd.angular.z = 0.0;
    } else {
      geometry_msgs::msg::PointStamped base_origin;
      geometry_msgs::msg::PointStamped base_x_axis;
      geometry_msgs::msg::PointStamped base_origin_target;
      geometry_msgs::msg::PointStamped base_x_axis_target;

      base_origin.header.frame_id = "robot_base_link";
      base_origin.header.stamp = builtin_interfaces::msg::Time();
      base_origin.point.x = 0.0;
      base_origin.point.y = 0.0;
      base_origin.point.z = 0.0;

      base_x_axis = base_origin;
      base_x_axis.point.x = 1.0;

      try {
        base_origin_target = tf_buffer_->transform(base_origin, target_frame,
                                                   tf2::durationFromSec(0.1));
        base_x_axis_target = tf_buffer_->transform(base_x_axis, target_frame,
                                                   tf2::durationFromSec(0.1));
      } catch (const tf2::TransformException &ex) {
        publishStop();
        RCLCPP_ERROR(node_->get_logger(),
                     "DockToShelf: failed to get robot heading: %s", ex.what());
        return BT::NodeStatus::FAILURE;
      }

      const double robot_yaw =
          std::atan2(base_x_axis_target.point.y - base_origin_target.point.y,
                     base_x_axis_target.point.x - base_origin_target.point.x);

      const double desired_yaw = std::atan2(shelf_center.y - robot_pose.y,
                                            shelf_center.x - robot_pose.x);

      double yaw_error = desired_yaw - robot_yaw;
      while (yaw_error > M_PI) {
        yaw_error -= 2.0 * M_PI;
      }
      while (yaw_error < -M_PI) {
        yaw_error += 2.0 * M_PI;
      }

      cmd.angular.z = std::clamp(yaw_gain * yaw_error, -max_angular_speed,
                                 max_angular_speed);
    }

    cmd_vel_pub_->publish(cmd);

    if ((node_->now() - start_time).seconds() > max_time_sec) {
      publishStop();
      RCLCPP_ERROR(node_->get_logger(), "DockToShelf: docking timeout");
      return BT::NodeStatus::FAILURE;
    }

    rate.sleep();
  }

  publishStop();
  rclcpp::sleep_for(std::chrono::milliseconds(300));

  std_msgs::msg::String s;
  for (int i = 0; i < 4; ++i) {
    elevator_pub_->publish(s);
    rclcpp::sleep_for(std::chrono::milliseconds(150));
  }

  RCLCPP_INFO(node_->get_logger(),
              "DockToShelf: SUCCESS, elevator trigger sent 4 times");

  return BT::NodeStatus::SUCCESS;
}

} // namespace rb1_bt

BT_REGISTER_NODES(factory) {
  factory.registerNodeType<rb1_bt::DockToShelf>("DockToShelf");
}