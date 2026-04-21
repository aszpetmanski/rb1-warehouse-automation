#include "rb1_nav2_bt_nodes/dock_to_shelf.hpp"

#include "rb1_nav2_bt_nodes/shelf_scan_utils.hpp"

#include <behaviortree_cpp_v3/bt_factory.h>
#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <tf2/exceptions.h>
#include <tf2/time.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdexcept>

namespace rb1_bt {

DockToShelf::DockToShelf(const std::string &name,
                         const BT::NodeConfiguration &config)
    : BT::SyncActionNode(name, config) {}

BT::PortsList DockToShelf::providedPorts() {
  return {
      BT::InputPort<geometry_msgs::msg::Point>("shelf_center",
                                               "Validated shelf center"),
      BT::InputPort<std::string>("target_frame", std::string("map"),
                                 "Frame for docking geometry"),
      BT::InputPort<std::string>(
          "cmd_vel_topic",
          std::string("/diffbot_base_controller/cmd_vel_unstamped"),
          "Velocity command topic"),
      BT::InputPort<std::string>("elevator_up_topic",
                                 std::string("/elevator_up"),
                                 "Elevator up trigger topic"),
      BT::InputPort<std::string>("robot_base_frame",
                                 std::string("robot_base_link"),
                                 "Robot base frame used to compute heading"),
      BT::InputPort<double>("drive_distance", 0.95,
                            "How far to drive forward during docking [m]"),
      BT::InputPort<double>("forward_speed", 0.15,
                            "Forward speed during docking [m/s]"),
      BT::InputPort<double>("yaw_gain", 1.5, "Yaw correction gain"),
      BT::InputPort<double>("max_angular_speed", 0.35,
                            "Maximum angular speed [rad/s]"),
      BT::InputPort<double>("shelf_normal_yaw",
                            "Yaw of shelf approach normal in target frame"),
      BT::InputPort<double>("control_rate", 20.0, "Control loop rate [Hz]")};
}

void DockToShelf::initializeRosNode() {
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

  if (!cmd_vel_pub_ || cmd_vel_pub_->get_topic_name() != cmd_vel_topic_) {
    cmd_vel_pub_ =
        node_->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);
  }

  if (!elevator_pub_ || elevator_pub_->get_topic_name() != elevator_up_topic_) {
    elevator_pub_ =
        node_->create_publisher<std_msgs::msg::String>(elevator_up_topic_, 10);
  }
}

void DockToShelf::publishStop() const {
  if (!cmd_vel_pub_) {
    return;
  }

  geometry_msgs::msg::Twist cmd;
  cmd_vel_pub_->publish(cmd);
}

BT::NodeStatus DockToShelf::tick() {
  geometry_msgs::msg::Point shelf_center;
  std::string target_frame{"map"};
  double drive_distance{0.95};
  double forward_speed{0.15};
  double yaw_gain{1.5};
  double max_angular_speed{0.35};
  double control_rate{20.0};

  if (!getInput("shelf_center", shelf_center)) {
    auto logger =
        node_ ? node_->get_logger() : rclcpp::get_logger("DockToShelf");
    RCLCPP_ERROR(logger, "DockToShelf: missing input [shelf_center]");
    return BT::NodeStatus::FAILURE;
  }

  getInput("target_frame", target_frame);
  getInput("cmd_vel_topic", cmd_vel_topic_);
  getInput("elevator_up_topic", elevator_up_topic_);
  getInput("robot_base_frame", robot_base_frame_);
  getInput("drive_distance", drive_distance);
  getInput("forward_speed", forward_speed);
  getInput("yaw_gain", yaw_gain);
  getInput("max_angular_speed", max_angular_speed);
  getInput("control_rate", control_rate);

  try {
    initializeRosNode();
  } catch (const std::exception &ex) {
    RCLCPP_ERROR(rclcpp::get_logger("DockToShelf"),
                 "DockToShelf: initialization failed: %s", ex.what());
    return BT::NodeStatus::FAILURE;
  }

  if (drive_distance <= 0.0 || forward_speed <= 0.0 || control_rate <= 0.0 ||
      max_angular_speed < 0.0) {
    RCLCPP_ERROR(node_->get_logger(),
                 "DockToShelf: invalid params drive_distance=%.3f "
                 "forward_speed=%.3f control_rate=%.3f "
                 "max_angular_speed=%.3f",
                 drive_distance, forward_speed, control_rate,
                 max_angular_speed);
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
              "cmd_vel_topic=%s, elevator_up_topic=%s, robot_base_frame=%s, "
              "drive_distance=%.3f",
              shelf_center.x, shelf_center.y, cmd_vel_topic_.c_str(),
              elevator_up_topic_.c_str(), robot_base_frame_.c_str(),
              drive_distance);

  rclcpp::Rate rate(control_rate);
  const auto start_time = node_->now();
  constexpr double kMaxTimeSec = 25.0;
  constexpr int kElevatorTriggerCount = 4;
  constexpr auto kPostStopSettle = std::chrono::milliseconds(300);
  constexpr auto kElevatorTriggerInterval = std::chrono::milliseconds(150);

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

    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = forward_speed;

    geometry_msgs::msg::PointStamped base_origin;
    geometry_msgs::msg::PointStamped base_x_axis;
    geometry_msgs::msg::PointStamped base_origin_target;
    geometry_msgs::msg::PointStamped base_x_axis_target;

    base_origin.header.frame_id = robot_base_frame_;
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

    double shelf_normal_yaw = 0.0;
    if (!getInput("shelf_normal_yaw", shelf_normal_yaw)) {
      publishStop();
      RCLCPP_ERROR(node_->get_logger(),
                   "DockToShelf: missing input [shelf_normal_yaw]");
      return BT::NodeStatus::FAILURE;
    }

    double yaw_error = shelf_normal_yaw - robot_yaw;
    while (yaw_error > M_PI) {
      yaw_error -= 2.0 * M_PI;
    }
    while (yaw_error < -M_PI) {
      yaw_error += 2.0 * M_PI;
    }

    cmd.angular.z =
        std::clamp(yaw_gain * yaw_error, -max_angular_speed, max_angular_speed);

    cmd_vel_pub_->publish(cmd);

    if ((node_->now() - start_time).seconds() > kMaxTimeSec) {
      publishStop();
      RCLCPP_ERROR(node_->get_logger(), "DockToShelf: docking timeout");
      return BT::NodeStatus::FAILURE;
    }

    rate.sleep();
  }

  publishStop();
  rclcpp::sleep_for(kPostStopSettle);

  std_msgs::msg::String msg;
  for (int i = 0; i < kElevatorTriggerCount; ++i) {
    elevator_pub_->publish(msg);
    rclcpp::sleep_for(kElevatorTriggerInterval);
  }

  RCLCPP_INFO(node_->get_logger(),
              "DockToShelf: SUCCESS, elevator trigger sent %d times",
              kElevatorTriggerCount);

  return BT::NodeStatus::SUCCESS;
}

} // namespace rb1_bt

BT_REGISTER_NODES(factory) {
  factory.registerNodeType<rb1_bt::DockToShelf>("DockToShelf");
}