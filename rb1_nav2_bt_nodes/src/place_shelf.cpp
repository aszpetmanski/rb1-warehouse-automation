#include "rb1_nav2_bt_nodes/place_shelf.hpp"

#include "rb1_nav2_bt_nodes/shelf_scan_utils.hpp"

#include <behaviortree_cpp_v3/bt_factory.h>
#include <tf2/utils.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdexcept>

namespace rb1_bt {

PlaceShelf::PlaceShelf(const std::string &name,
                       const BT::NodeConfiguration &config)
    : BT::SyncActionNode(name, config) {}

bool PlaceShelf::initializeRosNode() {
  if (!node_) {
    config().blackboard->get<rclcpp::Node::SharedPtr>("node", node_);
    if (!node_) {
      throw std::runtime_error("PlaceShelf: missing ROS node on blackboard");
    }
  }

  if (!tf_buffer_) {
    config().blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer",
                                                               tf_buffer_);
    if (!tf_buffer_) {
      throw std::runtime_error("PlaceShelf: missing tf_buffer on blackboard");
    }
  }

  return true;
}

void PlaceShelf::ensureInterfaces(const std::string &cmd_vel_topic,
                                  const std::string &elevator_down_topic) {
  if (!cmd_vel_pub_ || cmd_vel_topic != cmd_vel_topic_) {
    cmd_vel_topic_ = cmd_vel_topic;
    cmd_vel_pub_ =
        node_->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);
  }

  if (!elevator_down_pub_ || elevator_down_topic != elevator_down_topic_) {
    elevator_down_topic_ = elevator_down_topic;
    elevator_down_pub_ = node_->create_publisher<std_msgs::msg::String>(
        elevator_down_topic_, 10);
  }
}

BT::PortsList PlaceShelf::providedPorts() {
  return {
      BT::InputPort<geometry_msgs::msg::PoseStamped>(
          "dropoff_pose", "Dropoff pose with the final desired yaw"),
      BT::InputPort<std::string>("target_frame", std::string("map"),
                                 "Frame used for control"),
      BT::InputPort<double>(
          "drive_distance", 0.35,
          "Distance to drive forward before lowering shelf [m]"),
      BT::InputPort<double>("forward_speed", 0.10,
                            "Forward speed while placing [m/s]"),
      BT::InputPort<double>("yaw_gain", 2.0, "Yaw correction gain"),
      BT::InputPort<double>("max_angular_speed", 0.35,
                            "Maximum angular speed [rad/s]"),
      BT::InputPort<double>("yaw_tolerance", 0.03,
                            "Yaw tolerance for initial alignment [rad]"),
      BT::InputPort<double>("control_rate", 20.0, "Control loop rate [Hz]"),
      BT::InputPort<double>("align_timeout", 8.0,
                            "Timeout for yaw alignment [s]"),
      BT::InputPort<double>("motion_timeout", 15.0,
                            "Timeout for forward placing motion [s]"),
      BT::InputPort<double>("stop_settle_time", 0.30,
                            "Pause after stop before next phase [s]"),
      BT::InputPort<int>("elevator_trigger_count", 4,
                         "How many times to publish the elevator-down trigger"),
      BT::InputPort<double>(
          "elevator_trigger_interval", 0.15,
          "Delay between elevator-down trigger publishes [s]"),
      BT::InputPort<std::string>("cmd_vel_topic",
                                 "/diffbot_base_controller/cmd_vel_unstamped",
                                 "cmd_vel topic"),
      BT::InputPort<std::string>("elevator_down_topic", "/elevator_down",
                                 "Elevator-down trigger topic")};
}

void PlaceShelf::publishStop() const {
  geometry_msgs::msg::Twist cmd;
  cmd_vel_pub_->publish(cmd);
}

BT::NodeStatus PlaceShelf::tick() {
  initializeRosNode();

  geometry_msgs::msg::PoseStamped dropoff_pose;
  std::string target_frame = "map";
  std::string cmd_vel_topic = "/diffbot_base_controller/cmd_vel_unstamped";
  std::string elevator_down_topic = "/elevator_down";

  double drive_distance = 0.35;
  double forward_speed = 0.10;
  double yaw_gain = 2.0;
  double max_angular_speed = 0.35;
  double yaw_tolerance = 0.03;
  double control_rate = 20.0;
  double align_timeout = 8.0;
  double motion_timeout = 15.0;
  double stop_settle_time = 0.30;
  int elevator_trigger_count = 4;
  double elevator_trigger_interval = 0.15;

  if (!getInput("dropoff_pose", dropoff_pose)) {
    RCLCPP_ERROR(node_->get_logger(),
                 "PlaceShelf: missing input [dropoff_pose]");
    return BT::NodeStatus::FAILURE;
  }

  getInput("target_frame", target_frame);
  getInput("cmd_vel_topic", cmd_vel_topic);
  getInput("elevator_down_topic", elevator_down_topic);
  getInput("drive_distance", drive_distance);
  getInput("forward_speed", forward_speed);
  getInput("yaw_gain", yaw_gain);
  getInput("max_angular_speed", max_angular_speed);
  getInput("yaw_tolerance", yaw_tolerance);
  getInput("control_rate", control_rate);
  getInput("align_timeout", align_timeout);
  getInput("motion_timeout", motion_timeout);
  getInput("stop_settle_time", stop_settle_time);
  getInput("elevator_trigger_count", elevator_trigger_count);
  getInput("elevator_trigger_interval", elevator_trigger_interval);

  ensureInterfaces(cmd_vel_topic, elevator_down_topic);

  if (dropoff_pose.header.frame_id.empty()) {
    dropoff_pose.header.frame_id = target_frame;
  }

  if (dropoff_pose.header.frame_id != target_frame) {
    RCLCPP_ERROR(node_->get_logger(),
                 "PlaceShelf: dropoff_pose frame [%s] != target_frame [%s]",
                 dropoff_pose.header.frame_id.c_str(), target_frame.c_str());
    return BT::NodeStatus::FAILURE;
  }

  if (drive_distance <= 0.0 || forward_speed <= 0.0 || control_rate <= 0.0 ||
      yaw_tolerance < 0.0 || align_timeout <= 0.0 || motion_timeout <= 0.0 ||
      elevator_trigger_count < 1 || elevator_trigger_interval < 0.0) {
    RCLCPP_ERROR(
        node_->get_logger(),
        "PlaceShelf: invalid parameters drive_distance=%.3f forward_speed=%.3f "
        "control_rate=%.3f yaw_tolerance=%.3f align_timeout=%.3f "
        "motion_timeout=%.3f elevator_trigger_count=%d "
        "elevator_trigger_interval=%.3f",
        drive_distance, forward_speed, control_rate, yaw_tolerance,
        align_timeout, motion_timeout, elevator_trigger_count,
        elevator_trigger_interval);
    return BT::NodeStatus::FAILURE;
  }

  const double desired_yaw = tf2::getYaw(dropoff_pose.pose.orientation);

  RCLCPP_INFO(node_->get_logger(),
              "PlaceShelf: START desired_yaw=%.3f rad, drive_distance=%.3f m",
              desired_yaw, drive_distance);

  rclcpp::Rate rate(control_rate);

  // Phase 1: align to exact dropoff yaw
  {
    const auto start_time = node_->now();

    while (rclcpp::ok()) {
      SimplePose2D robot_pose;
      if (!scan_utils::getRobotPoseInFrame(*tf_buffer_, target_frame,
                                           robot_pose, node_->get_logger())) {
        publishStop();
        return BT::NodeStatus::FAILURE;
      }

      const double yaw_error =
          scan_utils::normalizeAngle(desired_yaw - robot_pose.yaw);

      if (std::abs(yaw_error) <= yaw_tolerance) {
        break;
      }

      geometry_msgs::msg::Twist cmd;
      cmd.linear.x = 0.0;
      cmd.angular.z = std::clamp(yaw_gain * yaw_error, -max_angular_speed,
                                 max_angular_speed);
      cmd_vel_pub_->publish(cmd);

      if ((node_->now() - start_time).seconds() > align_timeout) {
        publishStop();
        RCLCPP_ERROR(node_->get_logger(), "PlaceShelf: yaw alignment timeout");
        return BT::NodeStatus::FAILURE;
      }

      rate.sleep();
    }

    publishStop();
    rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(std::max(0.0, stop_settle_time))));
  }

  // Phase 2: drive forward while holding yaw
  {
    SimplePose2D start_pose;
    if (!scan_utils::getRobotPoseInFrame(*tf_buffer_, target_frame, start_pose,
                                         node_->get_logger())) {
      publishStop();
      return BT::NodeStatus::FAILURE;
    }

    const auto start_time = node_->now();

    while (rclcpp::ok()) {
      SimplePose2D robot_pose;
      if (!scan_utils::getRobotPoseInFrame(*tf_buffer_, target_frame,
                                           robot_pose, node_->get_logger())) {
        publishStop();
        return BT::NodeStatus::FAILURE;
      }

      const double dx = robot_pose.x - start_pose.x;
      const double dy = robot_pose.y - start_pose.y;
      const double progress =
          dx * std::cos(desired_yaw) + dy * std::sin(desired_yaw);

      if (progress >= drive_distance) {
        break;
      }

      const double yaw_error =
          scan_utils::normalizeAngle(desired_yaw - robot_pose.yaw);

      geometry_msgs::msg::Twist cmd;
      cmd.linear.x = forward_speed;
      cmd.angular.z = std::clamp(yaw_gain * yaw_error, -max_angular_speed,
                                 max_angular_speed);
      cmd_vel_pub_->publish(cmd);

      if ((node_->now() - start_time).seconds() > motion_timeout) {
        publishStop();
        RCLCPP_ERROR(node_->get_logger(),
                     "PlaceShelf: forward motion timeout, progress=%.3f / %.3f",
                     progress, drive_distance);
        return BT::NodeStatus::FAILURE;
      }

      rate.sleep();
    }

    publishStop();
    rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(std::max(0.0, stop_settle_time))));
  }

  // Phase 3: lower elevator
  {
    std_msgs::msg::String msg;
    for (int i = 0; i < elevator_trigger_count; ++i) {
      elevator_down_pub_->publish(msg);
      rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::duration<double>(
              std::max(0.0, elevator_trigger_interval))));
    }
  }

  RCLCPP_INFO(node_->get_logger(),
              "PlaceShelf: SUCCESS, elevator_down sent %d times",
              elevator_trigger_count);

  return BT::NodeStatus::SUCCESS;
}

} // namespace rb1_bt

BT_REGISTER_NODES(factory) {
  factory.registerNodeType<rb1_bt::PlaceShelf>("PlaceShelf");
}