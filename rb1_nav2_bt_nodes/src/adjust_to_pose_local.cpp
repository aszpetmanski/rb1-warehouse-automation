#include "rb1_nav2_bt_nodes/adjust_to_pose_local.hpp"

#include <behaviortree_cpp_v3/bt_factory.h>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <tf2/exceptions.h>
#include <tf2/time.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace rb1_bt {

static double normalizeAngle(double a) {
  while (a > M_PI) {
    a -= 2.0 * M_PI;
  }
  while (a < -M_PI) {
    a += 2.0 * M_PI;
  }
  return a;
}

AdjustToPoseLocal::AdjustToPoseLocal(const std::string &name,
                                     const BT::NodeConfiguration &config)
    : BT::SyncActionNode(name, config) {}

BT::PortsList AdjustToPoseLocal::providedPorts() {
  return {
      BT::InputPort<geometry_msgs::msg::PoseStamped>("goal",
                                                     "Local target pose"),
      BT::InputPort<std::string>("target_frame", std::string("map"),
                                 "Frame used for control"),
      BT::InputPort<std::string>("cmd_vel_topic", std::string("/cmd_vel"),
                                 "Velocity command topic"),
      BT::InputPort<std::string>("robot_base_frame",
                                 std::string("robot_base_link"),
                                 "Robot base frame used to compute heading"),
      BT::InputPort<double>("xy_tolerance", 0.03, "Position tolerance [m]"),
      BT::InputPort<double>("yaw_tolerance", 0.03, "Yaw tolerance [rad]"),
      BT::InputPort<double>("max_linear_speed", 0.08,
                            "Maximum linear speed [m/s]"),
      BT::InputPort<double>("max_angular_speed", 0.35,
                            "Maximum angular speed [rad/s]"),
      BT::InputPort<double>("control_rate", 15.0, "Control loop rate [Hz]")};
}

void AdjustToPoseLocal::initializeRosNode() {
  if (!node_) {
    config().blackboard->get<rclcpp::Node::SharedPtr>("node", node_);
    if (!node_) {
      throw std::runtime_error(
          "AdjustToPoseLocal: missing ROS node on blackboard");
    }
  }

  if (!tf_buffer_) {
    config().blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer",
                                                               tf_buffer_);
    if (!tf_buffer_) {
      throw std::runtime_error(
          "AdjustToPoseLocal: missing tf_buffer on blackboard");
    }
  }

  if (!cmd_vel_pub_ || cmd_vel_pub_->get_topic_name() != cmd_vel_topic_) {
    cmd_vel_pub_ =
        node_->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);
  }
}

void AdjustToPoseLocal::publishStop() const {
  if (!cmd_vel_pub_) {
    return;
  }

  geometry_msgs::msg::Twist cmd;
  cmd_vel_pub_->publish(cmd);
}

BT::NodeStatus AdjustToPoseLocal::tick() {
  geometry_msgs::msg::PoseStamped goal;
  std::string target_frame{"map"};
  double xy_tol{0.03};
  double yaw_tol{0.03};
  double max_lin{0.08};
  double max_ang{0.35};
  double rate_hz{15.0};

  if (!getInput("goal", goal)) {
    auto logger =
        node_ ? node_->get_logger() : rclcpp::get_logger("AdjustToPoseLocal");
    RCLCPP_ERROR(logger, "AdjustToPoseLocal: missing input [goal]");
    return BT::NodeStatus::FAILURE;
  }

  getInput("target_frame", target_frame);
  getInput("cmd_vel_topic", cmd_vel_topic_);
  getInput("robot_base_frame", robot_base_frame_);
  getInput("xy_tolerance", xy_tol);
  getInput("yaw_tolerance", yaw_tol);
  getInput("max_linear_speed", max_lin);
  getInput("max_angular_speed", max_ang);
  getInput("control_rate", rate_hz);

  try {
    initializeRosNode();
  } catch (const std::exception &ex) {
    RCLCPP_ERROR(rclcpp::get_logger("AdjustToPoseLocal"),
                 "AdjustToPoseLocal: initialization failed: %s", ex.what());
    return BT::NodeStatus::FAILURE;
  }

  if (xy_tol <= 0.0 || yaw_tol <= 0.0 || max_lin <= 0.0 || max_ang < 0.0 ||
      rate_hz <= 0.0) {
    RCLCPP_ERROR(node_->get_logger(),
                 "AdjustToPoseLocal: invalid params xy_tol=%.3f yaw_tol=%.3f "
                 "max_lin=%.3f max_ang=%.3f rate_hz=%.3f",
                 xy_tol, yaw_tol, max_lin, max_ang, rate_hz);
    return BT::NodeStatus::FAILURE;
  }

  if (!goal.header.frame_id.empty() && goal.header.frame_id != target_frame) {
    RCLCPP_WARN(
        node_->get_logger(),
        "AdjustToPoseLocal: goal frame is '%s' but target_frame is '%s'; "
        "assuming goal pose is already expressed in target_frame",
        goal.header.frame_id.c_str(), target_frame.c_str());
  }

  rclcpp::Rate rate(rate_hz);
  const auto start_time = node_->now();
  constexpr double kTimeoutSec = 20.0;

  tf2::Quaternion q;
  tf2::fromMsg(goal.pose.orientation, q);
  const double goal_yaw = tf2::getYaw(q);

  enum Phase { ROTATE_TO_GOAL, TRANSLATE, FINAL_ROTATE };
  Phase phase = ROTATE_TO_GOAL;

  RCLCPP_INFO(node_->get_logger(),
              "AdjustToPoseLocal: start goal=[%.3f, %.3f, %.3f], "
              "target_frame=%s, cmd_vel_topic=%s, robot_base_frame=%s",
              goal.pose.position.x, goal.pose.position.y, goal_yaw,
              target_frame.c_str(), cmd_vel_topic_.c_str(),
              robot_base_frame_.c_str());

  while (rclcpp::ok()) {
    geometry_msgs::msg::PointStamped origin;
    geometry_msgs::msg::PointStamped x_axis;
    geometry_msgs::msg::PointStamped origin_t;
    geometry_msgs::msg::PointStamped x_axis_t;

    origin.header.frame_id = robot_base_frame_;
    origin.header.stamp = builtin_interfaces::msg::Time();
    origin.point.x = 0.0;
    origin.point.y = 0.0;
    origin.point.z = 0.0;

    x_axis = origin;
    x_axis.point.x = 1.0;

    try {
      origin_t = tf_buffer_->transform(origin, target_frame,
                                       tf2::durationFromSec(0.1));
      x_axis_t = tf_buffer_->transform(x_axis, target_frame,
                                       tf2::durationFromSec(0.1));
    } catch (const tf2::TransformException &ex) {
      publishStop();
      RCLCPP_ERROR(node_->get_logger(),
                   "AdjustToPoseLocal: failed to get robot pose: %s",
                   ex.what());
      return BT::NodeStatus::FAILURE;
    }

    const double robot_x = origin_t.point.x;
    const double robot_y = origin_t.point.y;
    const double robot_yaw = std::atan2(x_axis_t.point.y - origin_t.point.y,
                                        x_axis_t.point.x - origin_t.point.x);

    const double dx = goal.pose.position.x - robot_x;
    const double dy = goal.pose.position.y - robot_y;

    const double dist = std::hypot(dx, dy);
    const double target_heading = std::atan2(dy, dx);
    const double heading_error = normalizeAngle(target_heading - robot_yaw);
    const double yaw_error = normalizeAngle(goal_yaw - robot_yaw);

    geometry_msgs::msg::Twist cmd;

    switch (phase) {
    case ROTATE_TO_GOAL:
      if (std::abs(heading_error) < yaw_tol) {
        phase = TRANSLATE;
        break;
      }
      cmd.angular.z = std::clamp(1.5 * heading_error, -max_ang, max_ang);
      break;

    case TRANSLATE:
      if (dist < xy_tol) {
        phase = FINAL_ROTATE;
        break;
      }
      cmd.linear.x = std::clamp(0.8 * dist, 0.02, max_lin);
      cmd.angular.z = std::clamp(1.0 * heading_error, -max_ang, max_ang);
      break;

    case FINAL_ROTATE:
      if (std::abs(yaw_error) < yaw_tol) {
        publishStop();
        RCLCPP_INFO(node_->get_logger(),
                    "AdjustToPoseLocal: SUCCESS, reached goal pose");
        return BT::NodeStatus::SUCCESS;
      }
      cmd.angular.z = std::clamp(1.5 * yaw_error, -max_ang, max_ang);
      break;
    }

    cmd_vel_pub_->publish(cmd);

    if ((node_->now() - start_time).seconds() > kTimeoutSec) {
      publishStop();
      RCLCPP_ERROR(node_->get_logger(), "AdjustToPoseLocal: timeout");
      return BT::NodeStatus::FAILURE;
    }

    rate.sleep();
  }

  publishStop();
  return BT::NodeStatus::FAILURE;
}

} // namespace rb1_bt

BT_REGISTER_NODES(factory) {
  factory.registerNodeType<rb1_bt::AdjustToPoseLocal>("AdjustToPoseLocal");
}