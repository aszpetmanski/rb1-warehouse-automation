#include "rb1_nav2_bt_nodes/compute_pre_dock_pose.hpp"

#include "rb1_nav2_bt_nodes/patrol_types.hpp"
#include "rb1_nav2_bt_nodes/shelf_scan_utils.hpp"

#include <behaviortree_cpp_v3/bt_factory.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <cmath>
#include <stdexcept>

namespace rb1_bt {

ComputePreDockPose::ComputePreDockPose(const std::string &name,
                                       const BT::NodeConfiguration &config)
    : BT::SyncActionNode(name, config) {}

bool ComputePreDockPose::initializeRosNode() {
  if (!node_) {
    config().blackboard->get<rclcpp::Node::SharedPtr>("node", node_);
    if (!node_) {
      throw std::runtime_error(
          "ComputePreDockPose: missing ROS node on blackboard");
    }
  }

  if (!tf_buffer_) {
    config().blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer",
                                                               tf_buffer_);
    if (!tf_buffer_) {
      throw std::runtime_error(
          "ComputePreDockPose: missing tf_buffer on blackboard");
    }
  }

  return true;
}

BT::PortsList ComputePreDockPose::providedPorts() {
  return {
      BT::InputPort<geometry_msgs::msg::Point>("shelf_left_leg",
                                               "Validated shelf left leg"),
      BT::InputPort<geometry_msgs::msg::Point>("shelf_right_leg",
                                               "Validated shelf right leg"),
      BT::InputPort<std::string>("target_frame", std::string("map"),
                                 "Frame for output predock pose"),
      BT::InputPort<double>("predock_offset", 0.60,
                            "Distance from shelf center along shelf normal"),

      BT::OutputPort<geometry_msgs::msg::PoseStamped>("predock_pose",
                                                      "Computed predock pose")};
}

BT::NodeStatus ComputePreDockPose::tick() {
  initializeRosNode();

  geometry_msgs::msg::Point shelf_left_leg;
  geometry_msgs::msg::Point shelf_right_leg;
  std::string target_frame = "map";
  double predock_offset = 0.60;

  if (!getInput("shelf_left_leg", shelf_left_leg)) {
    RCLCPP_ERROR(node_->get_logger(),
                 "ComputePreDockPose: missing input [shelf_left_leg]");
    return BT::NodeStatus::FAILURE;
  }

  if (!getInput("shelf_right_leg", shelf_right_leg)) {
    RCLCPP_ERROR(node_->get_logger(),
                 "ComputePreDockPose: missing input [shelf_right_leg]");
    return BT::NodeStatus::FAILURE;
  }

  getInput("target_frame", target_frame);
  getInput("predock_offset", predock_offset);

  if (predock_offset <= 0.0) {
    RCLCPP_ERROR(node_->get_logger(),
                 "ComputePreDockPose: predock_offset must be > 0.0, got %.3f",
                 predock_offset);
    return BT::NodeStatus::FAILURE;
  }

  const auto shelf_center =
      scan_utils::midpointGeometryPoints(shelf_left_leg, shelf_right_leg);

  const double vx = shelf_right_leg.x - shelf_left_leg.x;
  const double vy = shelf_right_leg.y - shelf_left_leg.y;
  const double norm = std::hypot(vx, vy);

  if (norm < 1e-6) {
    RCLCPP_ERROR(node_->get_logger(),
                 "ComputePreDockPose: shelf legs too close, cannot compute "
                 "shelf normal");
    return BT::NodeStatus::FAILURE;
  }

  const double tx = vx / norm;
  const double ty = vy / norm;

  const double nx = -ty;
  const double ny = tx;

  geometry_msgs::msg::Point candidate_a;
  candidate_a.x = shelf_center.x + predock_offset * nx;
  candidate_a.y = shelf_center.y + predock_offset * ny;
  candidate_a.z = shelf_center.z;

  geometry_msgs::msg::Point candidate_b;
  candidate_b.x = shelf_center.x - predock_offset * nx;
  candidate_b.y = shelf_center.y - predock_offset * ny;
  candidate_b.z = shelf_center.z;

  SimplePose2D robot_pose;
  if (!scan_utils::getRobotPoseInFrame(*tf_buffer_, target_frame, robot_pose,
                                       node_->get_logger())) {
    RCLCPP_ERROR(node_->get_logger(),
                 "ComputePreDockPose: failed to get robot pose in '%s'",
                 target_frame.c_str());
    return BT::NodeStatus::FAILURE;
  }

  const double dist_a = scan_utils::distance2D(candidate_a, robot_pose);
  const double dist_b = scan_utils::distance2D(candidate_b, robot_pose);

  const geometry_msgs::msg::Point chosen =
      (dist_a <= dist_b) ? candidate_a : candidate_b;

  const double yaw =
      std::atan2(shelf_center.y - chosen.y, shelf_center.x - chosen.x);

  SimplePose2D predock_waypoint;
  predock_waypoint.x = chosen.x;
  predock_waypoint.y = chosen.y;
  predock_waypoint.yaw = yaw;

  const auto predock_pose = scan_utils::buildPoseStampedFromWaypoint(
      predock_waypoint, target_frame, node_->now());

  setOutput("predock_pose", predock_pose);

  RCLCPP_INFO(node_->get_logger(),
              "ComputePreDockPose: SUCCESS "
              "L=[%.3f, %.3f] C=[%.3f, %.3f] R=[%.3f, %.3f] "
              "robot=[%.3f, %.3f, yaw=%.3f] predock=[%.3f, %.3f] yaw=%.3f",
              shelf_left_leg.x, shelf_left_leg.y, shelf_center.x,
              shelf_center.y, shelf_right_leg.x, shelf_right_leg.y,
              robot_pose.x, robot_pose.y, robot_pose.yaw,
              predock_pose.pose.position.x, predock_pose.pose.position.y,
              tf2::getYaw(predock_pose.pose.orientation));

  return BT::NodeStatus::SUCCESS;
}

} // namespace rb1_bt

BT_REGISTER_NODES(factory) {
  factory.registerNodeType<rb1_bt::ComputePreDockPose>("ComputePreDockPose");
}