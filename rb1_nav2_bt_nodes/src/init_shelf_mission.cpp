#include "rb1_nav2_bt_nodes/init_shelf_mission.hpp"
#include "rb1_nav2_bt_nodes/shelf_scan_utils.hpp"

#include <behaviortree_cpp_v3/bt_factory.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <stdexcept>
#include <string>
#include <vector>

namespace rb1_bt {

InitShelfMission::InitShelfMission(const std::string &name,
                                   const BT::NodeConfiguration &config)
    : BT::SyncActionNode(name, config) {}

bool InitShelfMission::initializeRosNode() {
  if (!node_) {
    config().blackboard->get<rclcpp::Node::SharedPtr>("node", node_);
    if (!node_) {
      throw std::runtime_error(
          "InitShelfMission: missing ROS node on blackboard");
    }
  }
  return true;
}

BT::PortsList InitShelfMission::providedPorts() {
  return {
      BT::InputPort<std::vector<SimplePose2D>>("patrol_waypoints_input",
                                               "Patrol route as x,y,yaw; ..."),
      BT::InputPort<SimplePose2D>("dropoff_waypoint",
                                  "Dropoff waypoint as x,y,yaw"),
      BT::InputPort<SimplePose2D>("init_waypoint",
                                  "Initial/home waypoint as x,y,yaw"),
      BT::InputPort<std::string>("target_frame", std::string("map"),
                                 "Frame for generated poses"),

      BT::OutputPort<std::vector<SimplePose2D>>(
          "patrol_waypoints", "Patrol route copied to blackboard"),
      BT::OutputPort<geometry_msgs::msg::PoseStamped>(
          "dropoff_pose", "Dropoff pose for navigation"),
      BT::OutputPort<geometry_msgs::msg::PoseStamped>(
          "init_pose", "Initial/home pose for navigation"),

      BT::OutputPort<geometry_msgs::msg::Point>("candidate_center",
                                                "Initialized candidate center"),
      BT::OutputPort<geometry_msgs::msg::Point>(
          "candidate_left_leg", "Initialized candidate left leg"),
      BT::OutputPort<geometry_msgs::msg::Point>(
          "candidate_right_leg", "Initialized candidate right leg"),

      BT::OutputPort<geometry_msgs::msg::Point>("shelf_center",
                                                "Initialized shelf center"),
      BT::OutputPort<geometry_msgs::msg::Point>("shelf_left_leg",
                                                "Initialized shelf left leg"),
      BT::OutputPort<geometry_msgs::msg::Point>("shelf_right_leg",
                                                "Initialized shelf right leg"),

      BT::OutputPort<geometry_msgs::msg::PoseStamped>(
          "predock_pose", "Initialized predock pose"),
      BT::OutputPort<geometry_msgs::msg::PoseStamped>("dock_pose",
                                                      "Initialized dock pose")};
}

BT::NodeStatus InitShelfMission::tick() {
  initializeRosNode();

  std::vector<SimplePose2D> patrol_waypoints;
  SimplePose2D dropoff_waypoint;
  SimplePose2D init_waypoint;
  std::string target_frame = "map";

  if (!getInput("patrol_waypoints_input", patrol_waypoints)) {
    RCLCPP_ERROR(node_->get_logger(),
                 "InitShelfMission: missing input [patrol_waypoints_input]");
    return BT::NodeStatus::FAILURE;
  }

  if (!getInput("dropoff_waypoint", dropoff_waypoint)) {
    RCLCPP_ERROR(node_->get_logger(),
                 "InitShelfMission: missing input [dropoff_waypoint]");
    return BT::NodeStatus::FAILURE;
  }

  if (!getInput("init_waypoint", init_waypoint)) {
    RCLCPP_ERROR(node_->get_logger(),
                 "InitShelfMission: missing input [init_waypoint]");
    return BT::NodeStatus::FAILURE;
  }

  getInput("target_frame", target_frame);

  if (patrol_waypoints.empty()) {
    RCLCPP_ERROR(node_->get_logger(),
                 "InitShelfMission: patrol_waypoints_input is empty");
    return BT::NodeStatus::FAILURE;
  }

  const auto dropoff_pose = scan_utils::buildPoseStampedFromWaypoint(
      dropoff_waypoint, target_frame, node_->now());

  const auto init_pose = scan_utils::buildPoseStampedFromWaypoint(
      init_waypoint, target_frame, node_->now());

  setOutput("patrol_waypoints", patrol_waypoints);
  setOutput("dropoff_pose", dropoff_pose);
  setOutput("init_pose", init_pose);

  geometry_msgs::msg::Point zero_point;
  zero_point.x = 0.0;
  zero_point.y = 0.0;
  zero_point.z = 0.0;

  geometry_msgs::msg::PoseStamped zero_pose;
  zero_pose.header.frame_id = target_frame;
  zero_pose.header.stamp = node_->now();
  zero_pose.pose.position = zero_point;
  zero_pose.pose.orientation.x = 0.0;
  zero_pose.pose.orientation.y = 0.0;
  zero_pose.pose.orientation.z = 0.0;
  zero_pose.pose.orientation.w = 1.0;

  setOutput("candidate_center", zero_point);
  setOutput("candidate_left_leg", zero_point);
  setOutput("candidate_right_leg", zero_point);

  setOutput("shelf_center", zero_point);
  setOutput("shelf_left_leg", zero_point);
  setOutput("shelf_right_leg", zero_point);

  setOutput("predock_pose", zero_pose);
  setOutput("dock_pose", zero_pose);

  RCLCPP_INFO(
      node_->get_logger(),
      "InitShelfMission: initialized mission with %zu patrol waypoints, "
      "dropoff=(%.3f, %.3f, %.3f), init=(%.3f, %.3f, %.3f)",
      patrol_waypoints.size(), dropoff_waypoint.x, dropoff_waypoint.y,
      dropoff_waypoint.yaw, init_waypoint.x, init_waypoint.y,
      init_waypoint.yaw);

  return BT::NodeStatus::SUCCESS;
}

} // namespace rb1_bt

BT_REGISTER_NODES(factory) {
  factory.registerNodeType<rb1_bt::InitShelfMission>("InitShelfMission");
}