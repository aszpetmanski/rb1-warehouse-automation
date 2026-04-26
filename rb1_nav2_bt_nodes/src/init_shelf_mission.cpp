#include "rb1_nav2_bt_nodes/init_shelf_mission.hpp"
#include "rb1_nav2_bt_nodes/shelf_scan_utils.hpp"

#include <behaviortree_cpp_v3/bt_factory.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <algorithm>
#include <cctype>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace rb1_bt {

namespace {

std::string trim(const std::string &input) {
  auto first =
      std::find_if_not(input.begin(), input.end(),
                       [](unsigned char c) { return std::isspace(c); });

  auto last =
      std::find_if_not(input.rbegin(), input.rend(), [](unsigned char c) {
        return std::isspace(c);
      }).base();

  if (first >= last) {
    return "";
  }

  return std::string(first, last);
}

bool parseSimplePose2DString(const std::string &text, SimplePose2D &pose,
                             std::string &error) {
  std::stringstream ss(text);
  std::string token;
  std::vector<std::string> tokens;

  while (std::getline(ss, token, ',')) {
    tokens.push_back(trim(token));
  }

  if (tokens.size() != 3) {
    error = "expected x,y,yaw but got [" + text + "]";
    return false;
  }

  try {
    pose.x = std::stod(tokens[0]);
    pose.y = std::stod(tokens[1]);
    pose.yaw = std::stod(tokens[2]);
  } catch (const std::exception &e) {
    error = "failed to parse x,y,yaw from [" + text + "]: " + e.what();
    return false;
  }

  return true;
}

bool readDropoffWaypointRosParameter(const rclcpp::Node::SharedPtr &node,
                                     SimplePose2D &dropoff_waypoint,
                                     std::string &raw_value,
                                     std::string &error) {
  raw_value.clear();

  rclcpp::Parameter parameter;

  if (!node->get_parameter("dropoff_waypoint", parameter)) {
    error = "ROS parameter [dropoff_waypoint] is not declared or not set";
    return false;
  }

  if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_STRING) {
    error = "ROS parameter [dropoff_waypoint] is not a string";
    return false;
  }

  raw_value = trim(parameter.as_string());

  if (raw_value.empty()) {
    error = "ROS parameter [dropoff_waypoint] is empty";
    return false;
  }

  return parseSimplePose2DString(raw_value, dropoff_waypoint, error);
}

} // namespace

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

  std::string dropoff_source = "BT XML input";
  std::string raw_dropoff_parameter;
  std::string dropoff_parameter_error;

  const bool got_dropoff_from_parameter = readDropoffWaypointRosParameter(
      node_, dropoff_waypoint, raw_dropoff_parameter, dropoff_parameter_error);

  if (got_dropoff_from_parameter) {
    dropoff_source = "ROS parameter dropoff_waypoint";

    RCLCPP_INFO(
        node_->get_logger(),
        "InitShelfMission: using dropoff waypoint from ROS parameter: %s",
        raw_dropoff_parameter.c_str());
  } else {
    RCLCPP_INFO(node_->get_logger(),
                "InitShelfMission: ROS parameter dropoff_waypoint not used: %s",
                dropoff_parameter_error.c_str());

    if (!getInput("dropoff_waypoint", dropoff_waypoint)) {
      RCLCPP_ERROR(
          node_->get_logger(),
          "InitShelfMission: missing input [dropoff_waypoint] and no valid "
          "ROS parameter [dropoff_waypoint] was available");
      return BT::NodeStatus::FAILURE;
    }
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
      "dropoff=(%.3f, %.3f, %.3f) from %s, init=(%.3f, %.3f, %.3f)",
      patrol_waypoints.size(), dropoff_waypoint.x, dropoff_waypoint.y,
      dropoff_waypoint.yaw, dropoff_source.c_str(), init_waypoint.x,
      init_waypoint.y, init_waypoint.yaw);

  return BT::NodeStatus::SUCCESS;
}

} // namespace rb1_bt

BT_REGISTER_NODES(factory) {
  factory.registerNodeType<rb1_bt::InitShelfMission>("InitShelfMission");
}