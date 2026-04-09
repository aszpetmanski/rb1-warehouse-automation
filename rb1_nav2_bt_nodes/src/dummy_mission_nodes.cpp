#include "rb1_nav2_bt_nodes/dummy_mission_nodes.hpp"

#include <behaviortree_cpp_v3/bt_factory.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <stdexcept>

namespace rb1_bt {

ValidateShelfCandidate::ValidateShelfCandidate(
    const std::string &name, const BT::NodeConfiguration &config)
    : BT::StatefulActionNode(name, config) {}

bool ValidateShelfCandidate::initializeRosNode() {
  if (!node_) {
    config().blackboard->get<rclcpp::Node::SharedPtr>("node", node_);
    if (!node_) {
      throw std::runtime_error(
          "ValidateShelfCandidate: missing ROS node on blackboard");
    }
  }

  return true;
}

BT::PortsList ValidateShelfCandidate::providedPorts() {
  return {BT::InputPort<geometry_msgs::msg::Point>(
              "candidate_center", "Detected shelf candidate center"),
          BT::InputPort<geometry_msgs::msg::Point>(
              "candidate_left_leg", "Detected shelf candidate left leg"),
          BT::InputPort<geometry_msgs::msg::Point>(
              "candidate_right_leg", "Detected shelf candidate right leg"),
          BT::InputPort<double>("wait_duration", 5.0,
                                "Seconds to wait in dummy validation"),

          BT::OutputPort<geometry_msgs::msg::Point>("shelf_center",
                                                    "Validated shelf center"),
          BT::OutputPort<geometry_msgs::msg::Point>("shelf_left_leg",
                                                    "Validated shelf left leg"),
          BT::OutputPort<geometry_msgs::msg::Point>(
              "shelf_right_leg", "Validated shelf right leg")};
}

BT::NodeStatus ValidateShelfCandidate::onStart() {
  initializeRosNode();

  geometry_msgs::msg::Point center;
  geometry_msgs::msg::Point left_leg;
  geometry_msgs::msg::Point right_leg;

  if (!getInput("candidate_center", center)) {
    RCLCPP_ERROR(node_->get_logger(),
                 "ValidateShelfCandidate: missing input [candidate_center]");
    return BT::NodeStatus::FAILURE;
  }

  if (!getInput("candidate_left_leg", left_leg)) {
    RCLCPP_ERROR(node_->get_logger(),
                 "ValidateShelfCandidate: missing input [candidate_left_leg]");
    return BT::NodeStatus::FAILURE;
  }

  if (!getInput("candidate_right_leg", right_leg)) {
    RCLCPP_ERROR(node_->get_logger(),
                 "ValidateShelfCandidate: missing input [candidate_right_leg]");
    return BT::NodeStatus::FAILURE;
  }

  getInput("wait_duration", wait_duration_sec_);

  setOutput("shelf_center", center);
  setOutput("shelf_left_leg", left_leg);
  setOutput("shelf_right_leg", right_leg);

  start_time_ = node_->now();

  RCLCPP_INFO(
      node_->get_logger(),
      "ValidateShelfCandidate: dummy validation started, waiting %.2f sec",
      wait_duration_sec_);

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus ValidateShelfCandidate::onRunning() {
  const double elapsed = (node_->now() - start_time_).seconds();

  if (elapsed >= wait_duration_sec_) {
    RCLCPP_INFO(node_->get_logger(),
                "ValidateShelfCandidate: dummy validation SUCCESS");
    return BT::NodeStatus::SUCCESS;
  }

  return BT::NodeStatus::RUNNING;
}

void ValidateShelfCandidate::onHalted() {
  RCLCPP_INFO(node_->get_logger(), "ValidateShelfCandidate: halted");
}

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

  return true;
}

BT::PortsList ComputePreDockPose::providedPorts() {
  return {BT::InputPort<geometry_msgs::msg::Point>("shelf_center",
                                                   "Validated shelf center"),
          BT::InputPort<std::string>("target_frame", std::string("map"),
                                     "Frame for output poses"),

          BT::OutputPort<geometry_msgs::msg::PoseStamped>("predock_pose",
                                                          "Dummy predock pose"),
          BT::OutputPort<geometry_msgs::msg::PoseStamped>("dock_pose",
                                                          "Dummy dock pose")};
}

BT::NodeStatus ComputePreDockPose::tick() {
  initializeRosNode();

  geometry_msgs::msg::Point shelf_center;
  std::string target_frame = "map";

  if (!getInput("shelf_center", shelf_center)) {
    RCLCPP_ERROR(node_->get_logger(),
                 "ComputePreDockPose: missing input [shelf_center]");
    return BT::NodeStatus::FAILURE;
  }

  getInput("target_frame", target_frame);

  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = target_frame;
  pose.header.stamp = node_->now();
  pose.pose.position = shelf_center;

  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, 0.0);
  pose.pose.orientation = tf2::toMsg(q);

  setOutput("predock_pose", pose);
  setOutput("dock_pose", pose);

  RCLCPP_INFO(node_->get_logger(),
              "ComputePreDockPose: dummy SUCCESS, predock/dock pose set to "
              "shelf center [%.3f, %.3f]",
              shelf_center.x, shelf_center.y);

  return BT::NodeStatus::SUCCESS;
}

DockToShelf::DockToShelf(const std::string &name,
                         const BT::NodeConfiguration &config)
    : BT::StatefulActionNode(name, config) {}

bool DockToShelf::initializeRosNode() {
  if (!node_) {
    config().blackboard->get<rclcpp::Node::SharedPtr>("node", node_);
    if (!node_) {
      throw std::runtime_error("DockToShelf: missing ROS node on blackboard");
    }
  }

  return true;
}

BT::PortsList DockToShelf::providedPorts() {
  return {BT::InputPort<double>("wait_duration", 5.0,
                                "Seconds to wait in dummy docking")};
}

BT::NodeStatus DockToShelf::onStart() {
  initializeRosNode();

  getInput("wait_duration", wait_duration_sec_);
  start_time_ = node_->now();

  RCLCPP_INFO(node_->get_logger(),
              "DockToShelf: DOCKUJE... dummy wait %.2f sec",
              wait_duration_sec_);

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus DockToShelf::onRunning() {
  const double elapsed = (node_->now() - start_time_).seconds();

  if (elapsed >= wait_duration_sec_) {
    RCLCPP_INFO(node_->get_logger(), "DockToShelf: dummy docking SUCCESS");
    return BT::NodeStatus::SUCCESS;
  }

  return BT::NodeStatus::RUNNING;
}

void DockToShelf::onHalted() {
  RCLCPP_INFO(node_->get_logger(), "DockToShelf: halted");
}

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
      BT::InputPort<std::string>("target_frame", std::string("map"),
                                 "Frame for generated poses"),

      BT::OutputPort<std::vector<SimplePose2D>>(
          "patrol_waypoints", "Patrol route copied to blackboard"),
      BT::OutputPort<geometry_msgs::msg::PoseStamped>(
          "dropoff_pose", "Dropoff pose for NavigateRobustly"),

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

  getInput("target_frame", target_frame);

  if (patrol_waypoints.empty()) {
    RCLCPP_ERROR(node_->get_logger(),
                 "InitShelfMission: patrol_waypoints_input is empty");
    return BT::NodeStatus::FAILURE;
  }

  const auto dropoff_pose =
      buildPoseStampedFromWaypoint(dropoff_waypoint, target_frame);

  // Właściwe dane misji
  setOutput("patrol_waypoints", patrol_waypoints);
  setOutput("dropoff_pose", dropoff_pose);

  // Czyścimy/zerujemy resztę blackboardu, żeby nie zostały stare śmieci
  geometry_msgs::msg::Point zero_point;
  zero_point.x = 0.0;
  zero_point.y = 0.0;
  zero_point.z = 0.0;

  geometry_msgs::msg::PoseStamped zero_pose;
  zero_pose.header.frame_id = target_frame;
  zero_pose.header.stamp = node_->now();
  zero_pose.pose.position = zero_point;
  zero_pose.pose.orientation.w = 1.0;

  setOutput("candidate_center", zero_point);
  setOutput("candidate_left_leg", zero_point);
  setOutput("candidate_right_leg", zero_point);

  setOutput("shelf_center", zero_point);
  setOutput("shelf_left_leg", zero_point);
  setOutput("shelf_right_leg", zero_point);

  setOutput("predock_pose", zero_pose);
  setOutput("dock_pose", zero_pose);

  RCLCPP_INFO(node_->get_logger(),
              "InitShelfMission: initialized mission with %zu patrol "
              "waypoints, dropoff=(%.3f, %.3f, %.3f)",
              patrol_waypoints.size(), dropoff_waypoint.x, dropoff_waypoint.y,
              dropoff_waypoint.yaw);

  return BT::NodeStatus::SUCCESS;
}

geometry_msgs::msg::PoseStamped InitShelfMission::buildPoseStampedFromWaypoint(
    const SimplePose2D &waypoint, const std::string &target_frame) const {
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = target_frame;
  pose.header.stamp = node_->now();

  pose.pose.position.x = waypoint.x;
  pose.pose.position.y = waypoint.y;
  pose.pose.position.z = 0.0;

  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, waypoint.yaw);
  pose.pose.orientation = tf2::toMsg(q);

  return pose;
}

LiftShelf::LiftShelf(const std::string &name,
                     const BT::NodeConfiguration &config)
    : BT::SyncActionNode(name, config) {}

bool LiftShelf::initializeRosNode() {
  if (!node_) {
    config().blackboard->get<rclcpp::Node::SharedPtr>("node", node_);
    if (!node_) {
      throw std::runtime_error("LiftShelf: missing ROS node on blackboard");
    }
  }

  return true;
}

BT::PortsList LiftShelf::providedPorts() { return {}; }

BT::NodeStatus LiftShelf::tick() {
  initializeRosNode();

  RCLCPP_INFO(node_->get_logger(), "LiftShelf: dummy SUCCESS");
  return BT::NodeStatus::SUCCESS;
}

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

  return true;
}

BT::PortsList PlaceShelf::providedPorts() { return {}; }

BT::NodeStatus PlaceShelf::tick() {
  initializeRosNode();

  RCLCPP_INFO(node_->get_logger(), "PlaceShelf: dummy SUCCESS");
  return BT::NodeStatus::SUCCESS;
}

StopRobot::StopRobot(const std::string &name,
                     const BT::NodeConfiguration &config)
    : BT::SyncActionNode(name, config) {}

bool StopRobot::initializeRosNode() {
  if (!node_) {
    config().blackboard->get<rclcpp::Node::SharedPtr>("node", node_);
    if (!node_) {
      throw std::runtime_error("StopRobot: missing ROS node on blackboard");
    }
  }

  return true;
}

BT::PortsList StopRobot::providedPorts() { return {}; }

BT::NodeStatus StopRobot::tick() {
  initializeRosNode();

  RCLCPP_INFO(node_->get_logger(), "StopRobot: dummy SUCCESS");
  return BT::NodeStatus::SUCCESS;
}

} // namespace rb1_bt

BT_REGISTER_NODES(factory) {
  factory.registerNodeType<rb1_bt::ValidateShelfCandidate>(
      "ValidateShelfCandidate");
  factory.registerNodeType<rb1_bt::ComputePreDockPose>("ComputePreDockPose");
  factory.registerNodeType<rb1_bt::DockToShelf>("DockToShelf");
  factory.registerNodeType<rb1_bt::InitShelfMission>("InitShelfMission");
  factory.registerNodeType<rb1_bt::LiftShelf>("LiftShelf");
  factory.registerNodeType<rb1_bt::PlaceShelf>("PlaceShelf");
  factory.registerNodeType<rb1_bt::StopRobot>("StopRobot");
}