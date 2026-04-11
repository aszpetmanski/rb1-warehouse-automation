#include "rb1_nav2_bt_nodes/dummy_mission_nodes.hpp"

#include <behaviortree_cpp_v3/bt_factory.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <stdexcept>

namespace rb1_bt {

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
  factory.registerNodeType<rb1_bt::DockToShelf>("DockToShelf");
  factory.registerNodeType<rb1_bt::LiftShelf>("LiftShelf");
  factory.registerNodeType<rb1_bt::PlaceShelf>("PlaceShelf");
  factory.registerNodeType<rb1_bt::StopRobot>("StopRobot");
}