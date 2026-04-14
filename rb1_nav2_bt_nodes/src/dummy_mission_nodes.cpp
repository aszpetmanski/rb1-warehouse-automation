#include "rb1_nav2_bt_nodes/dummy_mission_nodes.hpp"

#include <behaviortree_cpp_v3/bt_factory.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <stdexcept>

namespace rb1_bt {

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
  factory.registerNodeType<rb1_bt::StopRobot>("StopRobot");
}