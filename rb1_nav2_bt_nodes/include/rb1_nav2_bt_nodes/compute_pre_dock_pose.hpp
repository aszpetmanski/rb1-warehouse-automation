#pragma once

#include <behaviortree_cpp_v3/action_node.h>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

#include "rb1_nav2_bt_nodes/patrol_types.hpp"

namespace rb1_bt {

class ComputePreDockPose : public BT::SyncActionNode {
public:
  ComputePreDockPose(const std::string &name,
                     const BT::NodeConfiguration &config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;
  bool initializeRosNode();

private:
  rclcpp::Node::SharedPtr node_;
};

} // namespace rb1_bt