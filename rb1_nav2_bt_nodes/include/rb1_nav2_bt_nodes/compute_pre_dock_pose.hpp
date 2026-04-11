#pragma once

#include <behaviortree_cpp_v3/action_node.h>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>

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
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
};

} // namespace rb1_bt