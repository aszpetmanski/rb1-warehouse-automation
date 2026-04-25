#pragma once

#include <behaviortree_cpp_v3/action_node.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>

namespace rb1_bt {

class AdjustToPoseLocal : public BT::SyncActionNode {
public:
  AdjustToPoseLocal(const std::string &name,
                    const BT::NodeConfiguration &config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  void initializeRosNode();
  void publishStop() const;

  // ROS
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

  std::string cmd_vel_topic_;
  std::string robot_base_frame_;
};

} // namespace rb1_bt