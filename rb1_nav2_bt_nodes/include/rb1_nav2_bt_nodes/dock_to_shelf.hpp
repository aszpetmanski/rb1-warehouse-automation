#pragma once

#include <behaviortree_cpp_v3/action_node.h>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/buffer.h>

namespace rb1_bt {

class DockToShelf : public BT::SyncActionNode {
public:
  DockToShelf(const std::string &name, const BT::NodeConfiguration &config);

  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;

private:
  void initializeRosNode();
  void publishStop() const;

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;

  std::string cmd_vel_topic_{"/diffbot_base_controller/cmd_vel_unstamped"};
  std::string elevator_up_topic_{"/elevator_up"};
  std::string robot_base_frame_{"robot_base_link"};

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr elevator_pub_;
};

} // namespace rb1_bt