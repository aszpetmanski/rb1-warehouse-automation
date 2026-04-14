#pragma once

#include <behaviortree_cpp_v3/action_node.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/buffer.h>

namespace rb1_bt {

class PlaceShelf : public BT::SyncActionNode {
public:
  PlaceShelf(const std::string &name, const BT::NodeConfiguration &config);

  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;

private:
  bool initializeRosNode();
  void ensureInterfaces(const std::string &cmd_vel_topic,
                        const std::string &elevator_down_topic);
  void publishStop() const;

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr elevator_down_pub_;

  std::string cmd_vel_topic_;
  std::string elevator_down_topic_;
};

} // namespace rb1_bt