#pragma once

#include "rb1_nav2_bt_nodes/patrol_types.hpp"
#include <behaviortree_cpp_v3/action_node.h>
#include <behaviortree_cpp_v3/bt_factory.h>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <string>

namespace rb1_bt {

/**
 * @brief Dummy node odstawiania / opuszczania shelfa.
 *
 * Aktualnie tylko loguje i zwraca SUCCESS.
 * Docelowo tu będzie opuszczenie windy i potwierdzenie zakończenia operacji.
 */
class PlaceShelf : public BT::SyncActionNode {
public:
  PlaceShelf(const std::string &name, const BT::NodeConfiguration &config);

  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
  bool initializeRosNode();

private:
  rclcpp::Node::SharedPtr node_;
};

/**
 * @brief Dummy node zatrzymania robota.
 *
 * Na razie tylko loguje i zwraca SUCCESS.
 * Docelowo można tu wpiąć publikację zerowego Twist albo service do stopu.
 */
class StopRobot : public BT::SyncActionNode {
public:
  StopRobot(const std::string &name, const BT::NodeConfiguration &config);

  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
  bool initializeRosNode();

private:
  rclcpp::Node::SharedPtr node_;
};

} // namespace rb1_bt