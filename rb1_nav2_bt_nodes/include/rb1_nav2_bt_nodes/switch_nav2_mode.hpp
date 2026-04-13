#pragma once

#include <behaviortree_cpp_v3/action_node.h>
#include <rclcpp/rclcpp.hpp>

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace rb1_bt {

class Nav2ModeSwitchBase : public BT::SyncActionNode {
public:
  Nav2ModeSwitchBase(const std::string &name,
                     const BT::NodeConfiguration &config);

  static BT::PortsList commonPorts();

protected:
  void initializeRosNode();

  bool setRemoteParams(const std::string &remote_node,
                       const std::vector<rclcpp::Parameter> &params);

  bool clearCostmap(const std::string &service_name);

  static std::string makeLayerParam(const std::string &layer_name,
                                    const std::string &param_name);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Node::SharedPtr helper_node_;
  std::string helper_node_name_;
};

class SwitchToCarryMode : public Nav2ModeSwitchBase {
public:
  SwitchToCarryMode(const std::string &name,
                    const BT::NodeConfiguration &config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;
};

class SwitchToNormalMode : public Nav2ModeSwitchBase {
public:
  SwitchToNormalMode(const std::string &name,
                     const BT::NodeConfiguration &config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;
};

} // namespace rb1_bt