#include <behaviortree_cpp_v3/bt_factory.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <stdexcept>

namespace rb1_bt {

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

} // namespace rb1_bt

BT_REGISTER_NODES(factory) {
  factory.registerNodeType<rb1_bt::ComputePreDockPose>("ComputePreDockPose");
}