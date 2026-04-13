#include "rb1_nav2_bt_nodes/switch_nav2_mode.hpp"

#include <behaviortree_cpp_v3/bt_factory.h>
#include <nav2_msgs/srv/clear_entire_costmap.hpp>
#include <std_srvs/srv/empty.hpp>

#include <chrono>
#include <future>
#include <stdexcept>
#include <utility>

using namespace std::chrono_literals;

namespace rb1_bt {

Nav2ModeSwitchBase::Nav2ModeSwitchBase(const std::string &name,
                                       const BT::NodeConfiguration &config)
    : BT::SyncActionNode(name, config),
      helper_node_name_("bt_nav2_mode_switch_helper_" +
                        std::to_string(static_cast<unsigned long long>(
                            reinterpret_cast<std::uintptr_t>(this)))) {}

BT::PortsList Nav2ModeSwitchBase::commonPorts() {
  return {
      BT::InputPort<std::string>("local_costmap_node",
                                 "/local_costmap/local_costmap",
                                 "Remote node name of local costmap"),
      BT::InputPort<std::string>("global_costmap_node",
                                 "/global_costmap/global_costmap",
                                 "Remote node name of global costmap"),

      BT::InputPort<std::string>("local_clear_service",
                                 "/local_costmap/clear_entirely_local_costmap",
                                 "Service to clear local costmap"),
      BT::InputPort<std::string>(
          "global_clear_service",
          "/global_costmap/clear_entirely_global_costmap",
          "Service to clear global costmap"),

      BT::InputPort<std::string>("local_inflation_layer_name",
                                 "inflation_layer",
                                 "Plugin alias for local inflation layer"),
      BT::InputPort<std::string>("global_inflation_layer_name",
                                 "inflation_layer",
                                 "Plugin alias for global inflation layer"),

      BT::InputPort<std::string>(
          "local_clearing_layer_name", "obstacle_layer",
          "Plugin alias for local obstacle/clearing layer"),
      BT::InputPort<std::string>(
          "global_clearing_layer_name", "obstacle_layer",
          "Plugin alias for global obstacle/clearing layer"),

      BT::InputPort<double>("footprint_padding", 0.01, "Footprint padding [m]"),

      BT::InputPort<double>("local_inflation_radius", 0.55,
                            "Local inflation radius [m]"),
      BT::InputPort<double>("local_cost_scaling_factor", 10.0,
                            "Local inflation cost scaling factor"),

      BT::InputPort<double>("global_inflation_radius", 0.55,
                            "Global inflation radius [m]"),
      BT::InputPort<double>("global_cost_scaling_factor", 10.0,
                            "Global inflation cost scaling factor"),

      BT::InputPort<bool>("local_footprint_clearing_enabled", true,
                          "Clear local obstacles under footprint"),
      BT::InputPort<bool>("global_footprint_clearing_enabled", true,
                          "Clear global obstacles under footprint"),

      BT::InputPort<bool>("clear_costmaps", true,
                          "Clear both costmaps after switching mode"),

      BT::InputPort<double>("post_param_wait_sec", 0.20,
                            "Wait after parameter switch [s]"),
      BT::InputPort<double>("post_clear_wait_sec", 0.20,
                            "Wait after costmap clear [s]"),
  };
}

void Nav2ModeSwitchBase::initializeRosNode() {
  if (!node_) {
    config().blackboard->get<rclcpp::Node::SharedPtr>("node", node_);
    if (!node_) {
      throw std::runtime_error(
          "Nav2ModeSwitchBase: missing ROS node on blackboard");
    }
  }

  if (!helper_node_) {
    helper_node_ = std::make_shared<rclcpp::Node>(helper_node_name_);
  }
}

std::string Nav2ModeSwitchBase::makeLayerParam(const std::string &layer_name,
                                               const std::string &param_name) {
  if (layer_name.empty()) {
    return "";
  }
  return layer_name + "." + param_name;
}

bool Nav2ModeSwitchBase::setRemoteParams(
    const std::string &remote_node,
    const std::vector<rclcpp::Parameter> &params) {
  auto client = std::make_shared<rclcpp::AsyncParametersClient>(helper_node_,
                                                                remote_node);

  if (!client->wait_for_service(3s)) {
    RCLCPP_ERROR(helper_node_->get_logger(),
                 "Mode switch: parameter service unavailable for node [%s]",
                 remote_node.c_str());
    return false;
  }

  auto future = client->set_parameters(params);

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(helper_node_);

  const auto rc = exec.spin_until_future_complete(future, 5s);
  exec.remove_node(helper_node_);

  if (rc != rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(helper_node_->get_logger(),
                 "Mode switch: parameter update timed out for node [%s]",
                 remote_node.c_str());
    return false;
  }

  const auto results = future.get();
  if (results.size() != params.size()) {
    RCLCPP_ERROR(helper_node_->get_logger(),
                 "Mode switch: unexpected parameter result count for node [%s]",
                 remote_node.c_str());
    return false;
  }

  for (size_t i = 0; i < results.size(); ++i) {
    if (!results[i].successful) {
      RCLCPP_ERROR(helper_node_->get_logger(),
                   "Mode switch: rejected param [%s] on node [%s]: %s",
                   params[i].get_name().c_str(), remote_node.c_str(),
                   results[i].reason.c_str());
      return false;
    }
  }

  return true;
}

bool Nav2ModeSwitchBase::clearCostmap(const std::string &service_name) {
  auto client = helper_node_->create_client<nav2_msgs::srv::ClearEntireCostmap>(
      service_name);

  if (!client->wait_for_service(3s)) {
    RCLCPP_ERROR(helper_node_->get_logger(),
                 "Mode switch: clear service unavailable [%s]",
                 service_name.c_str());
    return false;
  }

  auto request =
      std::make_shared<nav2_msgs::srv::ClearEntireCostmap::Request>();
  auto future = client->async_send_request(request);

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(helper_node_);

  const auto rc = exec.spin_until_future_complete(future, 5s);
  exec.remove_node(helper_node_);

  if (rc != rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(helper_node_->get_logger(),
                 "Mode switch: clear service timed out [%s]",
                 service_name.c_str());
    return false;
  }

  return true;
}

// ========================= SwitchToCarryMode =========================

SwitchToCarryMode::SwitchToCarryMode(const std::string &name,
                                     const BT::NodeConfiguration &config)
    : Nav2ModeSwitchBase(name, config) {}

BT::PortsList SwitchToCarryMode::providedPorts() {
  auto ports = commonPorts();

  ports.insert(BT::InputPort<std::string>(
      "carry_footprint",
      "Polygon footprint string for carry mode, e.g. "
      "[[0.47,0.45],[0.47,-0.45],[-0.38,-0.45],[-0.38,0.45]]"));

  return ports;
}

BT::NodeStatus SwitchToCarryMode::tick() {
  initializeRosNode();

  std::string carry_footprint;
  std::string local_costmap_node;
  std::string global_costmap_node;
  std::string local_clear_service;
  std::string global_clear_service;
  std::string local_inflation_layer_name;
  std::string global_inflation_layer_name;
  std::string local_clearing_layer_name;
  std::string global_clearing_layer_name;

  double footprint_padding = 0.01;
  double local_inflation_radius = 0.45;
  double local_cost_scaling_factor = 10.0;
  double global_inflation_radius = 0.45;
  double global_cost_scaling_factor = 10.0;
  double post_param_wait_sec = 0.20;
  double post_clear_wait_sec = 0.20;

  bool local_footprint_clearing_enabled = true;
  bool global_footprint_clearing_enabled = true;
  bool clear_costmaps = true;

  if (!getInput("carry_footprint", carry_footprint) ||
      carry_footprint.empty()) {
    RCLCPP_ERROR(node_->get_logger(),
                 "SwitchToCarryMode: missing input [carry_footprint]");
    return BT::NodeStatus::FAILURE;
  }

  getInput("local_costmap_node", local_costmap_node);
  getInput("global_costmap_node", global_costmap_node);
  getInput("local_clear_service", local_clear_service);
  getInput("global_clear_service", global_clear_service);
  getInput("local_inflation_layer_name", local_inflation_layer_name);
  getInput("global_inflation_layer_name", global_inflation_layer_name);
  getInput("local_clearing_layer_name", local_clearing_layer_name);
  getInput("global_clearing_layer_name", global_clearing_layer_name);
  getInput("footprint_padding", footprint_padding);
  getInput("local_inflation_radius", local_inflation_radius);
  getInput("local_cost_scaling_factor", local_cost_scaling_factor);
  getInput("global_inflation_radius", global_inflation_radius);
  getInput("global_cost_scaling_factor", global_cost_scaling_factor);
  getInput("local_footprint_clearing_enabled",
           local_footprint_clearing_enabled);
  getInput("global_footprint_clearing_enabled",
           global_footprint_clearing_enabled);
  getInput("clear_costmaps", clear_costmaps);
  getInput("post_param_wait_sec", post_param_wait_sec);
  getInput("post_clear_wait_sec", post_clear_wait_sec);

  std::vector<rclcpp::Parameter> local_params;
  local_params.emplace_back("footprint", carry_footprint);
  local_params.emplace_back("footprint_padding", footprint_padding);

  const auto local_inflation_radius_param =
      makeLayerParam(local_inflation_layer_name, "inflation_radius");
  const auto local_cost_scaling_param =
      makeLayerParam(local_inflation_layer_name, "cost_scaling_factor");
  const auto local_fp_clearing_param =
      makeLayerParam(local_clearing_layer_name, "footprint_clearing_enabled");

  if (!local_inflation_radius_param.empty()) {
    local_params.emplace_back(local_inflation_radius_param,
                              local_inflation_radius);
  }
  if (!local_cost_scaling_param.empty()) {
    local_params.emplace_back(local_cost_scaling_param,
                              local_cost_scaling_factor);
  }
  if (!local_fp_clearing_param.empty()) {
    local_params.emplace_back(local_fp_clearing_param,
                              local_footprint_clearing_enabled);
  }

  std::vector<rclcpp::Parameter> global_params;
  global_params.emplace_back("footprint", carry_footprint);
  global_params.emplace_back("footprint_padding", footprint_padding);

  const auto global_inflation_radius_param =
      makeLayerParam(global_inflation_layer_name, "inflation_radius");
  const auto global_cost_scaling_param =
      makeLayerParam(global_inflation_layer_name, "cost_scaling_factor");
  const auto global_fp_clearing_param =
      makeLayerParam(global_clearing_layer_name, "footprint_clearing_enabled");

  if (!global_inflation_radius_param.empty()) {
    global_params.emplace_back(global_inflation_radius_param,
                               global_inflation_radius);
  }
  if (!global_cost_scaling_param.empty()) {
    global_params.emplace_back(global_cost_scaling_param,
                               global_cost_scaling_factor);
  }
  if (!global_fp_clearing_param.empty()) {
    global_params.emplace_back(global_fp_clearing_param,
                               global_footprint_clearing_enabled);
  }

  if (!setRemoteParams(local_costmap_node, local_params)) {
    RCLCPP_ERROR(node_->get_logger(),
                 "SwitchToCarryMode: failed to update local costmap params");
    return BT::NodeStatus::FAILURE;
  }

  if (!setRemoteParams(global_costmap_node, global_params)) {
    RCLCPP_ERROR(node_->get_logger(),
                 "SwitchToCarryMode: failed to update global costmap params");
    return BT::NodeStatus::FAILURE;
  }

  if (post_param_wait_sec > 0.0) {
    rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(post_param_wait_sec)));
  }

  if (clear_costmaps) {
    if (!clearCostmap(local_clear_service) ||
        !clearCostmap(global_clear_service)) {
      RCLCPP_ERROR(node_->get_logger(),
                   "SwitchToCarryMode: failed to clear costmaps");
      return BT::NodeStatus::FAILURE;
    }
  }

  if (post_clear_wait_sec > 0.0) {
    rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(post_clear_wait_sec)));
  }

  RCLCPP_INFO(node_->get_logger(),
              "SwitchToCarryMode: applied carry footprint [%s]",
              carry_footprint.c_str());

  return BT::NodeStatus::SUCCESS;
}

// ========================= SwitchToNormalMode =========================

SwitchToNormalMode::SwitchToNormalMode(const std::string &name,
                                       const BT::NodeConfiguration &config)
    : Nav2ModeSwitchBase(name, config) {}

BT::PortsList SwitchToNormalMode::providedPorts() {
  auto ports = commonPorts();

  ports.insert(BT::InputPort<double>("normal_robot_radius", 0.22,
                                     "Robot radius for normal mode [m]"));

  return ports;
}

BT::NodeStatus SwitchToNormalMode::tick() {
  initializeRosNode();

  std::string local_costmap_node;
  std::string global_costmap_node;
  std::string local_clear_service;
  std::string global_clear_service;
  std::string local_inflation_layer_name;
  std::string global_inflation_layer_name;
  std::string local_clearing_layer_name;
  std::string global_clearing_layer_name;

  double normal_robot_radius = 0.22;
  double footprint_padding = 0.01;
  double local_inflation_radius = 0.55;
  double local_cost_scaling_factor = 10.0;
  double global_inflation_radius = 0.55;
  double global_cost_scaling_factor = 10.0;
  double post_param_wait_sec = 0.20;
  double post_clear_wait_sec = 0.20;

  bool local_footprint_clearing_enabled = true;
  bool global_footprint_clearing_enabled = true;
  bool clear_costmaps = true;

  if (!getInput("normal_robot_radius", normal_robot_radius) ||
      normal_robot_radius <= 0.0) {
    RCLCPP_ERROR(node_->get_logger(),
                 "SwitchToNormalMode: invalid input [normal_robot_radius]");
    return BT::NodeStatus::FAILURE;
  }

  getInput("local_costmap_node", local_costmap_node);
  getInput("global_costmap_node", global_costmap_node);
  getInput("local_clear_service", local_clear_service);
  getInput("global_clear_service", global_clear_service);
  getInput("local_inflation_layer_name", local_inflation_layer_name);
  getInput("global_inflation_layer_name", global_inflation_layer_name);
  getInput("local_clearing_layer_name", local_clearing_layer_name);
  getInput("global_clearing_layer_name", global_clearing_layer_name);
  getInput("footprint_padding", footprint_padding);
  getInput("local_inflation_radius", local_inflation_radius);
  getInput("local_cost_scaling_factor", local_cost_scaling_factor);
  getInput("global_inflation_radius", global_inflation_radius);
  getInput("global_cost_scaling_factor", global_cost_scaling_factor);
  getInput("local_footprint_clearing_enabled",
           local_footprint_clearing_enabled);
  getInput("global_footprint_clearing_enabled",
           global_footprint_clearing_enabled);
  getInput("clear_costmaps", clear_costmaps);
  getInput("post_param_wait_sec", post_param_wait_sec);
  getInput("post_clear_wait_sec", post_clear_wait_sec);

  std::vector<rclcpp::Parameter> local_params;
  local_params.emplace_back("footprint", std::string("[]"));
  local_params.emplace_back("robot_radius", normal_robot_radius);
  local_params.emplace_back("footprint_padding", footprint_padding);

  const auto local_inflation_radius_param =
      makeLayerParam(local_inflation_layer_name, "inflation_radius");
  const auto local_cost_scaling_param =
      makeLayerParam(local_inflation_layer_name, "cost_scaling_factor");
  const auto local_fp_clearing_param =
      makeLayerParam(local_clearing_layer_name, "footprint_clearing_enabled");

  if (!local_inflation_radius_param.empty()) {
    local_params.emplace_back(local_inflation_radius_param,
                              local_inflation_radius);
  }
  if (!local_cost_scaling_param.empty()) {
    local_params.emplace_back(local_cost_scaling_param,
                              local_cost_scaling_factor);
  }
  if (!local_fp_clearing_param.empty()) {
    local_params.emplace_back(local_fp_clearing_param,
                              local_footprint_clearing_enabled);
  }

  std::vector<rclcpp::Parameter> global_params;
  global_params.emplace_back("footprint", std::string("[]"));
  global_params.emplace_back("robot_radius", normal_robot_radius);
  global_params.emplace_back("footprint_padding", footprint_padding);

  const auto global_inflation_radius_param =
      makeLayerParam(global_inflation_layer_name, "inflation_radius");
  const auto global_cost_scaling_param =
      makeLayerParam(global_inflation_layer_name, "cost_scaling_factor");
  const auto global_fp_clearing_param =
      makeLayerParam(global_clearing_layer_name, "footprint_clearing_enabled");

  if (!global_inflation_radius_param.empty()) {
    global_params.emplace_back(global_inflation_radius_param,
                               global_inflation_radius);
  }
  if (!global_cost_scaling_param.empty()) {
    global_params.emplace_back(global_cost_scaling_param,
                               global_cost_scaling_factor);
  }
  if (!global_fp_clearing_param.empty()) {
    global_params.emplace_back(global_fp_clearing_param,
                               global_footprint_clearing_enabled);
  }

  if (!setRemoteParams(local_costmap_node, local_params)) {
    RCLCPP_ERROR(node_->get_logger(),
                 "SwitchToNormalMode: failed to update local costmap params");
    return BT::NodeStatus::FAILURE;
  }

  if (!setRemoteParams(global_costmap_node, global_params)) {
    RCLCPP_ERROR(node_->get_logger(),
                 "SwitchToNormalMode: failed to update global costmap params");
    return BT::NodeStatus::FAILURE;
  }

  if (post_param_wait_sec > 0.0) {
    rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(post_param_wait_sec)));
  }

  if (clear_costmaps) {
    if (!clearCostmap(local_clear_service) ||
        !clearCostmap(global_clear_service)) {
      RCLCPP_ERROR(node_->get_logger(),
                   "SwitchToNormalMode: failed to clear costmaps");
      return BT::NodeStatus::FAILURE;
    }
  }

  if (post_clear_wait_sec > 0.0) {
    rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(post_clear_wait_sec)));
  }

  RCLCPP_INFO(node_->get_logger(),
              "SwitchToNormalMode: restored robot_radius=%.3f",
              normal_robot_radius);

  return BT::NodeStatus::SUCCESS;
}

} // namespace rb1_bt

BT_REGISTER_NODES(factory) {
  factory.registerNodeType<rb1_bt::SwitchToCarryMode>("SwitchToCarryMode");
  factory.registerNodeType<rb1_bt::SwitchToNormalMode>("SwitchToNormalMode");
}