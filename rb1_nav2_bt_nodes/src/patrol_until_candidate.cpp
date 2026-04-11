#include "rb1_nav2_bt_nodes/patrol_until_candidate.hpp"

#include "rb1_nav2_bt_nodes/shelf_scan_utils.hpp"

#include <behaviortree_cpp_v3/bt_factory.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdexcept>

using namespace std::chrono_literals;

namespace rb1_bt {

PatrolUntilCandidate::PatrolUntilCandidate(const std::string &name,
                                           const BT::NodeConfiguration &config)
    : BT::StatefulActionNode(name, config) {}

BT::PortsList PatrolUntilCandidate::providedPorts() {
  return {
      BT::InputPort<std::vector<rb1_bt::SimplePose2D>>("patrol_waypoints"),
      BT::InputPort<std::string>("nav_action_name", "navigate_to_pose",
                                 "BT navigator name"),
      BT::InputPort<std::string>("scan_topic", "/scan", "LaserScan topic"),
      BT::InputPort<std::string>("target_frame", "map", "Target/Global frame"),

      BT::InputPort<int>("scan_queue_size", 0, "Scan queue size"),
      BT::InputPort<int>("transform_tolerance_ms", 0, "TF tolerance in ms"),
      BT::InputPort<int>("some_other_int_port", 0, "description"),
      BT::InputPort<int>("another_int_port", 0, "description"),

      BT::InputPort<int>("left_window_start_idx", 65, "desc"),
      BT::InputPort<int>("left_window_end_idx", 295, "desc"),
      BT::InputPort<int>("right_window_start_idx", 785, "desc"),
      BT::InputPort<int>("right_window_end_idx", 1015, "desc"),

      BT::InputPort<double>("expected_leg_spacing", 0.60,
                            "Expected spacing between legs"),
      BT::InputPort<double>("leg_spacing_tolerance", 0.08,
                            "Tolerance on leg spacing"),
      BT::InputPort<double>("cluster_gap_tolerance", 0.10,
                            "Gap tolerance for clustering"),
      BT::InputPort<double>("intensity_threshold", 7500.0,
                            "Minimum reflector intensity"),
      BT::InputPort<double>("max_detection_range", 2.5,
                            "Maximum detection range"),
      BT::InputPort<int>("min_cluster_points", 2, "Minimum points per cluster"),
      BT::InputPort<int>("min_consecutive_hits", 3,
                         "Hits needed to confirm candidate"),
      BT::InputPort<double>("stable_hit_distance_tol", 0.25,
                            "Allowed distance drift between hits"),
      BT::InputPort<int>("max_waypoint_failures", 3,
                         "Maximum waypoint failures"),

      BT::OutputPort<geometry_msgs::msg::Point>("candidate_left_leg"),
      BT::OutputPort<geometry_msgs::msg::Point>("candidate_right_leg"),
      BT::OutputPort<geometry_msgs::msg::Point>("candidate_center"),
      BT::OutputPort<double>("candidate_confidence"),
      BT::OutputPort<std::string>("candidate_window")};
}

bool PatrolUntilCandidate::loadPorts() {
  getInput("patrol_waypoints", patrol_waypoints_);

  getInput("nav_action_name", nav_action_name_);
  getInput("scan_topic", scan_topic_);
  getInput("target_frame", target_frame_);

  getInput("left_window_start_idx", left_window_start_idx_);
  getInput("left_window_end_idx", left_window_end_idx_);
  getInput("right_window_start_idx", right_window_start_idx_);
  getInput("right_window_end_idx", right_window_end_idx_);

  getInput("expected_leg_spacing", detector_params_.expected_leg_spacing);
  getInput("leg_spacing_tolerance", detector_params_.leg_spacing_tolerance);
  getInput("cluster_gap_tolerance", detector_params_.cluster_gap_tolerance);
  getInput("intensity_threshold", detector_params_.intensity_threshold);
  getInput("max_detection_range", detector_params_.max_detection_range);
  getInput("min_cluster_points", detector_params_.min_cluster_points);

  getInput("min_consecutive_hits", min_consecutive_hits_);
  getInput("stable_hit_distance_tol", stable_hit_distance_tol_);
  getInput("max_waypoint_failures", max_waypoint_failures_);

  return true;
}

bool PatrolUntilCandidate::initializeRosResources() {
  if (!node_) {
    config().blackboard->get<rclcpp::Node::SharedPtr>("node", node_);
    if (!node_) {
      throw std::runtime_error(
          "PatrolUntilCandidate: missing ROS node on blackboard");
    }
  }

  loadPorts();

  if (!nav_client_) {
    nav_client_ =
        rclcpp_action::create_client<NavigateToPose>(node_, nav_action_name_);
  }

  if (!tf_buffer_) {
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  }

  if (!tf_listener_) {
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  }

  if (!scan_sub_) {
    scan_sub_ = node_->create_subscription<sensor_msgs::msg::LaserScan>(
        scan_topic_, rclcpp::SensorDataQoS(),
        std::bind(&PatrolUntilCandidate::scanCallback, this,
                  std::placeholders::_1));
  }

  return true;
}

void PatrolUntilCandidate::resetRunState() {
  current_waypoint_idx_ = 0;
  waypoint_failures_ = 0;

  goal_handle_future_ = {};
  goal_handle_.reset();
  result_future_ = {};

  stable_candidate_ = ShelfCandidateDetection{};
  last_candidate_ = ShelfCandidateDetection{};
  has_last_candidate_ = false;
  consecutive_hits_ = 0;
  last_candidate_window_.clear();
  stable_candidate_window_.clear();
}

BT::NodeStatus PatrolUntilCandidate::onStart() {
  initializeRosResources();
  resetRunState();

  if (patrol_waypoints_.empty()) {
    RCLCPP_ERROR(node_->get_logger(),
                 "PatrolUntilCandidate: patrol_waypoints is empty");
    return BT::NodeStatus::FAILURE;
  }

  if (!nav_client_->wait_for_action_server(2s)) {
    RCLCPP_ERROR(
        node_->get_logger(),
        "PatrolUntilCandidate: navigate_to_pose action server unavailable");
    return BT::NodeStatus::FAILURE;
  }

  if (!sendCurrentWaypointGoal()) {
    RCLCPP_ERROR(node_->get_logger(),
                 "PatrolUntilCandidate: failed to send first patrol goal");
    return BT::NodeStatus::FAILURE;
  }

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus PatrolUntilCandidate::onRunning() {
  if (!goal_handle_ && goal_handle_future_.valid() &&
      goal_handle_future_.wait_for(0ms) == std::future_status::ready) {
    goal_handle_ = goal_handle_future_.get();

    if (!goal_handle_) {
      RCLCPP_WARN(node_->get_logger(),
                  "PatrolUntilCandidate: current goal rejected");
      waypoint_failures_++;

      if (waypoint_failures_ > max_waypoint_failures_ ||
          !advanceToNextWaypoint()) {
        return BT::NodeStatus::FAILURE;
      }
    } else {
      result_future_ = nav_client_->async_get_result(goal_handle_);
    }
  }

  if (auto detection = detectCandidateFromLatestScan()) {
    if (updateStableHit(*detection)) {
      setOutput("candidate_center", stable_candidate_.center_target_frame);
      setOutput("candidate_left_leg", stable_candidate_.left_leg_target_frame);
      setOutput("candidate_right_leg",
                stable_candidate_.right_leg_target_frame);
      setOutput("candidate_confidence", stable_candidate_.confidence);
      setOutput("candidate_window", stable_candidate_window_);

      RCLCPP_INFO(node_->get_logger(), "PatrolUntilCandidate: stable shelf "
                                       "candidate found, canceling nav goal");

      if (goal_handle_) {
        nav_client_->async_cancel_goal(goal_handle_);
      }

      return BT::NodeStatus::SUCCESS;
    }
  } else {
    has_last_candidate_ = false;
    consecutive_hits_ = 0;
  }

  if (result_future_.valid() &&
      result_future_.wait_for(0ms) == std::future_status::ready) {
    const auto result = result_future_.get();
    goal_handle_.reset();

    switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      waypoint_failures_ = 0;
      if (!advanceToNextWaypoint()) {
        RCLCPP_WARN(node_->get_logger(),
                    "PatrolUntilCandidate: patrol completed and no candidate "
                    "was found");
        return BT::NodeStatus::FAILURE;
      }
      break;

    case rclcpp_action::ResultCode::ABORTED:
    case rclcpp_action::ResultCode::CANCELED:
    default:
      waypoint_failures_++;
      RCLCPP_WARN(node_->get_logger(),
                  "PatrolUntilCandidate: waypoint navigation failed/canceled, "
                  "failures=%d",
                  waypoint_failures_);

      if (waypoint_failures_ > max_waypoint_failures_ ||
          !advanceToNextWaypoint()) {
        return BT::NodeStatus::FAILURE;
      }
      break;
    }
  }

  return BT::NodeStatus::RUNNING;
}

void PatrolUntilCandidate::onHalted() {
  RCLCPP_INFO(node_->get_logger(), "PatrolUntilCandidate halted");

  if (goal_handle_) {
    nav_client_->async_cancel_goal(goal_handle_);
  }
}

bool PatrolUntilCandidate::sendCurrentWaypointGoal() {
  if (current_waypoint_idx_ >= patrol_waypoints_.size()) {
    return false;
  }

  const auto target_pose = scan_utils::buildPoseStampedFromWaypoint(
      patrol_waypoints_[current_waypoint_idx_], target_frame_, node_->now());

  NavigateToPose::Goal goal;
  goal.pose = target_pose;

  typename rclcpp_action::Client<NavigateToPose>::SendGoalOptions
      send_goal_options;
  goal_handle_future_ = nav_client_->async_send_goal(goal, send_goal_options);

  RCLCPP_INFO(node_->get_logger(),
              "PatrolUntilCandidate: sent waypoint %zu / %zu",
              current_waypoint_idx_ + 1, patrol_waypoints_.size());

  return true;
}

bool PatrolUntilCandidate::advanceToNextWaypoint() {
  current_waypoint_idx_++;
  goal_handle_future_ = {};
  goal_handle_.reset();
  result_future_ = {};

  if (current_waypoint_idx_ >= patrol_waypoints_.size()) {
    return false;
  }

  return sendCurrentWaypointGoal();
}

void PatrolUntilCandidate::scanCallback(
    const sensor_msgs::msg::LaserScan::SharedPtr msg) {
  std::scoped_lock<std::mutex> lock(scan_mutex_);
  last_scan_ = msg;
}

std::optional<ShelfCandidateDetection>
PatrolUntilCandidate::detectCandidateFromLatestScan() {
  sensor_msgs::msg::LaserScan::SharedPtr scan;

  {
    std::scoped_lock<std::mutex> lock(scan_mutex_);
    scan = last_scan_;
  }

  if (!scan) {
    return std::nullopt;
  }

  const auto left_detection = scan_utils::detectShelfCandidateInWindow(
      *scan, left_window_start_idx_, left_window_end_idx_, detector_params_,
      *tf_buffer_, target_frame_, node_->now(), node_->get_logger());

  const auto right_detection = scan_utils::detectShelfCandidateInWindow(
      *scan, right_window_start_idx_, right_window_end_idx_, detector_params_,
      *tf_buffer_, target_frame_, node_->now(), node_->get_logger());

  if (left_detection && right_detection) {
    if (left_detection->confidence >= right_detection->confidence) {
      last_candidate_window_ = "left";
      return left_detection;
    } else {
      last_candidate_window_ = "right";
      return right_detection;
    }
  }

  if (left_detection) {
    last_candidate_window_ = "left";
    return left_detection;
  }

  if (right_detection) {
    last_candidate_window_ = "right";
    return right_detection;
  }

  last_candidate_window_.clear();
  return std::nullopt;
}

bool PatrolUntilCandidate::updateStableHit(
    const ShelfCandidateDetection &detection) {
  if (!detection.valid) {
    has_last_candidate_ = false;
    consecutive_hits_ = 0;
    last_candidate_window_.clear();
    stable_candidate_window_.clear();
    return false;
  }

  if (!has_last_candidate_) {
    last_candidate_ = detection;
    stable_candidate_ = detection;
    stable_candidate_window_ = last_candidate_window_;
    has_last_candidate_ = true;
    consecutive_hits_ = 1;
    return false;
  }

  const double distance = pointDistance2D(last_candidate_.center_target_frame,
                                          detection.center_target_frame);

  if (distance <= stable_hit_distance_tol_) {
    consecutive_hits_++;
    last_candidate_ = detection;
    stable_candidate_ = detection;
    stable_candidate_window_ = last_candidate_window_;
  } else {
    consecutive_hits_ = 1;
    last_candidate_ = detection;
    stable_candidate_ = detection;
    stable_candidate_window_ = last_candidate_window_;
  }

  return consecutive_hits_ >= min_consecutive_hits_;
}

double PatrolUntilCandidate::pointDistance2D(
    const geometry_msgs::msg::Point &a,
    const geometry_msgs::msg::Point &b) const {
  return std::hypot(a.x - b.x, a.y - b.y);
}

} // namespace rb1_bt

BT_REGISTER_NODES(factory) {
  factory.registerNodeType<rb1_bt::PatrolUntilCandidate>(
      "PatrolUntilCandidate");
}