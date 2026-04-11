#include "rb1_nav2_bt_nodes/validate_shelf_candidate.hpp"

#include "rb1_nav2_bt_nodes/patrol_types.hpp"
#include "rb1_nav2_bt_nodes/shelf_scan_utils.hpp"

#include <behaviortree_cpp_v3/bt_factory.h>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

namespace rb1_bt {

ValidateShelfCandidate::ValidateShelfCandidate(
    const std::string &name, const BT::NodeConfiguration &config)
    : BT::StatefulActionNode(name, config) {}

bool ValidateShelfCandidate::initializeRosNode() {
  if (!node_) {
    config().blackboard->get<rclcpp::Node::SharedPtr>("node", node_);
    if (!node_) {
      throw std::runtime_error(
          "ValidateShelfCandidate: missing ROS node on blackboard");
    }
  }

  if (!tf_buffer_) {
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  }

  if (!tf_listener_) {
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  }

  return true;
}

void ValidateShelfCandidate::configureInterfaces() {
  if (!cmd_vel_pub_ || cmd_vel_pub_->get_topic_name() != cmd_vel_topic_) {
    cmd_vel_pub_ =
        node_->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);
  }

  const std::string current_scan_topic =
      scan_sub_ ? scan_sub_->get_topic_name() : std::string();

  if (!scan_sub_ || current_scan_topic != scan_topic_) {
    scan_sub_ = node_->create_subscription<sensor_msgs::msg::LaserScan>(
        scan_topic_, rclcpp::SensorDataQoS(),
        std::bind(&ValidateShelfCandidate::scanCallback, this,
                  std::placeholders::_1));
  }
}

BT::PortsList ValidateShelfCandidate::providedPorts() {
  return {
      // Candidate geometry ports kept for BT/XML compatibility.
      BT::InputPort<geometry_msgs::msg::Point>(
          "candidate_center", "Legacy compatibility input, currently unused"),
      BT::InputPort<geometry_msgs::msg::Point>(
          "candidate_left_leg", "Legacy compatibility input, currently unused"),
      BT::InputPort<geometry_msgs::msg::Point>(
          "candidate_right_leg",
          "Legacy compatibility input, currently unused"),

      BT::InputPort<std::string>(
          "candidate_window",
          "Window where candidate was detected: left/right"),

      BT::InputPort<std::string>("scan_topic", std::string("/scan"),
                                 "LaserScan topic used for validation"),
      BT::InputPort<std::string>("cmd_vel_topic", std::string("/cmd_vel"),
                                 "Velocity command topic used to stop robot"),
      BT::InputPort<std::string>("target_frame", std::string("map"),
                                 "Frame of outputs"),

      BT::InputPort<int>("left_window_start_idx", 55,
                         "Left scan window start index"),
      BT::InputPort<int>("left_window_end_idx", 305,
                         "Left scan window end index"),
      BT::InputPort<int>("right_window_start_idx", 775,
                         "Right scan window start index"),
      BT::InputPort<int>("right_window_end_idx", 1025,
                         "Right scan window end index"),

      BT::InputPort<double>("expected_leg_spacing", 0.60,
                            "Expected spacing between shelf legs"),
      BT::InputPort<double>("leg_spacing_tolerance", 0.14,
                            "Allowed spacing error"),
      BT::InputPort<double>("cluster_gap_tolerance", 0.10,
                            "Gap tolerance inside one cluster"),
      BT::InputPort<double>("intensity_threshold", 7500.0,
                            "Intensity threshold for reflector-like hits"),
      BT::InputPort<double>("max_detection_range", 2.5,
                            "Maximum range used in validation"),
      BT::InputPort<int>("min_cluster_points", 2,
                         "Minimum number of points in cluster"),
      BT::InputPort<int>("min_consecutive_hits", 2,
                         "Required number of consecutive valid detections"),
      BT::InputPort<double>("stable_hit_distance_tol", 0.18,
                            "Maximum allowed jump between accepted detections"),
      BT::InputPort<double>("candidate_match_distance_tol", 0.28,
                            "Legacy compatibility input, unused"),
      BT::InputPort<double>("stop_settle_time", 0.40,
                            "Time to wait after stop command"),
      BT::InputPort<double>("wait_duration", 10.0,
                            "Overall validation timeout"),
      BT::InputPort<double>("max_scan_age", 0.90,
                            "Maximum accepted age of LaserScan"),

      BT::OutputPort<geometry_msgs::msg::Point>("shelf_center",
                                                "Validated shelf center"),
      BT::OutputPort<geometry_msgs::msg::Point>("shelf_left_leg",
                                                "Validated shelf left leg"),
      BT::OutputPort<geometry_msgs::msg::Point>("shelf_right_leg",
                                                "Validated shelf right leg")};
}

BT::NodeStatus ValidateShelfCandidate::onStart() {
  initializeRosNode();

  getInput("scan_topic", scan_topic_);
  getInput("cmd_vel_topic", cmd_vel_topic_);
  getInput("target_frame", target_frame_);

  getInput("left_window_start_idx", left_window_start_idx_);
  getInput("left_window_end_idx", left_window_end_idx_);
  getInput("right_window_start_idx", right_window_start_idx_);
  getInput("right_window_end_idx", right_window_end_idx_);

  getInput("expected_leg_spacing", expected_leg_spacing_);
  getInput("leg_spacing_tolerance", leg_spacing_tolerance_);
  getInput("cluster_gap_tolerance", cluster_gap_tolerance_);
  getInput("intensity_threshold", intensity_threshold_);
  getInput("max_detection_range", max_detection_range_);
  getInput("min_cluster_points", min_cluster_points_);
  getInput("min_consecutive_hits", min_consecutive_hits_);
  getInput("stable_hit_distance_tol", stable_hit_distance_tol_);
  getInput("stop_settle_time", stop_settle_time_);
  getInput("wait_duration", wait_duration_sec_);
  getInput("max_scan_age", max_scan_age_sec_);

  if (!getInput("candidate_window", candidate_window_)) {
    throw std::runtime_error(
        "ValidateShelfCandidate: missing required input [candidate_window]");
  }

  std::transform(candidate_window_.begin(), candidate_window_.end(),
                 candidate_window_.begin(), [](unsigned char c) {
                   return static_cast<char>(std::tolower(c));
                 });

  if (candidate_window_ != "left" && candidate_window_ != "right") {
    throw std::runtime_error(
        "ValidateShelfCandidate: candidate_window must be 'left' or 'right'");
  }

  configureInterfaces();

  start_time_ = node_->now();
  has_processed_scan_ = false;
  last_processed_scan_stamp_ =
      rclcpp::Time(0, 0, node_->get_clock()->get_clock_type());

  resetConsecutiveValidation();
  publishZeroVelocity();

  RCLCPP_INFO(node_->get_logger(),
              "ValidateShelfCandidate: shelf-only validation started. "
              "window=%s spacing=%.3f±%.3f gap_tol=%.3f min_cluster_points=%d "
              "min_consecutive_hits=%d stable_tol=%.3f timeout=%.2f",
              candidate_window_.c_str(), expected_leg_spacing_,
              leg_spacing_tolerance_, cluster_gap_tolerance_,
              min_cluster_points_, min_consecutive_hits_,
              stable_hit_distance_tol_, wait_duration_sec_);

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus ValidateShelfCandidate::onRunning() {
  publishZeroVelocity();

  const double elapsed = (node_->now() - start_time_).seconds();
  if (elapsed >= wait_duration_sec_) {
    RCLCPP_WARN(node_->get_logger(),
                "ValidateShelfCandidate: validation timeout");
    return BT::NodeStatus::FAILURE;
  }

  if (elapsed < stop_settle_time_) {
    return BT::NodeStatus::RUNNING;
  }

  sensor_msgs::msg::LaserScan scan;
  if (!fetchLatestScan(scan)) {
    RCLCPP_DEBUG_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                          "ValidateShelfCandidate: waiting for first scan");
    return BT::NodeStatus::RUNNING;
  }

  if (scan.header.stamp.sec == 0 && scan.header.stamp.nanosec == 0) {
    RCLCPP_DEBUG_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                          "ValidateShelfCandidate: ignoring zero-stamp scan");
    return BT::NodeStatus::RUNNING;
  }

  const rclcpp::Time scan_stamp(scan.header.stamp,
                                node_->get_clock()->get_clock_type());

  if (has_processed_scan_ &&
      scan_stamp.nanoseconds() == last_processed_scan_stamp_.nanoseconds()) {
    RCLCPP_DEBUG_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                          "ValidateShelfCandidate: duplicate scan stamp");
    return BT::NodeStatus::RUNNING;
  }

  const double scan_age = (node_->now() - scan_stamp).seconds();
  if (scan_age > max_scan_age_sec_) {
    RCLCPP_DEBUG_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 1000,
        "ValidateShelfCandidate: scan too old age=%.3f > %.3f", scan_age,
        max_scan_age_sec_);
    return BT::NodeStatus::RUNNING;
  }

  has_processed_scan_ = true;
  last_processed_scan_stamp_ = scan_stamp;

  Detection detection;
  if (!validateFromScan(scan, detection)) {
    resetConsecutiveValidation();
    return BT::NodeStatus::RUNNING;
  }

  if (!isStableAgainstPrevious(detection)) {
    resetConsecutiveValidation();
    accumulateDetection(detection);
    last_valid_detection_ = detection;
    has_last_valid_detection_ = true;
    consecutive_valid_hits_ = 1;

    RCLCPP_INFO(node_->get_logger(),
                "ValidateShelfCandidate: valid shelf detection but restarted "
                "streak after jump");
    return BT::NodeStatus::RUNNING;
  }

  accumulateDetection(detection);
  last_valid_detection_ = detection;
  has_last_valid_detection_ = true;
  consecutive_valid_hits_++;

  RCLCPP_INFO(node_->get_logger(),
              "ValidateShelfCandidate: accepted hit %d/%d, "
              "center=(%.3f, %.3f), spacing=%.3f",
              consecutive_valid_hits_, min_consecutive_hits_,
              detection.center.x, detection.center.y, detection.spacing);

  if (consecutive_valid_hits_ >= min_consecutive_hits_) {
    const auto avg = averagedDetection();

    setOutput("shelf_center", avg.center);
    setOutput("shelf_left_leg", avg.left_leg);
    setOutput("shelf_right_leg", avg.right_leg);

    RCLCPP_INFO(node_->get_logger(),
                "ValidateShelfCandidate: validation SUCCESS. "
                "center=(%.3f, %.3f), left=(%.3f, %.3f), right=(%.3f, %.3f), "
                "spacing=%.3f",
                avg.center.x, avg.center.y, avg.left_leg.x, avg.left_leg.y,
                avg.right_leg.x, avg.right_leg.y, avg.spacing);

    return BT::NodeStatus::SUCCESS;
  }

  return BT::NodeStatus::RUNNING;
}

void ValidateShelfCandidate::onHalted() {
  publishZeroVelocity();
  resetConsecutiveValidation();
  RCLCPP_INFO(node_->get_logger(), "ValidateShelfCandidate: halted");
}

void ValidateShelfCandidate::scanCallback(
    const sensor_msgs::msg::LaserScan::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(scan_mutex_);
  latest_scan_ = *msg;
  has_scan_ = true;
}

void ValidateShelfCandidate::publishZeroVelocity() const {
  if (!cmd_vel_pub_) {
    return;
  }

  geometry_msgs::msg::Twist cmd;
  cmd.linear.x = 0.0;
  cmd.linear.y = 0.0;
  cmd.linear.z = 0.0;
  cmd.angular.x = 0.0;
  cmd.angular.y = 0.0;
  cmd.angular.z = 0.0;

  cmd_vel_pub_->publish(cmd);
}

bool ValidateShelfCandidate::fetchLatestScan(
    sensor_msgs::msg::LaserScan &scan) const {
  std::lock_guard<std::mutex> lock(scan_mutex_);
  if (!has_scan_) {
    return false;
  }
  scan = latest_scan_;
  return true;
}

bool ValidateShelfCandidate::validateFromScan(
    const sensor_msgs::msg::LaserScan &scan, Detection &detection_out) const {
  ShelfDetectorParams params;
  params.expected_leg_spacing = expected_leg_spacing_;
  params.leg_spacing_tolerance = leg_spacing_tolerance_;
  params.cluster_gap_tolerance = cluster_gap_tolerance_;
  params.intensity_threshold = intensity_threshold_;
  params.max_detection_range = max_detection_range_;
  params.min_cluster_points = min_cluster_points_;

  TransformPointOptions tf_options;
  tf_options.exact_timeout_sec = tf_exact_timeout_sec_;
  tf_options.allow_latest_fallback = true;
  tf_options.fallback_timeout_sec = tf_fallback_timeout_sec_;
  tf_options.warn_on_fallback = true;

  int active_window_start = 0;
  int active_window_end = 0;
  const char *active_window_name = nullptr;

  if (candidate_window_ == "left") {
    active_window_start = left_window_start_idx_;
    active_window_end = left_window_end_idx_;
    active_window_name = "left";
  } else if (candidate_window_ == "right") {
    active_window_start = right_window_start_idx_;
    active_window_end = right_window_end_idx_;
    active_window_name = "right";
  } else {
    logRejectThrottled("invalid candidate_window='%s'",
                       candidate_window_.c_str());
    return false;
  }

  const auto window_clusters = scan_utils::extractClustersFromIndexWindow(
      scan, active_window_start, active_window_end, params);

  if (window_clusters.size() < 2) {
    logRejectThrottled("not enough clusters in %s window (found=%zu need>=2)",
                       active_window_name, window_clusters.size());
    return false;
  }

  const auto cluster_score = [](const ScanCluster &cluster) {
    const double range =
        std::hypot(cluster.centroid_laser.x, cluster.centroid_laser.y);

    // Prefer more points first, then intensity, then slightly closer range.
    return 10.0 * static_cast<double>(cluster.num_points) +
           0.001 * cluster.mean_intensity - 0.5 * range;
  };

  bool found_pair = false;
  double best_pair_score = -std::numeric_limits<double>::infinity();

  ScanCluster best_cluster_a;
  ScanCluster best_cluster_b;
  double best_spacing = 0.0;

  for (size_t i = 0; i < window_clusters.size(); ++i) {
    for (size_t j = i + 1; j < window_clusters.size(); ++j) {
      const auto &cluster_a = window_clusters[i];
      const auto &cluster_b = window_clusters[j];

      const double spacing = scan_utils::distance2D(cluster_a.centroid_laser,
                                                    cluster_b.centroid_laser);
      const double spacing_error = std::fabs(spacing - expected_leg_spacing_);

      if (spacing_error > leg_spacing_tolerance_) {
        continue;
      }

      const double pair_score = cluster_score(cluster_a) +
                                cluster_score(cluster_b) - 20.0 * spacing_error;

      if (!found_pair || pair_score > best_pair_score) {
        found_pair = true;
        best_pair_score = pair_score;
        best_cluster_a = cluster_a;
        best_cluster_b = cluster_b;
        best_spacing = spacing;
      }
    }
  }

  if (!found_pair) {
    logRejectThrottled(
        "no valid pair inside %s window (clusters=%zu expected=%.3f tol=%.3f)",
        active_window_name, window_clusters.size(), expected_leg_spacing_,
        leg_spacing_tolerance_);
    return false;
  }

  // Enforce consistent leg ordering: in laser frame, larger Y is more to the
  // left.
  const ScanCluster *laser_left_cluster = &best_cluster_a;
  const ScanCluster *laser_right_cluster = &best_cluster_b;

  if (best_cluster_a.centroid_laser.y < best_cluster_b.centroid_laser.y) {
    laser_left_cluster = &best_cluster_b;
    laser_right_cluster = &best_cluster_a;
  }

  const rclcpp::Time transform_time(scan.header.stamp,
                                    node_->get_clock()->get_clock_type());

  geometry_msgs::msg::Point left_leg_target;
  if (!scan_utils::transformPointToFrame(
          *tf_buffer_, scan.header.frame_id, target_frame_, transform_time,
          laser_left_cluster->centroid_laser, left_leg_target,
          node_->get_logger(), tf_options)) {
    logRejectThrottled("failed to transform left cluster centroid");
    return false;
  }

  geometry_msgs::msg::Point right_leg_target;
  if (!scan_utils::transformPointToFrame(
          *tf_buffer_, scan.header.frame_id, target_frame_, transform_time,
          laser_right_cluster->centroid_laser, right_leg_target,
          node_->get_logger(), tf_options)) {
    logRejectThrottled("failed to transform right cluster centroid");
    return false;
  }

  geometry_msgs::msg::Point center;
  center.x = 0.5 * (left_leg_target.x + right_leg_target.x);
  center.y = 0.5 * (left_leg_target.y + right_leg_target.y);
  center.z = 0.5 * (left_leg_target.z + right_leg_target.z);

  detection_out.left_leg = left_leg_target;
  detection_out.right_leg = right_leg_target;
  detection_out.center = center;
  detection_out.spacing = best_spacing;

  RCLCPP_DEBUG(
      node_->get_logger(),
      "ValidateShelfCandidate: valid shelf detection in %s window "
      "left=(%.3f, %.3f) right=(%.3f, %.3f) center=(%.3f, %.3f) "
      "spacing=%.3f left_pts=%d right_pts=%d left_I=%.1f right_I=%.1f",
      active_window_name, detection_out.left_leg.x, detection_out.left_leg.y,
      detection_out.right_leg.x, detection_out.right_leg.y,
      detection_out.center.x, detection_out.center.y, detection_out.spacing,
      laser_left_cluster->num_points, laser_right_cluster->num_points,
      laser_left_cluster->mean_intensity, laser_right_cluster->mean_intensity);

  return true;
}

bool ValidateShelfCandidate::isStableAgainstPrevious(
    const Detection &current) const {
  if (!has_last_valid_detection_) {
    return true;
  }

  const double center_dist =
      scan_utils::distance2D(current.center, last_valid_detection_.center);
  const double left_dist =
      scan_utils::distance2D(current.left_leg, last_valid_detection_.left_leg);
  const double right_dist = scan_utils::distance2D(
      current.right_leg, last_valid_detection_.right_leg);

  const bool stable = center_dist <= stable_hit_distance_tol_ &&
                      left_dist <= stable_hit_distance_tol_ &&
                      right_dist <= stable_hit_distance_tol_;

  if (!stable) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                         "ValidateShelfCandidate: stability failed "
                         "(center=%.3f left=%.3f right=%.3f tol=%.3f)",
                         center_dist, left_dist, right_dist,
                         stable_hit_distance_tol_);
  }

  return stable;
}

void ValidateShelfCandidate::logRejectThrottled(const char *fmt, ...) const {
  char buffer[512];

  va_list args;
  va_start(args, fmt);
  std::vsnprintf(buffer, sizeof(buffer), fmt, args);
  va_end(args);

  RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                       "ValidateShelfCandidate: rejected scan - %s", buffer);
}

void ValidateShelfCandidate::resetConsecutiveValidation() {
  consecutive_valid_hits_ = 0;
  has_last_valid_detection_ = false;

  accumulated_center_.x = 0.0;
  accumulated_center_.y = 0.0;
  accumulated_center_.z = 0.0;

  accumulated_left_leg_.x = 0.0;
  accumulated_left_leg_.y = 0.0;
  accumulated_left_leg_.z = 0.0;

  accumulated_right_leg_.x = 0.0;
  accumulated_right_leg_.y = 0.0;
  accumulated_right_leg_.z = 0.0;

  accumulated_spacing_ = 0.0;
  accumulated_count_ = 0;
}

void ValidateShelfCandidate::accumulateDetection(const Detection &detection) {
  accumulated_center_.x += detection.center.x;
  accumulated_center_.y += detection.center.y;
  accumulated_center_.z += detection.center.z;

  accumulated_left_leg_.x += detection.left_leg.x;
  accumulated_left_leg_.y += detection.left_leg.y;
  accumulated_left_leg_.z += detection.left_leg.z;

  accumulated_right_leg_.x += detection.right_leg.x;
  accumulated_right_leg_.y += detection.right_leg.y;
  accumulated_right_leg_.z += detection.right_leg.z;

  accumulated_spacing_ += detection.spacing;
  accumulated_count_++;
}

ValidateShelfCandidate::Detection
ValidateShelfCandidate::averagedDetection() const {
  Detection avg;

  if (accumulated_count_ <= 0) {
    return avg;
  }

  const double inv_n = 1.0 / static_cast<double>(accumulated_count_);

  avg.center.x = accumulated_center_.x * inv_n;
  avg.center.y = accumulated_center_.y * inv_n;
  avg.center.z = accumulated_center_.z * inv_n;

  avg.left_leg.x = accumulated_left_leg_.x * inv_n;
  avg.left_leg.y = accumulated_left_leg_.y * inv_n;
  avg.left_leg.z = accumulated_left_leg_.z * inv_n;

  avg.right_leg.x = accumulated_right_leg_.x * inv_n;
  avg.right_leg.y = accumulated_right_leg_.y * inv_n;
  avg.right_leg.z = accumulated_right_leg_.z * inv_n;

  avg.spacing = accumulated_spacing_ * inv_n;
  return avg;
}

} // namespace rb1_bt

BT_REGISTER_NODES(factory) {
  factory.registerNodeType<rb1_bt::ValidateShelfCandidate>(
      "ValidateShelfCandidate");
}