#include "rb1_nav2_bt_nodes/refine_shelf_geometry.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "behaviortree_cpp_v3/bt_factory.h"
#include "rb1_nav2_bt_nodes/patrol_types.hpp"
#include "rb1_nav2_bt_nodes/shelf_scan_utils.hpp"

namespace rb1_bt {

RefineShelfGeometry::RefineShelfGeometry(const std::string &name,
                                         const BT::NodeConfiguration &config)
    : BT::StatefulActionNode(name, config) {}

BT::PortsList RefineShelfGeometry::providedPorts() {
  return {
      BT::InputPort<std::string>("scan_topic", "/scan", "LaserScan topic"),
      BT::InputPort<std::string>("cmd_vel_topic", "/cmd_vel", "cmd_vel topic"),
      BT::InputPort<std::string>("target_frame", std::string("map"),
                                 "Frame for output shelf geometry"),

      BT::InputPort<int>("front_window_start_idx", 430,
                         "Front window start scan index"),
      BT::InputPort<int>("front_window_end_idx", 650,
                         "Front window end scan index"),

      BT::InputPort<double>("expected_leg_spacing", 0.60,
                            "Expected spacing between shelf legs"),
      BT::InputPort<double>("leg_spacing_tolerance", 0.07,
                            "Tolerance on expected spacing"),
      BT::InputPort<double>("cluster_gap_tolerance", 0.05,
                            "Max gap between adjacent cluster points"),
      BT::InputPort<double>("intensity_threshold", 3000.0,
                            "Min intensity for valid scan point"),
      BT::InputPort<double>("max_detection_range", 1.8,
                            "Max usable range for refinement"),
      BT::InputPort<int>("min_cluster_points", 5,
                         "Min points required per cluster"),
      BT::InputPort<int>("min_consecutive_hits", 3,
                         "Required stable consecutive detections"),
      BT::InputPort<double>("stable_hit_distance_tol", 0.08,
                            "Max center shift to count as stable"),
      BT::InputPort<double>("stop_settle_time", 0.30,
                            "Time to wait after stop before processing scans"),
      BT::InputPort<double>("wait_duration", 4.0,
                            "Total refinement timeout in seconds"),
      BT::InputPort<double>("max_scan_age", 0.40,
                            "Max acceptable scan age in seconds"),

      BT::InputPort<int>("side_window_size", 100,
                         "Number of scan indices used per side window"),
      BT::InputPort<double>("side_range_cap", 2.0,
                            "Cap for side-clearance scoring"),
      BT::InputPort<double>("side_free_min_diff", 0.10,
                            "Minimum score difference to prefer one side"),

      BT::OutputPort<int>(
          "backup_turn_sign",
          "Preferred backing turn sign: +1 left, -1 right, 0 neutral"),
      BT::OutputPort<double>("left_free_score",
                             "Robust clearance score on left side"),
      BT::OutputPort<double>("right_free_score",
                             "Robust clearance score on right side"),

      BT::OutputPort<double>("shelf_tangent_yaw",
                             "Yaw of shelf front (direction along legs line)"),
      BT::OutputPort<double>("shelf_normal_yaw",
                             "Yaw of shelf approach normal facing the shelf"),
      BT::OutputPort<geometry_msgs::msg::Point>("shelf_center",
                                                "Refined shelf center"),
      BT::OutputPort<geometry_msgs::msg::Point>("shelf_left_leg",
                                                "Refined shelf left leg"),
      BT::OutputPort<geometry_msgs::msg::Point>("shelf_right_leg",
                                                "Refined shelf right leg")};
}

bool RefineShelfGeometry::initializeRosNode() {
  if (!node_) {
    config().blackboard->get<rclcpp::Node::SharedPtr>("node", node_);
    if (!node_) {
      throw std::runtime_error(
          "RefineShelfGeometry: missing ROS node on blackboard");
    }
  }

  if (!tf_buffer_) {
    config().blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer",
                                                               tf_buffer_);
    if (!tf_buffer_) {
      throw std::runtime_error(
          "RefineShelfGeometry: missing tf_buffer on blackboard");
    }
  }

  return true;
}

void RefineShelfGeometry::ensureInterfaces() {
  std::string new_scan_topic = "/scan";
  std::string new_cmd_vel_topic = "/cmd_vel";

  getInput("scan_topic", new_scan_topic);
  getInput("cmd_vel_topic", new_cmd_vel_topic);

  if (!cmd_pub_ || new_cmd_vel_topic != cmd_vel_topic_) {
    cmd_vel_topic_ = new_cmd_vel_topic;
    cmd_pub_ =
        node_->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);
  }

  if (!scan_sub_ || new_scan_topic != scan_topic_) {
    scan_topic_ = new_scan_topic;
    scan_sub_ = node_->create_subscription<sensor_msgs::msg::LaserScan>(
        scan_topic_, rclcpp::SensorDataQoS(),
        std::bind(&RefineShelfGeometry::scanCallback, this,
                  std::placeholders::_1));
  }
}

void RefineShelfGeometry::scanCallback(
    const sensor_msgs::msg::LaserScan::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(scan_mutex_);
  latest_scan_ = *msg;
  have_scan_ = true;
}

void RefineShelfGeometry::publishCmd(double linear_x, double angular_z) {
  if (!cmd_pub_) {
    return;
  }

  geometry_msgs::msg::Twist cmd;
  cmd.linear.x = linear_x;
  cmd.angular.z = angular_z;
  cmd_pub_->publish(cmd);
}

void RefineShelfGeometry::stopRobot() { publishCmd(0.0, 0.0); }

BT::NodeStatus RefineShelfGeometry::onStart() {
  initializeRosNode();
  ensureInterfaces();

  getInput("target_frame", target_frame_);
  getInput("front_window_start_idx", front_window_start_idx_);
  getInput("front_window_end_idx", front_window_end_idx_);
  getInput("expected_leg_spacing", expected_leg_spacing_);
  getInput("leg_spacing_tolerance", leg_spacing_tolerance_);
  getInput("cluster_gap_tolerance", cluster_gap_tolerance_);
  getInput("intensity_threshold", intensity_threshold_);
  getInput("max_detection_range", max_detection_range_);
  getInput("min_cluster_points", min_cluster_points_);
  getInput("min_consecutive_hits", min_consecutive_hits_);
  getInput("stable_hit_distance_tol", stable_hit_distance_tol_);
  getInput("stop_settle_time", stop_settle_time_);
  getInput("wait_duration", wait_duration_);
  getInput("max_scan_age", max_scan_age_);

  getInput("side_window_size", side_window_size_);
  getInput("side_range_cap", side_range_cap_);
  getInput("side_free_min_diff", side_free_min_diff_);

  if (expected_leg_spacing_ <= 0.0) {
    RCLCPP_ERROR(node_->get_logger(),
                 "RefineShelfGeometry: expected_leg_spacing must be > 0");
    return BT::NodeStatus::FAILURE;
  }

  if (leg_spacing_tolerance_ <= 0.0) {
    RCLCPP_ERROR(node_->get_logger(),
                 "RefineShelfGeometry: leg_spacing_tolerance must be > 0");
    return BT::NodeStatus::FAILURE;
  }

  if (cluster_gap_tolerance_ <= 0.0) {
    RCLCPP_ERROR(node_->get_logger(),
                 "RefineShelfGeometry: cluster_gap_tolerance must be > 0");
    return BT::NodeStatus::FAILURE;
  }

  if (max_detection_range_ <= 0.0) {
    RCLCPP_ERROR(node_->get_logger(),
                 "RefineShelfGeometry: max_detection_range must be > 0");
    return BT::NodeStatus::FAILURE;
  }

  if (min_cluster_points_ < 1) {
    RCLCPP_ERROR(node_->get_logger(),
                 "RefineShelfGeometry: min_cluster_points must be >= 1");
    return BT::NodeStatus::FAILURE;
  }

  if (min_consecutive_hits_ < 1) {
    RCLCPP_ERROR(node_->get_logger(),
                 "RefineShelfGeometry: min_consecutive_hits must be >= 1");
    return BT::NodeStatus::FAILURE;
  }

  if (stable_hit_distance_tol_ <= 0.0) {
    RCLCPP_ERROR(node_->get_logger(),
                 "RefineShelfGeometry: stable_hit_distance_tol must be > 0");
    return BT::NodeStatus::FAILURE;
  }

  if (wait_duration_ <= 0.0) {
    RCLCPP_ERROR(node_->get_logger(),
                 "RefineShelfGeometry: wait_duration must be > 0");
    return BT::NodeStatus::FAILURE;
  }

  if (side_window_size_ < 1) {
    RCLCPP_ERROR(node_->get_logger(),
                 "RefineShelfGeometry: side_window_size must be >= 1");
    return BT::NodeStatus::FAILURE;
  }

  if (side_range_cap_ <= 0.0) {
    RCLCPP_ERROR(node_->get_logger(),
                 "RefineShelfGeometry: side_range_cap must be > 0");
    return BT::NodeStatus::FAILURE;
  }

  if (side_free_min_diff_ < 0.0) {
    RCLCPP_ERROR(node_->get_logger(),
                 "RefineShelfGeometry: side_free_min_diff must be >= 0");
    return BT::NodeStatus::FAILURE;
  }

  accepted_hits_ = 0;
  have_last_accepted_center_ = false;
  have_last_processed_scan_ = false;

  start_time_ = node_->now();
  settle_deadline_ =
      start_time_ + rclcpp::Duration::from_seconds(stop_settle_time_);

  stopRobot();

  RCLCPP_INFO(node_->get_logger(),
              "RefineShelfGeometry: started. window=%d..%d spacing=%.3f±%.3f "
              "gap_tol=%.3f min_cluster_points=%d min_consecutive_hits=%d "
              "stable_tol=%.3f timeout=%.2f side_window=%d side_cap=%.2f "
              "side_min_diff=%.2f",
              front_window_start_idx_, front_window_end_idx_,
              expected_leg_spacing_, leg_spacing_tolerance_,
              cluster_gap_tolerance_, min_cluster_points_,
              min_consecutive_hits_, stable_hit_distance_tol_, wait_duration_,
              side_window_size_, side_range_cap_, side_free_min_diff_);

  return BT::NodeStatus::RUNNING;
}

bool RefineShelfGeometry::detectBestShelfGeometry(
    const sensor_msgs::msg::LaserScan &scan,
    geometry_msgs::msg::Point &center_out,
    geometry_msgs::msg::Point &left_leg_out,
    geometry_msgs::msg::Point &right_leg_out, double &confidence_out,
    double &spacing_out, std::string &reason_out) {
  ShelfDetectorParams params;
  params.expected_leg_spacing = expected_leg_spacing_;
  params.leg_spacing_tolerance = leg_spacing_tolerance_;
  params.cluster_gap_tolerance = cluster_gap_tolerance_;
  params.intensity_threshold = intensity_threshold_;
  params.max_detection_range = max_detection_range_;
  params.min_cluster_points = min_cluster_points_;

  const auto clusters = scan_utils::extractClustersFromIndexWindow(
      scan, front_window_start_idx_, front_window_end_idx_, params);

  if (clusters.size() < 2) {
    reason_out = "not enough clusters in front window (found=" +
                 std::to_string(clusters.size()) + " need>=2)";
    return false;
  }

  TransformPointOptions tf_options;
  tf_options.exact_timeout_sec = 0.05;
  tf_options.allow_latest_fallback = true;
  tf_options.fallback_timeout_sec = 0.05;
  tf_options.warn_on_fallback = false;

  bool found = false;
  double best_score = -std::numeric_limits<double>::infinity();
  std::string last_tf_reason = "all candidate transforms failed";

  const rclcpp::Time transform_time(scan.header.stamp);

  for (std::size_t i = 0; i < clusters.size(); ++i) {
    for (std::size_t j = i + 1; j < clusters.size(); ++j) {
      const auto &first = clusters[i];
      const auto &second = clusters[j];

      const double dx = first.centroid_laser.x - second.centroid_laser.x;
      const double dy = first.centroid_laser.y - second.centroid_laser.y;
      const double spacing = std::hypot(dx, dy);
      const double spacing_error = std::abs(spacing - expected_leg_spacing_);

      if (spacing_error > leg_spacing_tolerance_) {
        continue;
      }

      const Point2D center_laser{
          0.5 * (first.centroid_laser.x + second.centroid_laser.x),
          0.5 * (first.centroid_laser.y + second.centroid_laser.y)};

      // We are in predock and looking at the shelf from the front.
      // "Left" and "right" must be defined in laser frame, not in map frame.
      const Point2D &left_laser =
          (first.centroid_laser.y >= second.centroid_laser.y)
              ? first.centroid_laser
              : second.centroid_laser;

      const Point2D &right_laser =
          (first.centroid_laser.y >= second.centroid_laser.y)
              ? second.centroid_laser
              : first.centroid_laser;

      geometry_msgs::msg::Point center_target;
      geometry_msgs::msg::Point left_target;
      geometry_msgs::msg::Point right_target;

      if (!scan_utils::transformPointToFrame(
              *tf_buffer_, scan.header.frame_id, target_frame_, transform_time,
              center_laser, center_target, node_->get_logger(), tf_options)) {
        last_tf_reason = "failed to transform center";
        continue;
      }

      if (!scan_utils::transformPointToFrame(
              *tf_buffer_, scan.header.frame_id, target_frame_, transform_time,
              left_laser, left_target, node_->get_logger(), tf_options)) {
        last_tf_reason = "failed to transform left leg";
        continue;
      }

      if (!scan_utils::transformPointToFrame(
              *tf_buffer_, scan.header.frame_id, target_frame_, transform_time,
              right_laser, right_target, node_->get_logger(), tf_options)) {
        last_tf_reason = "failed to transform right leg";
        continue;
      }

      // Prefer well-centered detections when the robot already faces the shelf.
      const double centering_penalty = std::abs(center_laser.y);
      const double depth_diff =
          std::abs(first.centroid_laser.x - second.centroid_laser.x);

      // At predock, the two front legs should be roughly left/right of the
      // robot, not both on the same side.
      if (!(left_laser.y > 0.0 && right_laser.y < 0.0)) {
        continue;
      }

      // Shelf front should be roughly perpendicular to robot heading.
      // If this is too strict, start with 0.20.
      if (depth_diff > 0.18) {
        continue;
      }

      // Do not allow a very off-center shelf pair to win.
      if (std::abs(center_laser.y) > 0.18) {
        continue;
      }

      // Use intensity only as a small capped bonus, not the main discriminator.
      const double points_bonus =
          std::min(12.0, 0.5 * static_cast<double>(first.num_points +
                                                   second.num_points));

      const double score =
          100.0 -
          120.0 * (spacing_error / std::max(leg_spacing_tolerance_, 1e-6)) -
          180.0 * std::abs(center_laser.y) - 80.0 * depth_diff + points_bonus;

      if (!found || score > best_score) {
        found = true;
        best_score = score;
        confidence_out = score;
        spacing_out = spacing;
        center_out = center_target;
        left_leg_out = left_target;
        right_leg_out = right_target;
      }
    }
  }

  if (!found) {
    reason_out = "no valid pair inside front window (clusters=" +
                 std::to_string(clusters.size()) +
                 ", expected=" + std::to_string(expected_leg_spacing_) +
                 ", tol=" + std::to_string(leg_spacing_tolerance_) + ", " +
                 last_tf_reason + ")";
    return false;
  }

  reason_out.clear();
  return true;
}

BT::NodeStatus RefineShelfGeometry::onRunning() {
  stopRobot();

  const auto now = node_->now();

  if (now < settle_deadline_) {
    return BT::NodeStatus::RUNNING;
  }

  if ((now - start_time_).seconds() > wait_duration_) {
    RCLCPP_WARN(node_->get_logger(), "RefineShelfGeometry: refinement timeout");
    return BT::NodeStatus::FAILURE;
  }

  sensor_msgs::msg::LaserScan scan;
  {
    std::lock_guard<std::mutex> lock(scan_mutex_);
    if (!have_scan_) {
      return BT::NodeStatus::RUNNING;
    }
    scan = latest_scan_;
  }

  const rclcpp::Time scan_stamp(scan.header.stamp);

  if (have_last_processed_scan_ && scan_stamp == last_processed_scan_stamp_) {
    return BT::NodeStatus::RUNNING;
  }

  if (max_scan_age_ > 0.0) {
    const double scan_age = (now - scan_stamp).seconds();
    if (scan_age > max_scan_age_) {
      RCLCPP_WARN_THROTTLE(
          node_->get_logger(), *node_->get_clock(), 1000,
          "RefineShelfGeometry: waiting for fresh scan, age=%.3f > %.3f",
          scan_age, max_scan_age_);
      return BT::NodeStatus::RUNNING;
    }
  }

  last_processed_scan_stamp_ = scan_stamp;
  have_last_processed_scan_ = true;

  geometry_msgs::msg::Point center;
  geometry_msgs::msg::Point left_leg;
  geometry_msgs::msg::Point right_leg;
  double confidence = 0.0;
  double spacing = 0.0;
  std::string reason;

  if (!detectBestShelfGeometry(scan, center, left_leg, right_leg, confidence,
                               spacing, reason)) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                         "RefineShelfGeometry: rejected scan - %s",
                         reason.c_str());
    accepted_hits_ = 0;
    have_last_accepted_center_ = false;
    return BT::NodeStatus::RUNNING;
  }

  if (!have_last_accepted_center_) {
    last_accepted_center_ = center;
    have_last_accepted_center_ = true;
    accepted_hits_ = 1;
  } else {
    const double delta = scan_utils::distance2D(center, last_accepted_center_);

    if (delta <= stable_hit_distance_tol_) {
      last_accepted_center_ = center;
      accepted_hits_ += 1;
    } else {
      last_accepted_center_ = center;
      accepted_hits_ = 1;
    }
  }

  RCLCPP_INFO(node_->get_logger(),
              "RefineShelfGeometry: accepted hit %d/%d, center=(%.3f, %.3f), "
              "spacing=%.3f, conf=%.2f",
              accepted_hits_, min_consecutive_hits_, center.x, center.y,
              spacing, confidence);

  if (accepted_hits_ < min_consecutive_hits_) {
    return BT::NodeStatus::RUNNING;
  }

  setOutput("shelf_center", center);
  setOutput("shelf_left_leg", left_leg);
  setOutput("shelf_right_leg", right_leg);

  const double vx = right_leg.x - left_leg.x;
  const double vy = right_leg.y - left_leg.y;
  const double norm = std::hypot(vx, vy);

  if (norm < 1e-6) {
    RCLCPP_ERROR(node_->get_logger(),
                 "RefineShelfGeometry: shelf legs too close to compute yaw");
    return BT::NodeStatus::FAILURE;
  }

  const double tx = vx / norm;
  const double ty = vy / norm;

  // Tangent: along shelf front
  const double shelf_tangent_yaw = std::atan2(ty, tx);

  // Normal candidates
  double nx = -ty;
  double ny = tx;

  // Choose normal pointing from robot toward shelf center
  SimplePose2D robot_pose;
  if (!scan_utils::getRobotPoseInFrame(*tf_buffer_, target_frame_, robot_pose,
                                       node_->get_logger())) {
    RCLCPP_ERROR(
        node_->get_logger(),
        "RefineShelfGeometry: failed to get robot pose for normal yaw");
    return BT::NodeStatus::FAILURE;
  }

  const double rx = center.x - robot_pose.x;
  const double ry = center.y - robot_pose.y;

  // If normal points away from shelf center as seen from robot, flip it
  if ((nx * rx + ny * ry) < 0.0) {
    nx = -nx;
    ny = -ny;
  }

  const double shelf_normal_yaw = std::atan2(ny, nx);

  setOutput("shelf_tangent_yaw", shelf_tangent_yaw);
  setOutput("shelf_normal_yaw", shelf_normal_yaw);

  const auto side_decision = scan_utils::detectPreferredBackupTurnSide(
      scan, side_window_size_, side_range_cap_, side_free_min_diff_);

  int backup_turn_sign = 0;
  double left_free_score = 0.0;
  double right_free_score = 0.0;

  if (!side_decision.valid) {
    RCLCPP_WARN(
        node_->get_logger(),
        "RefineShelfGeometry: side-clearance evaluation unavailable: %s",
        side_decision.reason.c_str());
  } else {
    backup_turn_sign = side_decision.preferred_turn_sign;
    left_free_score = side_decision.left_score;
    right_free_score = side_decision.right_score;
  }

  setOutput("backup_turn_sign", backup_turn_sign);
  setOutput("left_free_score", left_free_score);
  setOutput("right_free_score", right_free_score);

  RCLCPP_INFO(node_->get_logger(),
              "RefineShelfGeometry: free-side scores left=%.3f right=%.3f "
              "backup_turn_sign=%d",
              left_free_score, right_free_score, backup_turn_sign);

  RCLCPP_INFO(node_->get_logger(),
              "RefineShelfGeometry: SUCCESS. center=(%.3f, %.3f), "
              "left=(%.3f, %.3f), right=(%.3f, %.3f), spacing=%.3f, "
              "tangent_yaw=%.3f, normal_yaw=%.3f",
              center.x, center.y, left_leg.x, left_leg.y, right_leg.x,
              right_leg.y, spacing, shelf_tangent_yaw, shelf_normal_yaw);

  return BT::NodeStatus::SUCCESS;
}

void RefineShelfGeometry::onHalted() {
  stopRobot();
  RCLCPP_WARN(node_->get_logger(), "RefineShelfGeometry: HALTED");
}

} // namespace rb1_bt

BT_REGISTER_NODES(factory) {
  factory.registerNodeType<rb1_bt::RefineShelfGeometry>("RefineShelfGeometry");
}