#include "rb1_nav2_bt_nodes/shelf_scan_utils.hpp"

#include <geometry_msgs/msg/point_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <vector>

namespace rb1_bt::scan_utils {

namespace {

constexpr double kHalfPi = 1.5707963267948966;
constexpr int kMinSideSamples = 5;

// Use only the brightest local returns inside each cluster to estimate the
// reflective leg center. This is per-cluster, so one globally brighter side
// does not pull the shelf center by itself.
constexpr double kBrightCoreFraction = 0.25; // brightest 35% of cluster points
constexpr int kMinBrightCorePoints = 3;
constexpr double kBrightWeightGain = 2.0; // max weight = 1 + gain

// Checks whether a single LaserScan reading can be used for
// clustering/detection.
bool isMeasurementValid(const sensor_msgs::msg::LaserScan &scan, int index,
                        const ShelfDetectorParams &params) {
  if (index < 0 || index >= static_cast<int>(scan.ranges.size())) {
    return false;
  }

  const double range = scan.ranges[index];
  if (!std::isfinite(range)) {
    return false;
  }

  if (range < scan.range_min || range > scan.range_max) {
    return false;
  }

  if (range > params.max_detection_range) {
    return false;
  }

  if (!scan.intensities.empty()) {
    if (index >= static_cast<int>(scan.intensities.size())) {
      return false;
    }
    if (scan.intensities[index] < params.intensity_threshold) {
      return false;
    }
  }

  return true;
}

double median(std::vector<double> values) {
  if (values.empty()) {
    return 0.0;
  }

  std::sort(values.begin(), values.end());
  const std::size_t n = values.size();

  if ((n % 2) == 1) {
    return values[n / 2];
  }

  return 0.5 * (values[(n / 2) - 1] + values[n / 2]);
}

double percentile(std::vector<double> values, double p) {
  if (values.empty()) {
    return 0.0;
  }

  p = std::clamp(p, 0.0, 1.0);
  std::sort(values.begin(), values.end());

  const double idx = p * static_cast<double>(values.size() - 1);
  const std::size_t lo = static_cast<std::size_t>(std::floor(idx));
  const std::size_t hi = static_cast<std::size_t>(std::ceil(idx));

  if (lo == hi) {
    return values[lo];
  }

  const double t = idx - static_cast<double>(lo);
  return (1.0 - t) * values[lo] + t * values[hi];
}

int angleToNearestScanIndex(const sensor_msgs::msg::LaserScan &scan,
                            double angle_rad) {
  if (std::abs(scan.angle_increment) < 1e-12) {
    return -1;
  }

  const double raw = (angle_rad - static_cast<double>(scan.angle_min)) /
                     static_cast<double>(scan.angle_increment);

  return static_cast<int>(std::lround(raw));
}

std::vector<double> collectWindowRanges(const sensor_msgs::msg::LaserScan &scan,
                                        int center_idx, int window_size,
                                        double range_cap) {
  std::vector<double> values;

  if (scan.ranges.empty() || window_size <= 0) {
    return values;
  }

  const int n = static_cast<int>(scan.ranges.size());
  if (center_idx < 0 || center_idx >= n) {
    return values;
  }

  const int half = window_size / 2;
  const int start = std::max(0, center_idx - half);
  const int end = std::min(n - 1, center_idx + half);

  values.reserve(end - start + 1);

  for (int i = start; i <= end; ++i) {
    const float raw = scan.ranges[i];

    if (std::isnan(raw)) {
      continue;
    }

    if (std::isinf(raw)) {
      values.push_back(range_cap);
      continue;
    }

    const double r = static_cast<double>(raw);

    if (r < scan.range_min) {
      continue;
    }

    if (r > scan.range_max) {
      values.push_back(range_cap);
      continue;
    }

    values.push_back(std::min(r, range_cap));
  }

  return values;
}

} // namespace

// Builds a PoseStamped from a SimplePose2D waypoint.
geometry_msgs::msg::PoseStamped
buildPoseStampedFromWaypoint(const SimplePose2D &waypoint,
                             const std::string &frame_id,
                             const rclcpp::Time &stamp) {
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = frame_id;
  pose.header.stamp = stamp;

  pose.pose.position.x = waypoint.x;
  pose.pose.position.y = waypoint.y;
  pose.pose.position.z = 0.0;
  pose.pose.orientation = yawToQuaternion(waypoint.yaw);

  return pose;
}

// Converts a polar LiDAR measurement into a 2D Cartesian point.
Point2D polarToCartesian(double range, double angle_rad) {
  Point2D point;
  point.x = range * std::cos(angle_rad);
  point.y = range * std::sin(angle_rad);
  return point;
}

// Clamps a scan index window into valid bounds.
bool normalizeWindow(int requested_start_idx, int requested_end_idx,
                     int scan_size, int &normalized_start_idx,
                     int &normalized_end_idx) {
  if (scan_size <= 0) {
    return false;
  }

  const int max_idx = scan_size - 1;

  normalized_start_idx = std::clamp(requested_start_idx, 0, max_idx);
  normalized_end_idx = std::clamp(requested_end_idx, 0, max_idx);

  if (normalized_start_idx > normalized_end_idx) {
    std::swap(normalized_start_idx, normalized_end_idx);
  }

  return normalized_start_idx <= normalized_end_idx;
}

double normalizeAngle(double angle) {
  while (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }
  while (angle < -M_PI) {
    angle += 2.0 * M_PI;
  }
  return angle;
}

// Returns the 2D distance between two Point2D values.
double distance2D(const Point2D &a, const Point2D &b) {
  return std::hypot(a.x - b.x, a.y - b.y);
}

// Returns the 2D distance between two geometry points.
double distance2D(const geometry_msgs::msg::Point &a,
                  const geometry_msgs::msg::Point &b) {
  return std::hypot(a.x - b.x, a.y - b.y);
}

double distance2D(const SimplePose2D &a, const SimplePose2D &b) {
  return std::hypot(a.x - b.x, a.y - b.y);
}

double distance2D(const SimplePose2D &a, const geometry_msgs::msg::Point &b) {
  return std::hypot(a.x - b.x, a.y - b.y);
}

double distance2D(const geometry_msgs::msg::Point &a, const SimplePose2D &b) {
  return std::hypot(a.x - b.x, a.y - b.y);
}

// Computes the average of a list of geometry points.
geometry_msgs::msg::Point
averageGeometryPoints(const std::vector<geometry_msgs::msg::Point> &points) {
  geometry_msgs::msg::Point avg;
  avg.x = 0.0;
  avg.y = 0.0;
  avg.z = 0.0;

  if (points.empty()) {
    return avg;
  }

  for (const auto &p : points) {
    avg.x += p.x;
    avg.y += p.y;
    avg.z += p.z;
  }

  const double inv_n = 1.0 / static_cast<double>(points.size());
  avg.x *= inv_n;
  avg.y *= inv_n;
  avg.z *= inv_n;

  return avg;
}

// Computes midpoint of two geometry points.
geometry_msgs::msg::Point
midpointGeometryPoints(const geometry_msgs::msg::Point &a,
                       const geometry_msgs::msg::Point &b) {
  geometry_msgs::msg::Point midpoint;
  midpoint.x = 0.5 * (a.x + b.x);
  midpoint.y = 0.5 * (a.y + b.y);
  midpoint.z = 0.5 * (a.z + b.z);
  return midpoint;
}

// Builds a quaternion from yaw angle.
geometry_msgs::msg::Quaternion yawToQuaternion(double yaw) {
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw);
  return tf2::toMsg(q);
}

// Gets current robot origin position in target_frame by transforming
// (0,0,0) from robot_frame.
bool getRobotPoseInFrame(const tf2_ros::Buffer &tf_buffer,
                         const std::string &target_frame,
                         SimplePose2D &robot_pose_out,
                         const rclcpp::Logger &logger,
                         const std::string &robot_frame) {
  geometry_msgs::msg::PointStamped in_point;
  geometry_msgs::msg::PointStamped out_point;

  in_point.header.frame_id = robot_frame;
  in_point.header.stamp = builtin_interfaces::msg::Time();
  in_point.point.x = 0.0;
  in_point.point.y = 0.0;
  in_point.point.z = 0.0;

  try {
    if (robot_frame == target_frame) {
      robot_pose_out.x = 0.0;
      robot_pose_out.y = 0.0;
    } else {
      out_point = tf_buffer.transform(in_point, target_frame,
                                      tf2::durationFromSec(0.1));
      robot_pose_out.x = out_point.point.x;
      robot_pose_out.y = out_point.point.y;
    }

    const auto tf = tf_buffer.lookupTransform(target_frame, robot_frame,
                                              tf2::TimePointZero);
    robot_pose_out.yaw = tf2::getYaw(tf.transform.rotation);

    return true;
  } catch (const tf2::TransformException &ex) {
    RCLCPP_WARN(logger, "getRobotPoseInFrame failed from '%s' to '%s': %s",
                robot_frame.c_str(), target_frame.c_str(), ex.what());
    return false;
  }
}

// Extracts contiguous point clusters from a scan index window.
std::vector<ScanCluster>
extractClustersFromIndexWindow(const sensor_msgs::msg::LaserScan &scan,
                               int start_idx, int end_idx,
                               const ShelfDetectorParams &params) {
  std::vector<ScanCluster> output_clusters;

  int normalized_start = 0;
  int normalized_end = 0;
  if (!normalizeWindow(start_idx, end_idx, static_cast<int>(scan.ranges.size()),
                       normalized_start, normalized_end)) {
    return output_clusters;
  }

  struct ClusterSample {
    Point2D point;
    double intensity{0.0};
  };

  struct RawCluster {
    std::vector<ClusterSample> samples;
    double intensity_sum{0.0};
    int intensity_count{0};
  };

  const bool has_intensity = !scan.intensities.empty();

  auto computeGeometricCentroid =
      [](const std::vector<ClusterSample> &samples) -> Point2D {
    Point2D centroid;

    if (samples.empty()) {
      centroid.x = 0.0;
      centroid.y = 0.0;
      return centroid;
    }

    double sum_x = 0.0;
    double sum_y = 0.0;

    for (const auto &sample : samples) {
      sum_x += sample.point.x;
      sum_y += sample.point.y;
    }

    const double inv_n = 1.0 / static_cast<double>(samples.size());
    centroid.x = sum_x * inv_n;
    centroid.y = sum_y * inv_n;
    return centroid;
  };

  auto computeBrightCoreCentroid =
      [&](const RawCluster &raw_cluster) -> Point2D {
    if (!has_intensity ||
        static_cast<int>(raw_cluster.samples.size()) < kMinBrightCorePoints) {
      return computeGeometricCentroid(raw_cluster.samples);
    }

    auto bright_samples = raw_cluster.samples;

    std::sort(bright_samples.begin(), bright_samples.end(),
              [](const ClusterSample &a, const ClusterSample &b) {
                return a.intensity > b.intensity;
              });

    const int desired_keep = std::max(
        kMinBrightCorePoints,
        static_cast<int>(std::ceil(
            kBrightCoreFraction * static_cast<double>(bright_samples.size()))));

    const int keep_count =
        std::min(desired_keep, static_cast<int>(bright_samples.size()));

    bright_samples.resize(static_cast<std::size_t>(keep_count));

    const double max_i = bright_samples.front().intensity;
    const double min_i = bright_samples.back().intensity;
    const double span = std::max(1.0, max_i - min_i);

    double weighted_x = 0.0;
    double weighted_y = 0.0;
    double weight_sum = 0.0;

    for (const auto &sample : bright_samples) {
      const double normalized =
          std::clamp((sample.intensity - min_i) / span, 0.0, 1.0);

      // Per-cluster normalized weight.
      // This focuses the centroid on the local bright core without letting
      // one globally brighter shelf side dominate the final center.
      const double weight = 1.0 + kBrightWeightGain * normalized;

      weighted_x += weight * sample.point.x;
      weighted_y += weight * sample.point.y;
      weight_sum += weight;
    }

    if (weight_sum <= 1e-9) {
      return computeGeometricCentroid(raw_cluster.samples);
    }

    Point2D centroid;
    centroid.x = weighted_x / weight_sum;
    centroid.y = weighted_y / weight_sum;
    return centroid;
  };

  RawCluster current_cluster;
  std::optional<Point2D> previous_point;

  auto appendPoint = [&](int index, const Point2D &point) {
    double intensity = 0.0;

    if (has_intensity && index < static_cast<int>(scan.intensities.size())) {
      intensity = scan.intensities[index];
      current_cluster.intensity_sum += intensity;
      current_cluster.intensity_count++;
    }

    current_cluster.samples.push_back(ClusterSample{point, intensity});
  };

  auto flushCluster = [&]() {
    if (static_cast<int>(current_cluster.samples.size()) >=
        params.min_cluster_points) {
      ScanCluster cluster;

      cluster.centroid_laser = computeBrightCoreCentroid(current_cluster);
      cluster.num_points = static_cast<int>(current_cluster.samples.size());

      cluster.mean_intensity =
          (current_cluster.intensity_count > 0)
              ? current_cluster.intensity_sum / current_cluster.intensity_count
              : 0.0;

      output_clusters.push_back(cluster);
    }

    current_cluster = RawCluster{};
    previous_point.reset();
  };

  for (int index = normalized_start; index <= normalized_end; ++index) {
    if (!isMeasurementValid(scan, index, params)) {
      flushCluster();
      continue;
    }

    const double range = scan.ranges[index];
    const double angle =
        scan.angle_min + static_cast<double>(index) * scan.angle_increment;
    const Point2D current_point = polarToCartesian(range, angle);

    if (!previous_point.has_value()) {
      appendPoint(index, current_point);
      previous_point = current_point;
      continue;
    }

    const double gap = distance2D(current_point, *previous_point);
    if (gap > params.cluster_gap_tolerance) {
      flushCluster();
    }

    appendPoint(index, current_point);
    previous_point = current_point;
  }

  flushCluster();
  return output_clusters;
}

// Transforms a 2D point from one frame into another using the latest
// available TF. No exact-time lookup is attempted.
bool transformPointToFrame(const tf2_ros::Buffer &tf_buffer,
                           const std::string &source_frame,
                           const std::string &target_frame,
                           const Point2D &input_point,
                           geometry_msgs::msg::Point &output_point,
                           const rclcpp::Logger &logger) {
  geometry_msgs::msg::PointStamped in_point;
  geometry_msgs::msg::PointStamped out_point;

  in_point.header.frame_id = source_frame;
  in_point.header.stamp = builtin_interfaces::msg::Time();
  in_point.point.x = input_point.x;
  in_point.point.y = input_point.y;
  in_point.point.z = 0.0;

  if (source_frame == target_frame) {
    output_point = in_point.point;
    return true;
  }

  try {
    out_point =
        tf_buffer.transform(in_point, target_frame, tf2::durationFromSec(0.05));
    output_point = out_point.point;
    return true;
  } catch (const tf2::TransformException &ex) {
    RCLCPP_WARN(logger, "transformPointToFrame failed from '%s' to '%s': %s",
                source_frame.c_str(), target_frame.c_str(), ex.what());
    return false;
  }
}

// Legacy compatibility overload: stamp is ignored and latest TF is used.
bool transformPointToFrame(const tf2_ros::Buffer &tf_buffer,
                           const std::string &source_frame,
                           const std::string &target_frame,
                           const rclcpp::Time &stamp,
                           const Point2D &input_point,
                           geometry_msgs::msg::Point &output_point,
                           const rclcpp::Logger &logger) {
  (void)stamp;
  return transformPointToFrame(tf_buffer, source_frame, target_frame,
                               input_point, output_point, logger);
}

// Legacy compatibility overload: stamp/options are ignored and latest TF is
// used.
bool transformPointToFrame(
    const tf2_ros::Buffer &tf_buffer, const std::string &source_frame,
    const std::string &target_frame, const rclcpp::Time &stamp,
    const Point2D &input_point, geometry_msgs::msg::Point &output_point,
    const rclcpp::Logger &logger, const TransformPointOptions &options) {
  (void)stamp;
  (void)options;
  return transformPointToFrame(tf_buffer, source_frame, target_frame,
                               input_point, output_point, logger);
}

std::optional<ShelfCandidateDetection> detectShelfCandidateInWindow(
    const sensor_msgs::msg::LaserScan &scan, int start_idx, int end_idx,
    const ShelfDetectorParams &params, const tf2_ros::Buffer &tf_buffer,
    const std::string &target_frame, const rclcpp::Time &transform_time,
    const rclcpp::Logger &logger) {
  (void)transform_time; // intentionally ignored: always use latest TF

  const auto clusters =
      extractClustersFromIndexWindow(scan, start_idx, end_idx, params);

  if (clusters.size() < 2) {
    return std::nullopt;
  }

  double best_score = -std::numeric_limits<double>::infinity();
  std::optional<ShelfCandidateDetection> best_detection;

  for (std::size_t i = 0; i < clusters.size(); ++i) {
    for (std::size_t j = i + 1; j < clusters.size(); ++j) {
      const auto &first = clusters[i];
      const auto &second = clusters[j];

      const double dx = first.centroid_laser.x - second.centroid_laser.x;
      const double dy = first.centroid_laser.y - second.centroid_laser.y;
      const double spacing = std::hypot(dx, dy);
      const double spacing_error =
          std::abs(spacing - params.expected_leg_spacing);

      if (spacing_error > params.leg_spacing_tolerance) {
        continue;
      }

      const Point2D center_laser{
          0.5 * (first.centroid_laser.x + second.centroid_laser.x),
          0.5 * (first.centroid_laser.y + second.centroid_laser.y)};

      // IMPORTANT:
      // Left/right must be decided in laser frame, not target/map frame.
      const Point2D &left_laser =
          (first.centroid_laser.y >= second.centroid_laser.y)
              ? first.centroid_laser
              : second.centroid_laser;

      const Point2D &right_laser =
          (first.centroid_laser.y >= second.centroid_laser.y)
              ? second.centroid_laser
              : first.centroid_laser;

      geometry_msgs::msg::Point left_target;
      geometry_msgs::msg::Point right_target;
      geometry_msgs::msg::Point center_target;

      if (!transformPointToFrame(tf_buffer, scan.header.frame_id, target_frame,
                                 left_laser, left_target, logger)) {
        continue;
      }

      if (!transformPointToFrame(tf_buffer, scan.header.frame_id, target_frame,
                                 right_laser, right_target, logger)) {
        continue;
      }

      if (!transformPointToFrame(tf_buffer, scan.header.frame_id, target_frame,
                                 center_laser, center_target, logger)) {
        continue;
      }

      ShelfCandidateDetection detection;
      detection.valid = true;
      detection.center_target_frame = center_target;
      detection.left_leg_target_frame = left_target;
      detection.right_leg_target_frame = right_target;

      detection.confidence =
          100.0 -
          100.0 *
              (spacing_error / std::max(params.leg_spacing_tolerance, 1e-6)) +
          0.01 * (first.mean_intensity + second.mean_intensity) +
          2.0 * (first.num_points + second.num_points);

      if (detection.confidence > best_score) {
        best_score = detection.confidence;
        best_detection = detection;
      }
    }
  }

  return best_detection;
}

SideClearanceDecision
detectPreferredBackupTurnSide(const sensor_msgs::msg::LaserScan &scan,
                              int side_window_size, double side_range_cap,
                              double min_preference_diff) {
  SideClearanceDecision result;

  if (scan.ranges.empty()) {
    result.reason = "empty scan";
    return result;
  }

  if (side_window_size < 1) {
    result.reason = "side_window_size must be >= 1";
    return result;
  }

  if (side_range_cap <= 0.0) {
    result.reason = "side_range_cap must be > 0";
    return result;
  }

  if (std::abs(scan.angle_increment) < 1e-12) {
    result.reason = "invalid angle_increment";
    return result;
  }

  const int left_center_idx = angleToNearestScanIndex(scan, +kHalfPi);
  const int right_center_idx = angleToNearestScanIndex(scan, -kHalfPi);

  const int n = static_cast<int>(scan.ranges.size());
  if (left_center_idx < 0 || left_center_idx >= n || right_center_idx < 0 ||
      right_center_idx >= n) {
    result.reason = "scan does not cover both robot sides (+/-90deg)";
    return result;
  }

  const auto left_values = collectWindowRanges(
      scan, left_center_idx, side_window_size, side_range_cap);
  const auto right_values = collectWindowRanges(
      scan, right_center_idx, side_window_size, side_range_cap);

  if (static_cast<int>(left_values.size()) < kMinSideSamples) {
    result.reason = "not enough valid left-side samples";
    return result;
  }

  if (static_cast<int>(right_values.size()) < kMinSideSamples) {
    result.reason = "not enough valid right-side samples";
    return result;
  }

  result.left_score = percentile(left_values, 0.25);
  result.right_score = percentile(right_values, 0.25);

  const double diff = result.left_score - result.right_score;

  if (std::abs(diff) < min_preference_diff) {
    result.preferred_turn_sign = 0;
  } else {
    result.preferred_turn_sign = (diff > 0.0) ? +1 : -1;
  }

  result.valid = true;
  result.reason = "ok";
  return result;
}

} // namespace rb1_bt::scan_utils