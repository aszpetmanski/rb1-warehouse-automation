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

  struct RawCluster {
    std::vector<Point2D> points;
    double intensity_sum{0.0};
    int intensity_count{0};
  };

  const bool has_intensity = !scan.intensities.empty();

  RawCluster current_cluster;
  std::optional<Point2D> previous_point;

  auto flushCluster = [&]() {
    if (static_cast<int>(current_cluster.points.size()) >=
        params.min_cluster_points) {
      ScanCluster cluster;

      double sum_x = 0.0;
      double sum_y = 0.0;
      for (const auto &point : current_cluster.points) {
        sum_x += point.x;
        sum_y += point.y;
      }

      cluster.centroid_laser.x = sum_x / current_cluster.points.size();
      cluster.centroid_laser.y = sum_y / current_cluster.points.size();
      cluster.num_points = static_cast<int>(current_cluster.points.size());
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
      current_cluster.points.push_back(current_point);

      if (has_intensity && index < static_cast<int>(scan.intensities.size())) {
        current_cluster.intensity_sum += scan.intensities[index];
        current_cluster.intensity_count++;
      }

      previous_point = current_point;
      continue;
    }

    const double gap = distance2D(current_point, *previous_point);
    if (gap > params.cluster_gap_tolerance) {
      flushCluster();
    }

    current_cluster.points.push_back(current_point);

    if (has_intensity && index < static_cast<int>(scan.intensities.size())) {
      current_cluster.intensity_sum += scan.intensities[index];
      current_cluster.intensity_count++;
    }

    previous_point = current_point;
  }

  flushCluster();
  return output_clusters;
}

// Convenience overload that transforms a 2D point using default TF options.
bool transformPointToFrame(const tf2_ros::Buffer &tf_buffer,
                           const std::string &source_frame,
                           const std::string &target_frame,
                           const rclcpp::Time &stamp,
                           const Point2D &input_point,
                           geometry_msgs::msg::Point &output_point,
                           const rclcpp::Logger &logger) {
  TransformPointOptions options;
  return transformPointToFrame(tf_buffer, source_frame, target_frame, stamp,
                               input_point, output_point, logger, options);
}

// Transforms a 2D point from one frame into another.
bool transformPointToFrame(
    const tf2_ros::Buffer &tf_buffer, const std::string &source_frame,
    const std::string &target_frame, const rclcpp::Time &stamp,
    const Point2D &input_point, geometry_msgs::msg::Point &output_point,
    const rclcpp::Logger &logger, const TransformPointOptions &options) {
  geometry_msgs::msg::PointStamped in_point;
  geometry_msgs::msg::PointStamped out_point;

  in_point.header.frame_id = source_frame;
  in_point.header.stamp = stamp;
  in_point.point.x = input_point.x;
  in_point.point.y = input_point.y;
  in_point.point.z = 0.0;

  if (source_frame == target_frame) {
    output_point = in_point.point;
    return true;
  }

  try {
    out_point =
        tf_buffer.transform(in_point, target_frame,
                            tf2::durationFromSec(options.exact_timeout_sec));
    output_point = out_point.point;
    return true;
  } catch (const tf2::TransformException &) {
    // fall through to latest-transform fallback if enabled
  }

  if (!options.allow_latest_fallback) {
    RCLCPP_WARN(logger,
                "transformPointToFrame failed from '%s' to '%s' at exact time",
                source_frame.c_str(), target_frame.c_str());
    return false;
  }

  try {
    in_point.header.stamp = builtin_interfaces::msg::Time();

    out_point =
        tf_buffer.transform(in_point, target_frame,
                            tf2::durationFromSec(options.fallback_timeout_sec));
    output_point = out_point.point;

    if (options.warn_on_fallback) {
      RCLCPP_WARN_THROTTLE(
          logger, *rclcpp::Clock::make_shared(), 2000,
          "transformPointToFrame: exact-time TF unavailable for %s -> %s, "
          "used latest-transform fallback",
          source_frame.c_str(), target_frame.c_str());
    }

    return true;
  } catch (const tf2::TransformException &ex) {
    RCLCPP_WARN(logger, "transformPointToFrame failed from '%s' to '%s': %s",
                source_frame.c_str(), target_frame.c_str(), ex.what());
    return false;
  }
}

/*
'''
    // Finds the closest transformed cluster in the given scan window.
    bool
    findClosestTransformedClusterInWindow(
        const sensor_msgs::msg::LaserScan &scan, int start_idx, int end_idx,
        const ShelfDetectorParams &params, const tf2_ros::Buffer &tf_buffer,
        const std::string &target_frame, const rclcpp::Time &transform_time,
        const geometry_msgs::msg::Point &reference_point_target_frame,
        double max_match_distance, TransformedClusterMatch &match_out,
        const rclcpp::Logger &logger, const TransformPointOptions &tf_options) {
  const auto clusters =
      extractClustersFromIndexWindow(scan, start_idx, end_idx, params);

  if (clusters.empty()) {
    return false;
  }

  double best_dist = std::numeric_limits<double>::infinity();
  bool found = false;

  for (const auto &cluster : clusters) {
    geometry_msgs::msg::Point transformed;
    if (!transformPointToFrame(tf_buffer, scan.header.frame_id, target_frame,
                               transform_time, cluster.centroid_laser,
                               transformed, logger, tf_options)) {
      continue;
    }

    const double dist = distance2D(transformed, reference_point_target_frame);
    if (dist < best_dist) {
      best_dist = dist;
      match_out.cluster = cluster;
      match_out.centroid_target_frame = transformed;
      match_out.distance_to_ref = dist;
      found = true;
    }
  }

  if (!found) {
    return false;
  }

  return best_dist <= max_match_distance;
}

'''
*/

// Finds the best shelf-like pair of clusters in the given scan window.
std::optional<ShelfCandidateDetection> detectShelfCandidateInWindow(
    const sensor_msgs::msg::LaserScan &scan, int start_idx, int end_idx,
    const ShelfDetectorParams &params, const tf2_ros::Buffer &tf_buffer,
    const std::string &target_frame, const rclcpp::Time &transform_time,
    const rclcpp::Logger &logger) {
  const auto clusters =
      extractClustersFromIndexWindow(scan, start_idx, end_idx, params);

  if (clusters.size() < 2) {
    return std::nullopt;
  }

  double best_score = -std::numeric_limits<double>::infinity();
  std::optional<ShelfCandidateDetection> best_detection;

  TransformPointOptions tf_options;
  tf_options.exact_timeout_sec = 0.05;
  tf_options.allow_latest_fallback = false;
  tf_options.fallback_timeout_sec = 0.05;
  tf_options.warn_on_fallback = false;

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

      geometry_msgs::msg::Point first_target;
      geometry_msgs::msg::Point second_target;
      geometry_msgs::msg::Point center_target;

      if (!transformPointToFrame(tf_buffer, scan.header.frame_id, target_frame,
                                 transform_time, first.centroid_laser,
                                 first_target, logger, tf_options)) {
        continue;
      }

      if (!transformPointToFrame(tf_buffer, scan.header.frame_id, target_frame,
                                 transform_time, second.centroid_laser,
                                 second_target, logger, tf_options)) {
        continue;
      }

      if (!transformPointToFrame(tf_buffer, scan.header.frame_id, target_frame,
                                 transform_time, center_laser, center_target,
                                 logger, tf_options)) {
        continue;
      }

      ShelfCandidateDetection detection;
      detection.valid = true;
      detection.center_target_frame = center_target;

      if (first_target.y >= second_target.y) {
        detection.left_leg_target_frame = first_target;
        detection.right_leg_target_frame = second_target;
      } else {
        detection.left_leg_target_frame = second_target;
        detection.right_leg_target_frame = first_target;
      }

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

} // namespace rb1_bt::scan_utils