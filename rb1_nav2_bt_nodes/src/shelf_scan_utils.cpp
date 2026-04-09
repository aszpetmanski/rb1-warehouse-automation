#include "rb1_nav2_bt_nodes/shelf_scan_utils.hpp"

#include <geometry_msgs/msg/point_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <vector>

namespace rb1_bt::scan_utils {

Point2D polarToCartesian(double range, double angle_rad) {
  Point2D point;
  point.x = range * std::cos(angle_rad);
  point.y = range * std::sin(angle_rad);
  return point;
}

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
    const double range = scan.ranges[index];

    if (!std::isfinite(range) || range < scan.range_min ||
        range > scan.range_max || range > params.max_detection_range) {
      flushCluster();
      continue;
    }

    if (has_intensity) {
      if (index >= static_cast<int>(scan.intensities.size()) ||
          scan.intensities[index] < params.intensity_threshold) {
        flushCluster();
        continue;
      }
    }

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

    const double gap = std::hypot(current_point.x - previous_point->x,
                                  current_point.y - previous_point->y);

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

bool transformPointToFrame(const tf2_ros::Buffer &tf_buffer,
                           const std::string &source_frame,
                           const std::string &target_frame,
                           const rclcpp::Time &stamp,
                           const Point2D &input_point,
                           geometry_msgs::msg::Point &output_point,
                           const rclcpp::Logger &logger) {
  geometry_msgs::msg::PointStamped in_point;
  geometry_msgs::msg::PointStamped out_point;

  in_point.header.frame_id = source_frame;
  in_point.header.stamp = stamp;
  in_point.point.x = input_point.x;
  in_point.point.y = input_point.y;
  in_point.point.z = 0.0;

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
                                 first_target, logger)) {
        continue;
      }

      if (!transformPointToFrame(tf_buffer, scan.header.frame_id, target_frame,
                                 transform_time, second.centroid_laser,
                                 second_target, logger)) {
        continue;
      }

      if (!transformPointToFrame(tf_buffer, scan.header.frame_id, target_frame,
                                 transform_time, center_laser, center_target,
                                 logger)) {
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