#pragma once

#include <behaviortree_cpp_v3/basic_types.h>
#include <geometry_msgs/msg/point.hpp>

#include "rosidl_runtime_cpp/message_initialization.hpp"

#include <cctype>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace rb1_bt {

// Lightweight 2D pose used to pass patrol waypoints through the BT.
struct SimplePose2D {
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

// Simple 2D Cartesian point used by scan-processing helpers.
struct Point2D {
  double x{0.0};
  double y{0.0};
};

// Summary of a point cluster extracted from a laser scan window.
struct ScanCluster {
  Point2D centroid_laser;
  double mean_intensity{0.0};
  int num_points{0};
};

// Tunable parameters for shelf-leg detection from LaserScan data.
struct ShelfDetectorParams {
  double expected_leg_spacing{0.60};
  double leg_spacing_tolerance{0.08};
  double cluster_gap_tolerance{0.10};
  double intensity_threshold{1500.0};
  double max_detection_range{2.5};
  int min_cluster_points{2};
};

// Options for transforming points between TF frames.
struct TransformPointOptions {
  double exact_timeout_sec{0.05};
  bool allow_latest_fallback{false};
  double fallback_timeout_sec{0.05};
  bool warn_on_fallback{true};
};

/*
    // Result of matching a transformed cluster against a reference point.
    struct TransformedClusterMatch {
  ScanCluster cluster;
  geometry_msgs::msg::Point centroid_target_frame;
  double distance_to_ref{0.0};
};

// Candidate shelf detection represented in the target frame.
*/

// We do this weird shinanigans to supress build warning
struct ShelfCandidateDetection {
  geometry_msgs::msg::Point center_target_frame;
  geometry_msgs::msg::Point left_leg_target_frame;
  geometry_msgs::msg::Point right_leg_target_frame;
  double confidence;
  bool valid;

  ShelfCandidateDetection()
      : center_target_frame(rosidl_runtime_cpp::MessageInitialization::ALL),
        left_leg_target_frame(rosidl_runtime_cpp::MessageInitialization::ALL),
        right_leg_target_frame(rosidl_runtime_cpp::MessageInitialization::ALL),
        confidence(0.0), valid(false) {}
};

} // namespace rb1_bt

namespace BT {

// Returns a copy of the string with leading/trailing whitespace removed.
inline std::string trimCopy(const std::string &input) {
  std::size_t begin = 0;
  while (begin < input.size() &&
         std::isspace(static_cast<unsigned char>(input[begin]))) {
    ++begin;
  }

  std::size_t end = input.size();
  while (end > begin &&
         std::isspace(static_cast<unsigned char>(input[end - 1]))) {
    --end;
  }

  return input.substr(begin, end - begin);
}

// Parses a single pose from the format: "x,y,yaw".
template <> inline rb1_bt::SimplePose2D convertFromString(StringView str) {
  const std::string input(str.data(), str.size());
  std::stringstream ss(input);
  std::string item;
  std::vector<double> values;

  while (std::getline(ss, item, ',')) {
    const auto trimmed = trimCopy(item);
    if (!trimmed.empty()) {
      values.push_back(std::stod(trimmed));
    }
  }

  if (values.size() != 3) {
    throw RuntimeError(
        "SimplePose2D expects exactly 3 comma-separated values: x,y,yaw. Got: ",
        input);
  }

  rb1_bt::SimplePose2D pose;
  pose.x = values[0];
  pose.y = values[1];
  pose.yaw = values[2];
  return pose;
}

// Parses a list of poses from the format: "x,y,yaw; x,y,yaw; ...".
template <>
inline std::vector<rb1_bt::SimplePose2D> convertFromString(StringView str) {
  const std::string input(str.data(), str.size());
  std::stringstream ss(input);
  std::string pose_token;
  std::vector<rb1_bt::SimplePose2D> poses;

  while (std::getline(ss, pose_token, ';')) {
    const auto trimmed = trimCopy(pose_token);
    if (!trimmed.empty()) {
      poses.push_back(convertFromString<rb1_bt::SimplePose2D>(trimmed));
    }
  }

  if (poses.empty()) {
    throw RuntimeError(
        "Vector<SimplePose2D> parsing produced empty result. Input: ", input);
  }

  return poses;
}

} // namespace BT