#pragma once

#include "rb1_nav2_bt_nodes/patrol_types.hpp"

#include <behaviortree_cpp_v3/action_node.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <geometry_msgs/msg/point.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

namespace rb1_bt {

class PatrolUntilCandidate : public BT::StatefulActionNode {
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  PatrolUntilCandidate(const std::string &name,
                       const BT::NodeConfiguration &config);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;

  BT::NodeStatus onRunning() override;

  void onHalted() override;

private:
  bool loadPorts();

  void resetRunState();

  bool sendCurrentWaypointGoal();

  bool advanceToNextWaypoint();

  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);

  std::optional<ShelfCandidateDetection> detectCandidateFromLatestScan();

  bool updateStableHit(const ShelfCandidateDetection &detection);

  bool initializeRosResources();

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  std::mutex scan_mutex_;
  sensor_msgs::msg::LaserScan::SharedPtr last_scan_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  std::vector<SimplePose2D> patrol_waypoints_;
  std::size_t current_waypoint_idx_{0};

  int waypoint_failures_{0};
  int max_waypoint_failures_{3};

  std::string nav_action_name_{"navigate_to_pose"};
  std::string scan_topic_{"/scan"};
  std::string target_frame_{"map"};

  int left_window_start_idx_{0};
  int left_window_end_idx_{0};
  int right_window_start_idx_{0};
  int right_window_end_idx_{0};

  ShelfDetectorParams detector_params_;

  int min_consecutive_hits_{3};
  double stable_hit_distance_tol_{0.25};

  std::shared_future<typename GoalHandleNav::SharedPtr> goal_handle_future_;
  GoalHandleNav::SharedPtr goal_handle_;
  std::shared_future<typename GoalHandleNav::WrappedResult> result_future_;

  ShelfCandidateDetection stable_candidate_;
  ShelfCandidateDetection last_candidate_;
  bool has_last_candidate_{false};
  int consecutive_hits_{0};
  std::string last_candidate_window_;
  std::string stable_candidate_window_;
};

} // namespace rb1_bt