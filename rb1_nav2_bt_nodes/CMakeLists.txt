cmake_minimum_required(VERSION 3.8)
project(rb1_nav2_bt_nodes)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)

find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)

find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

find_package(behaviortree_cpp_v3 REQUIRED)
find_package(nav2_util REQUIRED)

include_directories(include)

# -------------------------------
# Shared utility library
# -------------------------------
add_library(rb1_shelf_scan_utils SHARED
  src/shelf_scan_utils.cpp
)

target_include_directories(rb1_shelf_scan_utils
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(rb1_shelf_scan_utils
  rclcpp
  geometry_msgs
  sensor_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  behaviortree_cpp_v3
)

target_compile_features(rb1_shelf_scan_utils PUBLIC cxx_std_17)

# -------------------------------
# Plugin 1: PatrolUntilCandidate
# -------------------------------
add_library(rb1_patrol_until_candidate_bt_node SHARED
  src/patrol_until_candidate.cpp
)

target_include_directories(rb1_patrol_until_candidate_bt_node
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(rb1_patrol_until_candidate_bt_node
  rclcpp
  rclcpp_action
  geometry_msgs
  sensor_msgs
  nav2_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  behaviortree_cpp_v3
  nav2_util
)

target_link_libraries(rb1_patrol_until_candidate_bt_node
  rb1_shelf_scan_utils
)

target_compile_features(rb1_patrol_until_candidate_bt_node PUBLIC cxx_std_17)
target_compile_definitions(rb1_patrol_until_candidate_bt_node PRIVATE BT_PLUGIN_EXPORT)

# -------------------------------
# Plugin 2: InitShelfMission
# -------------------------------
add_library(rb1_init_shelf_mission_bt_node SHARED
  src/init_shelf_mission.cpp
)

target_include_directories(rb1_init_shelf_mission_bt_node
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(rb1_init_shelf_mission_bt_node
  rclcpp
  geometry_msgs
  sensor_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  behaviortree_cpp_v3
  nav2_util
)

target_link_libraries(rb1_init_shelf_mission_bt_node
  rb1_shelf_scan_utils
)

target_compile_features(rb1_init_shelf_mission_bt_node PUBLIC cxx_std_17)
target_compile_definitions(rb1_init_shelf_mission_bt_node PRIVATE BT_PLUGIN_EXPORT)

# -------------------------------
# Plugin 3: ValidateShelfCandidate
# -------------------------------
add_library(rb1_validate_shelf_candidate_bt_node SHARED
  src/validate_shelf_candidate.cpp
)

target_include_directories(rb1_validate_shelf_candidate_bt_node
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(rb1_validate_shelf_candidate_bt_node
  rclcpp
  geometry_msgs
  sensor_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  behaviortree_cpp_v3
  nav2_util
)

target_link_libraries(rb1_validate_shelf_candidate_bt_node
  rb1_shelf_scan_utils
)

target_compile_features(rb1_validate_shelf_candidate_bt_node PUBLIC cxx_std_17)
target_compile_definitions(rb1_validate_shelf_candidate_bt_node PRIVATE BT_PLUGIN_EXPORT)

# -------------------------------
# Plugin 4: ComputePreDockPose
# -------------------------------
add_library(rb1_compute_pre_dock_pose_bt_node SHARED
  src/compute_pre_dock_pose.cpp
)

target_include_directories(rb1_compute_pre_dock_pose_bt_node
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(rb1_compute_pre_dock_pose_bt_node
  rclcpp
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  behaviortree_cpp_v3
  nav2_util
)

target_link_libraries(rb1_compute_pre_dock_pose_bt_node
  rb1_shelf_scan_utils
)

target_compile_features(rb1_compute_pre_dock_pose_bt_node PUBLIC cxx_std_17)
target_compile_definitions(rb1_compute_pre_dock_pose_bt_node PRIVATE BT_PLUGIN_EXPORT)

# -------------------------------
# Plugin 5: Remaining dummy mission nodes
# -------------------------------
add_library(rb1_dummy_mission_bt_nodes SHARED
  src/dummy_mission_nodes.cpp
)

target_include_directories(rb1_dummy_mission_bt_nodes
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(rb1_dummy_mission_bt_nodes
  rclcpp
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  behaviortree_cpp_v3
  nav2_util
)

target_compile_features(rb1_dummy_mission_bt_nodes PUBLIC cxx_std_17)
target_compile_definitions(rb1_dummy_mission_bt_nodes PRIVATE BT_PLUGIN_EXPORT)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS
    rb1_shelf_scan_utils
    rb1_patrol_until_candidate_bt_node
    rb1_init_shelf_mission_bt_node
    rb1_validate_shelf_candidate_bt_node
    rb1_compute_pre_dock_pose_bt_node
    rb1_dummy_mission_bt_nodes
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_include_directories(include)
ament_export_libraries(
  rb1_shelf_scan_utils
  rb1_patrol_until_candidate_bt_node
  rb1_init_shelf_mission_bt_node
  rb1_validate_shelf_candidate_bt_node
  rb1_compute_pre_dock_pose_bt_node
  rb1_dummy_mission_bt_nodes
)
ament_export_dependencies(
  rclcpp
  rclcpp_action
  geometry_msgs
  sensor_msgs
  nav2_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  behaviortree_cpp_v3
  nav2_util
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()