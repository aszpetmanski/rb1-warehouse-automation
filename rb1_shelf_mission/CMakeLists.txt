cmake_minimum_required(VERSION 3.8)
project(rb1_shelf_mission)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rb1_nav2_bringup REQUIRED)
find_package(rb1_nav2_bt_nodes REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(shelf_mission_executor
  src/shelf_mission_executor.cpp
)
ament_target_dependencies(shelf_mission_executor
  rclcpp
  std_srvs
  behaviortree_cpp_v3
  tf2_ros
)

install(TARGETS
  shelf_mission_executor
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(
  DIRECTORY
    launch
    #rviz
    config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()