import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory, get_package_prefix


def lib_path(pkg_name: str, lib_name: str) -> str:
    return os.path.join(get_package_prefix(pkg_name), 'lib', lib_name)


def launch_setup(context, *args, **kwargs):
    sim_value = LaunchConfiguration('sim').perform(context).lower()
    sim = sim_value in ['true', '1', 'yes']

    use_sim_time = sim

    dropoff_waypoint_value = LaunchConfiguration('dropoff_waypoint').perform(context)

    if not dropoff_waypoint_value:
        if sim:
            dropoff_waypoint_value = '2.4,0.05,1.57'
        else:
            dropoff_waypoint_value = '1.73,2.68,2.01'

    mission_pkg_share = get_package_share_directory('rb1_shelf_mission')

    tree_base_name = 'find_carry_shelf'
    tree_suffix = '' if sim else '_real_robot'
    tree_xml_file = os.path.join(
        mission_pkg_share,
        'config',
        f'{tree_base_name}{tree_suffix}.xml'
    )

    plugin_libs = [
        # Custom BT pluginy
        lib_path('rb1_nav2_bt_nodes', 'librb1_patrol_until_candidate_bt_node.so'),
        lib_path('rb1_nav2_bt_nodes', 'librb1_init_shelf_mission_bt_node.so'),
        lib_path('rb1_nav2_bt_nodes', 'librb1_validate_shelf_candidate_bt_node.so'),
        lib_path('rb1_nav2_bt_nodes', 'librb1_compute_pre_dock_pose_bt_node.so'),
        lib_path('rb1_nav2_bt_nodes', 'librb1_dock_to_shelf_bt_node.so'),
        lib_path('rb1_nav2_bt_nodes', 'librb1_switch_nav2_mode_bt_node.so'),
        lib_path('rb1_nav2_bt_nodes', 'librb1_custom_back_up_bt_node.so'),
        lib_path('rb1_nav2_bt_nodes', 'librb1_place_shelf_bt_node.so'),
        lib_path('rb1_nav2_bt_nodes', 'librb1_custom_drive_on_heading_bt_node.so'),
        lib_path('rb1_nav2_bt_nodes', 'librb1_refine_shelf_geometry_bt_node.so'),
        lib_path('rb1_nav2_bt_nodes', 'librb1_adjust_to_pose_local_bt_node.so'),
        lib_path('rb1_nav2_bt_nodes', 'librb1_dummy_mission_bt_nodes.so'),

        # Nav2 BT pluginy użyte w XML
        lib_path('nav2_behavior_tree', 'libnav2_compute_path_to_pose_action_bt_node.so'),
        lib_path('nav2_behavior_tree', 'libnav2_follow_path_action_bt_node.so'),
        lib_path('nav2_behavior_tree', 'libnav2_clear_costmap_service_bt_node.so'),
        lib_path('nav2_behavior_tree', 'libnav2_spin_action_bt_node.so'),
        lib_path('nav2_behavior_tree', 'libnav2_back_up_action_bt_node.so'),
        lib_path('nav2_behavior_tree', 'libnav2_wait_action_bt_node.so'),
        lib_path('nav2_behavior_tree', 'libnav2_rate_controller_bt_node.so'),
        lib_path('nav2_behavior_tree', 'libnav2_recovery_node_bt_node.so'),
        lib_path('nav2_behavior_tree', 'libnav2_pipeline_sequence_bt_node.so'),
    ]

    return [
        Node(
            package='rb1_shelf_mission',
            executable='shelf_mission_executor',
            name='shelf_mission_executor',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'tree_xml_file': tree_xml_file,
                'tick_period_ms': 100,
                'server_timeout_ms': 2000,
                'wait_for_service_timeout_ms': 2000,
                'autostart': False,
                'exit_on_finish': True,
                'plugin_libs': plugin_libs,

                # Passed from web_process_manager:
                #   dropoff_waypoint:=x,y,yaw
                #
                # Example:
                #   ros2 launch rb1_shelf_mission find_and_carry_shelf.launch.py \
                #     dropoff_waypoint:=2.4,0.05,1.57
                'dropoff_waypoint': ParameterValue(
                    dropoff_waypoint_value,
                    value_type=str
                ),
            }],
        )
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'sim',
            default_value='true',
            description='Use simulation time'
        ),

        DeclareLaunchArgument(
            'dropoff_waypoint',
            default_value='',
            description='Shelf dropoff waypoint as x,y,yaw'
        ),

        OpaqueFunction(function=launch_setup)
    ])