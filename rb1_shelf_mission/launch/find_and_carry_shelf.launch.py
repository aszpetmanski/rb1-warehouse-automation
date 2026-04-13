import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory, get_package_prefix


def lib_path(pkg_name: str, lib_name: str) -> str:
    return os.path.join(get_package_prefix(pkg_name), 'lib', lib_name)


def launch_setup(context, *args, **kwargs):
    sim_value = LaunchConfiguration('sim').perform(context).lower()
    sim = sim_value in ['true', '1', 'yes']

    use_sim_time = sim

    mission_pkg_share = get_package_share_directory('rb1_shelf_mission')

    tree_xml_file = os.path.join(
        mission_pkg_share,
        'config',
        'find_carry_shelf.xml'
    )

    plugin_libs = [
        # Custom BT pluginy
        lib_path('rb1_nav2_bt_nodes', 'librb1_patrol_until_candidate_bt_node.so'),
        lib_path('rb1_nav2_bt_nodes', 'librb1_init_shelf_mission_bt_node.so'),
        lib_path('rb1_nav2_bt_nodes', 'librb1_validate_shelf_candidate_bt_node.so'),
        lib_path('rb1_nav2_bt_nodes', 'librb1_compute_pre_dock_pose_bt_node.so'),
        lib_path('rb1_nav2_bt_nodes', 'librb1_dock_to_shelf_bt_node.so'),
        lib_path('rb1_nav2_bt_nodes', 'librb1_switch_nav2_mode_bt_node.so'),
        lib_path('rb1_nav2_bt_nodes', 'librb1_dummy_mission_bt_nodes.so'),

        # Nav2 BT pluginy użyte w XML
        lib_path('nav2_behavior_tree', 'libnav2_compute_path_to_pose_action_bt_node.so'),
        lib_path('nav2_behavior_tree', 'libnav2_follow_path_action_bt_node.so'),
        lib_path('nav2_behavior_tree', 'libnav2_clear_costmap_service_bt_node.so'),
        lib_path('nav2_behavior_tree', 'libnav2_spin_action_bt_node.so'),
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
                'plugin_libs': plugin_libs,
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
        OpaqueFunction(function=launch_setup)
    ])