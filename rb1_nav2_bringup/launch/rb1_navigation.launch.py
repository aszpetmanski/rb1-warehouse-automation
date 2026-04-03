import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def launch_setup(context, *args, **kwargs):
    sim_value = LaunchConfiguration('sim').perform(context).lower()
    sim = sim_value in ['true', '1', 'yes']

    
    pkg_share = get_package_share_directory('rb1_nav2_bringup')

    suffix = '' if sim else '_real_robot'
    use_sim_time = sim

    map_file = os.path.join(pkg_share, 'maps', f'warehouse_map{suffix}.yaml')
    amcl_config = os.path.join(pkg_share, 'config', f'amcl_config{suffix}.yaml')
    controller_config = os.path.join(pkg_share, 'config', f'controller{suffix}.yaml')
    planner_config = os.path.join(pkg_share, 'config', f'planner{suffix}.yaml')
    behavior_config = os.path.join(pkg_share, 'config', f'recoveries{suffix}.yaml')
    bt_config = os.path.join(pkg_share, 'config', f'bt_navigator{suffix}.yaml')
    rviz_config = os.path.join(pkg_share, 'rviz', f'localization{suffix}.rviz')

    return [
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[
                {'use_sim_time': use_sim_time},
                {'yaml_filename': map_file},
            ],
        ),

        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[
                amcl_config,
                {'use_sim_time': use_sim_time},
            ],
        ),

        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[
                planner_config,
                {'use_sim_time': use_sim_time},
            ],
        ),

        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[
                controller_config,
                {'use_sim_time': use_sim_time},
            ],
            remappings=[
                ('/cmd_vel', '/diffbot_base_controller/cmd_vel_unstamped')
            ],
        ),

        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            parameters=[
                behavior_config,
                {'use_sim_time': use_sim_time},
            ],
            remappings=[
                ('/cmd_vel', '/diffbot_base_controller/cmd_vel_unstamped')
            ],
        ),

        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[
                bt_config,
                {'use_sim_time': use_sim_time},
            ],
        ),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': True,
                'node_names': [
                    'map_server',
                    'amcl',
                    'planner_server',
                    'controller_server',
                    'behavior_server',
                    'bt_navigator'
                ]
            }],
        ),

        # Optional RViz
         Node(
             package='rviz2',
             executable='rviz2',
             name='rviz2',
             output='screen',
             parameters=[{'use_sim_time': use_sim_time}],
             arguments=['-d', rviz_config],
         ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'sim',
            default_value='true',
            description='Use simulation config and sim time'
        ),
        OpaqueFunction(function=launch_setup)
    ])