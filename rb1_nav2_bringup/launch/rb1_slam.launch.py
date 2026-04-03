from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('rb1_nav2_bringup')
    slam_toolbox_share = get_package_share_directory('slam_toolbox')

    sim = LaunchConfiguration('sim')

    # Files for simulation
    sim_slam_params = os.path.join(pkg_share, 'config', 'slam_toolbox.yaml')
    sim_rviz_config = os.path.join(pkg_share, 'rviz', 'mapping.rviz')

    # Files for real robot
    real_slam_params = os.path.join(pkg_share, 'config', 'slam_toolbox_real_robot.yaml')
    real_rviz_config = os.path.join(pkg_share, 'rviz', 'mapping_real_robot.rviz')  # change if needed

    slam_toolbox_launch = os.path.join(
        slam_toolbox_share,
        'launch',
        'online_async_launch.py'   # or online_sync_launch.py
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'sim',
            default_value='true',
            description='Use simulation settings (true/false)'
        ),

        # SIM branch
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(slam_toolbox_launch),
            launch_arguments={
                'slam_params_file': sim_slam_params,
                'use_sim_time': 'true',
            }.items(),
            condition=IfCondition(sim)
        ),

        # REAL ROBOT branch
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(slam_toolbox_launch),
            launch_arguments={
                'slam_params_file': real_slam_params,
                'use_sim_time': 'false',
            }.items(),
            condition=UnlessCondition(sim)
        ),

        # Optional RViz for SIM
        # Node(
        #     package='rviz2',
        #     executable='rviz2',
        #     name='rviz2',
        #     output='screen',
        #     arguments=['-d', sim_rviz_config],
        #     parameters=[{'use_sim_time': True}],
        #     condition=IfCondition(sim)
        # ),

        # Optional RViz for REAL ROBOT
        # Node(
        #     package='rviz2',
        #     executable='rviz2',
        #     name='rviz2',
        #     output='screen',
        #     arguments=['-d', real_rviz_config],
        #     parameters=[{'use_sim_time': False}],
        #     condition=UnlessCondition(sim)
        # ),
    ])