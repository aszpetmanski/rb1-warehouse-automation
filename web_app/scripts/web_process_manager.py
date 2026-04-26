#!/usr/bin/env python3

import json
import os
import signal
import subprocess
import threading
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, DurabilityPolicy
from rclpy.time import Time

from std_srvs.srv import Trigger
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose

from tf2_ros import Buffer, TransformListener, TransformException


class WebProcessManager(Node):
    def __init__(self):
        super().__init__('web_process_manager')

        self.processes = {}

        self.log_dir = Path.home() / 'web_process_manager_logs'
        self.log_dir.mkdir(exist_ok=True)

        self.nav2_log_pub = self.create_publisher(String, '/web/log/nav2', 10)
        self.mission_log_pub = self.create_publisher(String, '/web/log/mission', 10)

        self.nav2_status_pub = self.create_publisher(String, '/web/status/nav2', 10)
        self.mission_status_pub = self.create_publisher(String, '/web/status/mission', 10)

        config_qos = QoSProfile(depth=1)
        config_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.web_config_pub = self.create_publisher(
            String,
            '/web/config',
            config_qos
        )

        self.robot_pose_map_pub = self.create_publisher(
            PoseStamped,
            '/web/robot_pose_map',
            10
        )

        self.nav_to_pose_sub = self.create_subscription(
            PoseStamped,
            '/web/nav_to_pose',
            self.handle_nav_to_pose,
            10
        )

        self.selected_dropoff_sub = self.create_subscription(
            String,
            '/web/selected_dropoff_waypoint',
            self.handle_selected_dropoff_waypoint,
            10
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.handle_odom_for_base_frame,
            10
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.nav_action_client = ActionClient(
            self,
            NavigateToPose,
            '/navigate_to_pose'
        )

        self.last_feedback_log_time = 0.0
        self.robot_base_frame = 'base_link'

        self.web_config = {
            'modes': {
                'simulation': {
                    'cmd_vel_topic': '/diffbot_base_controller/cmd_vel_unstamped',
                    'dropoff_waypoints': [
                        {
                            'label': 'SIM dropoff',
                            'value': '2.4,0.05,1.57'
                        }
                    ]
                },
                'real': {
                    'cmd_vel_topic': '/cmd_vel',
                    'dropoff_waypoints': [
                        {
                            'label': 'po prawej górna',
                            'value': '1.73,2.68,2.01'
                        },
                        {
                            'label': 'po prawej środkowa',
                            'value': '2.29,3.45,1.98'
                        },
                        {
                            'label': 'po prawej dolna',
                            'value': '3.132,3.449,2.035'
                        },
                        {
                            'label': 'tam gdzie była',
                            'value': '4.46,3.577,-1.11'
                        },
                        {
                            'label': 'koło ładowarki',
                            'value': '1.927,1.394,-1.116'
                        }
                    ]
                }
            }
        }

        self.selected_mode = 'simulation'
        self.selected_dropoff_waypoint = '2.4,0.05,1.57'

        self.commands = {
            'nav2_stack': {
                'service': '/web/start_nav2_stack',
                'command': 'ros2 launch rb1_nav2_bringup rb1_navigation.launch.py',
                'process_group': 'nav2',
                'log_group': 'nav2'
            },
            'nav2_stack_real': {
                'service': '/web/start_nav2_stack_real',
                'command': 'ros2 launch rb1_nav2_bringup rb1_navigation.launch.py sim:=false',
                'process_group': 'nav2',
                'log_group': 'nav2'
            },
            'self_localization': {
                'service': '/web/start_self_localization',
                'command': 'ros2 run rb1_nav2_bringup startup_localizer.py',
                'process_group': 'localization',
                'log_group': 'nav2'
            },
            'self_localization_unstamped': {
                'service': '/web/start_self_localization_unstamped',
                'command': (
                    'ros2 run rb1_nav2_bringup startup_localizer.py '
                    '--cmd-vel-topic /diffbot_base_controller/cmd_vel_unstamped '
                    '--target-yaw 3.14'
                ),
                'process_group': 'localization',
                'log_group': 'nav2'
            },
            'bt_mission': {
                'service': '/web/start_bt_mission',
                'command': (
                    'ros2 launch rb1_shelf_mission find_and_carry_shelf.launch.py '
                    'dropoff_waypoint:={dropoff_waypoint}'
                ),
                'process_group': 'mission',
                'log_group': 'mission'
            },
            'bt_mission_real': {
                'service': '/web/start_bt_mission_real',
                'command': (
                    'ros2 launch rb1_shelf_mission find_and_carry_shelf.launch.py '
                    'sim:=false '
                    'dropoff_waypoint:={dropoff_waypoint}'
                ),
                'process_group': 'mission',
                'log_group': 'mission'
            },
        }

        for key, config in self.commands.items():
            self.create_service(
                Trigger,
                config['service'],
                self.make_start_callback(key, config)
            )

            self.get_logger().info(f"Created service: {config['service']}")

        self.create_service(
            Trigger,
            '/web/stop_bt_mission',
            self.stop_mission_callback
        )

        self.create_service(
            Trigger,
            '/web/stop_nav2_stack',
            self.stop_nav2_callback
        )

        self.create_timer(1.0, self.publish_status)
        self.create_timer(2.0, self.publish_web_config)
        self.create_timer(0.2, self.publish_robot_pose_map)

        self.get_logger().info('Web process manager is ready.')

    def make_start_callback(self, key, config):
        def callback(request, response):
            return self.start_process(
                key=key,
                command=config['command'],
                process_group=config['process_group'],
                log_group=config['log_group'],
                response=response
            )

        return callback

    def publish_log(self, log_group, text):
        msg = String()
        msg.data = text

        if log_group == 'nav2':
            self.nav2_log_pub.publish(msg)
        elif log_group == 'mission':
            self.mission_log_pub.publish(msg)

    def publish_web_config(self):
        msg = String()
        msg.data = json.dumps(self.web_config, ensure_ascii=False)
        self.web_config_pub.publish(msg)

    def handle_selected_dropoff_waypoint(self, msg):
        try:
            data = json.loads(msg.data)

            mode = data.get('mode')
            value = data.get('value')

            if mode not in self.web_config['modes']:
                self.publish_log('mission', f'Rejected waypoint. Unknown mode: {mode}')
                return

            allowed_values = [
                waypoint['value']
                for waypoint in self.web_config['modes'][mode]['dropoff_waypoints']
            ]

            if value not in allowed_values:
                self.publish_log('mission', f'Rejected waypoint. Not in config: {value}')
                return

            self.selected_mode = mode
            self.selected_dropoff_waypoint = value

            self.publish_log(
                'mission',
                f'Selected dropoff waypoint for {mode}: {value}'
            )

        except Exception as error:
            self.publish_log('mission', f'Failed to parse selected waypoint: {error}')

    def handle_odom_for_base_frame(self, msg):
        if msg.child_frame_id:
            self.robot_base_frame = msg.child_frame_id

    def publish_robot_pose_map(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                'map',
                self.robot_base_frame,
                Time()
            )
        except TransformException:
            return

        pose_msg = PoseStamped()
        pose_msg.header = transform.header

        pose_msg.pose.position.x = transform.transform.translation.x
        pose_msg.pose.position.y = transform.transform.translation.y
        pose_msg.pose.position.z = transform.transform.translation.z

        pose_msg.pose.orientation = transform.transform.rotation

        self.robot_pose_map_pub.publish(pose_msg)

    def is_process_running(self, key):
        record = self.processes.get(key)

        if not record:
            return False

        return record['process'].poll() is None

    def is_group_running(self, process_group):
        for record in self.processes.values():
            if record['process_group'] == process_group:
                if record['process'].poll() is None:
                    return True

        return False

    def publish_status(self):
        nav2_msg = String()
        nav2_msg.data = 'running' if self.is_group_running('nav2') else 'stopped'
        self.nav2_status_pub.publish(nav2_msg)

        mission_msg = String()
        mission_msg.data = 'running' if self.is_group_running('mission') else 'stopped'
        self.mission_status_pub.publish(mission_msg)

    def start_process(self, key, command, process_group, log_group, response):
        if process_group in ['nav2', 'mission'] and self.is_group_running(process_group):
            response.success = False
            response.message = f'{process_group} is already running'
            return response

        if self.is_process_running(key):
            process = self.processes[key]['process']
            response.success = False
            response.message = f'{key} is already running with PID {process.pid}'
            return response

        if '{dropoff_waypoint}' in command:
            if not self.selected_dropoff_waypoint:
                response.success = False
                response.message = 'No dropoff waypoint selected'
                return response

            command = command.format(
                dropoff_waypoint=self.selected_dropoff_waypoint
            )

        log_file_path = self.log_dir / f'{key}.log'

        full_command = f"""
set -e

if [ -n "$ROS_DISTRO" ] && [ -f "/opt/ros/$ROS_DISTRO/setup.bash" ]; then
    source /opt/ros/$ROS_DISTRO/setup.bash
fi

if [ -f "$HOME/ros2_ws/install/setup.bash" ]; then
    source "$HOME/ros2_ws/install/setup.bash"
fi

{command}
"""

        try:
            process = subprocess.Popen(
                full_command,
                shell=True,
                executable='/bin/bash',
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1,
                preexec_fn=os.setsid
            )

            self.processes[key] = {
                'process': process,
                'process_group': process_group,
                'log_group': log_group,
                'log_file_path': log_file_path
            }

            threading.Thread(
                target=self.stream_process_logs,
                args=(key,),
                daemon=True
            ).start()

            threading.Thread(
                target=self.monitor_process,
                args=(key,),
                daemon=True
            ).start()

            response.success = True
            response.message = f'Started {key} with PID {process.pid}. Log: {log_file_path}'

            self.publish_log(log_group, response.message)
            self.get_logger().info(response.message)

        except Exception as error:
            response.success = False
            response.message = f'Failed to start {key}: {error}'
            self.get_logger().error(response.message)

        return response

    def stream_process_logs(self, key):
        record = self.processes.get(key)

        if not record:
            return

        process = record['process']
        log_group = record['log_group']
        log_file_path = record['log_file_path']

        with open(log_file_path, 'a', buffering=1) as log_file:
            for line in process.stdout:
                clean_line = line.rstrip()

                if not clean_line:
                    continue

                log_file.write(clean_line + '\n')
                self.publish_log(log_group, f'[{key}] {clean_line}')

    def monitor_process(self, key):
        record = self.processes.get(key)

        if not record:
            return

        process = record['process']
        return_code = process.wait()

        log_group = record['log_group']

        message = f'{key} exited with code {return_code}'
        self.publish_log(log_group, message)
        self.get_logger().info(message)

        self.publish_status()

    def stop_group(self, process_group):
        stopped = []

        for key, record in self.processes.items():
            if record['process_group'] != process_group:
                continue

            process = record['process']

            if process.poll() is None:
                os.killpg(os.getpgid(process.pid), signal.SIGTERM)
                stopped.append(key)

        return stopped

    def stop_mission_callback(self, request, response):
        stopped = self.stop_group('mission')

        if stopped:
            response.success = True
            response.message = f'Stopped mission processes: {", ".join(stopped)}'
            self.publish_log('mission', response.message)
        else:
            response.success = False
            response.message = 'No mission process was running'

        self.publish_status()
        return response

    def stop_nav2_callback(self, request, response):
        stopped = self.stop_group('nav2')

        if stopped:
            response.success = True
            response.message = f'Stopped Nav2 processes: {", ".join(stopped)}'
            self.publish_log('nav2', response.message)
        else:
            response.success = False
            response.message = 'No Nav2 process was running'

        self.publish_status()
        return response

    def handle_nav_to_pose(self, pose_msg):
        if not self.is_group_running('nav2'):
            self.publish_log('nav2', 'Rejected Nav To Pose. Nav2 stack is not running.')
            return

        self.publish_log(
            'nav2',
            f'Received Nav To Pose: x={pose_msg.pose.position.x:.2f}, y={pose_msg.pose.position.y:.2f}'
        )

        if not self.nav_action_client.wait_for_server(timeout_sec=2.0):
            self.publish_log('nav2', 'NavigateToPose action server is not available.')
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose_msg
        goal_msg.behavior_tree = ''

        send_goal_future = self.nav_action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.nav_feedback_callback
        )

        send_goal_future.add_done_callback(self.nav_goal_response_callback)

    def nav_goal_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.publish_log('nav2', 'Nav To Pose goal was rejected.')
            return

        self.publish_log('nav2', 'Nav To Pose goal accepted.')

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.nav_result_callback)

    def nav_feedback_callback(self, feedback_msg):
        now = self.get_clock().now().nanoseconds / 1e9

        if now - self.last_feedback_log_time < 2.0:
            return

        self.last_feedback_log_time = now

        feedback = feedback_msg.feedback

        try:
            distance = feedback.distance_remaining
            self.publish_log('nav2', f'Nav feedback. Distance remaining: {distance:.2f} m')
        except Exception:
            self.publish_log('nav2', 'Nav feedback received.')

    def nav_result_callback(self, future):
        result = future.result()
        self.publish_log('nav2', f'Nav To Pose finished with status: {result.status}')

    def destroy_node(self):
        for key, record in self.processes.items():
            process = record['process']

            if process and process.poll() is None:
                self.get_logger().info(f'Stopping {key} with PID {process.pid}')
                os.killpg(os.getpgid(process.pid), signal.SIGTERM)

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = WebProcessManager()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()