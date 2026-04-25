#!/usr/bin/env python3

import os
import signal
import subprocess
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger


class WebProcessManager(Node):
    def __init__(self):
        super().__init__('web_process_manager')

        self.processes = {}

        self.log_dir = Path.home() / 'web_process_manager_logs'
        self.log_dir.mkdir(exist_ok=True)

        self.commands = {
            'nav2_stack': (
                '/web/start_nav2_stack',
                'ros2 launch rb1_nav2_bringup rb1_navigation.launch.py'
            ),
            'nav2_stack_real': (
                '/web/start_nav2_stack_real',
                'ros2 launch rb1_nav2_bringup rb1_navigation.launch.py sim:=false'
            ),
            'self_localization': (
                '/web/start_self_localization',
                'ros2 run rb1_nav2_bringup startup_localizer.py'
            ),
            'self_localization_unstamped': (
                '/web/start_self_localization_unstamped',
                'ros2 run rb1_nav2_bringup startup_localizer.py --cmd-vel-topic /diffbot_base_controller/cmd_vel_unstamped'
            ),
            'bt_mission': (
                '/web/start_bt_mission',
                'ros2 launch rb1_shelf_mission find_and_carry_shelf.launch.py'
            ),
            'bt_mission_real': (
                '/web/start_bt_mission_real',
                'ros2 launch rb1_shelf_mission find_and_carry_shelf.launch.py sim:=false'
            ),
        }

        for key, service_data in self.commands.items():
            service_name, command = service_data
            self.create_service(
                Trigger,
                service_name,
                lambda request, response, k=key, c=command: self.start_process(k, c, response)
            )
            self.get_logger().info(f'Created service: {service_name}')

    def start_process(self, key, command, response):
        existing_process = self.processes.get(key)

        if existing_process and existing_process.poll() is None:
            response.success = False
            response.message = f'{key} is already running with PID {existing_process.pid}'
            return response

        log_file_path = self.log_dir / f'{key}.log'
        log_file = open(log_file_path, 'a')

        full_command = f"""
source /opt/ros/$ROS_DISTRO/setup.bash
if [ -f ~/ros2_ws/install/setup.bash ]; then
    source ~/ros2_ws/install/setup.bash
fi
{command}
"""

        try:
            process = subprocess.Popen(
                full_command,
                shell=True,
                executable='/bin/bash',
                stdout=log_file,
                stderr=subprocess.STDOUT,
                preexec_fn=os.setsid
            )

            self.processes[key] = process

            response.success = True
            response.message = f'Started {key} with PID {process.pid}. Log: {log_file_path}'
            self.get_logger().info(response.message)

        except Exception as error:
            response.success = False
            response.message = f'Failed to start {key}: {error}'
            self.get_logger().error(response.message)

        return response

    def destroy_node(self):
        for key, process in self.processes.items():
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