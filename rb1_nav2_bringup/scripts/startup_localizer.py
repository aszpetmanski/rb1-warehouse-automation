#!/usr/bin/env python3

import math
import sys
import time
import argparse
from typing import Optional

import rclpy
from rclpy.node import Node

from std_srvs.srv import Empty
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


def quat_to_yaw(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


class ManualStartupSpin(Node):
    def __init__(
        self,
        reinit_service: str,
        cmd_vel_topic: str,
        odom_topic: str,
        angular_speed: float,
        target_yaw: float,
        wait_after_reset: float,
    ) -> None:
        super().__init__('manual_startup_spin')

        self._reinit_client = self.create_client(Empty, reinit_service)
        self._cmd_pub = self.create_publisher(Twist, cmd_vel_topic, 10)
        self._odom_sub = self.create_subscription(
            Odometry,
            odom_topic,
            self.odom_callback,
            10,
        )

        self.angular_speed = angular_speed
        self.target_yaw = abs(target_yaw)
        self.wait_after_reset = wait_after_reset

        self.current_yaw: Optional[float] = None
        self.prev_yaw: Optional[float] = None
        self.accumulated_yaw: float = 0.0

    def odom_callback(self, msg: Odometry) -> None:
        q = msg.pose.pose.orientation
        yaw = quat_to_yaw(q.x, q.y, q.z, q.w)
        self.current_yaw = yaw

        if self.prev_yaw is None:
            self.prev_yaw = yaw
            return

        delta = normalize_angle(yaw - self.prev_yaw)
        self.accumulated_yaw += abs(delta)
        self.prev_yaw = yaw

    def wait_for_odom(self, timeout_sec: float = 10.0) -> bool:
        self.get_logger().info('Waiting for odom...')
        start = time.time()
        while rclpy.ok() and time.time() - start < timeout_sec:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.current_yaw is not None:
                self.get_logger().info(f'Got odom yaw: {self.current_yaw:.3f} rad')
                return True
        self.get_logger().error('Timed out waiting for odom.')
        return False

    def call_reinitialize(self, timeout_sec: float = 10.0) -> bool:
        self.get_logger().info(
            f'Waiting for relocalization service "{self._reinit_client.srv_name}"...'
        )
        if not self._reinit_client.wait_for_service(timeout_sec=timeout_sec):
            self.get_logger().error('Relocalization service not available.')
            return False

        self.get_logger().info('Calling global relocalization...')
        future = self._reinit_client.call_async(Empty.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)

        if not future.done():
            self.get_logger().error('Timed out waiting for relocalization response.')
            return False

        if future.exception() is not None:
            self.get_logger().error(f'Relocalization failed: {future.exception()}')
            return False

        self.get_logger().info('Global relocalization triggered.')
        return True

    def publish_stop(self) -> None:
        msg = Twist()
        self._cmd_pub.publish(msg)

    def rotate_until_done(self) -> bool:
        self.accumulated_yaw = 0.0
        self.prev_yaw = self.current_yaw

        msg = Twist()
        msg.angular.z = self.angular_speed

        direction = 'CCW' if self.angular_speed > 0.0 else 'CW'
        self.get_logger().info(
            f'Starting manual spin: speed={self.angular_speed:.3f} rad/s, '
            f'target={self.target_yaw:.3f} rad ({direction})'
        )

        rate_hz = 20.0
        period = 1.0 / rate_hz

        while rclpy.ok():
            self._cmd_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=period)

            self.get_logger().info(
                f'Accumulated yaw: {self.accumulated_yaw:.3f} / {self.target_yaw:.3f} rad',
                throttle_duration_sec=1.0,
            )

            if self.accumulated_yaw >= self.target_yaw:
                break

        self.publish_stop()
        time.sleep(0.2)
        self.publish_stop()

        self.get_logger().info(
            f'Spin complete. Total measured yaw: {self.accumulated_yaw:.3f} rad'
        )
        return True


def parse_args():
    parser = argparse.ArgumentParser(
        description='Call global relocalization, wait 2s, then rotate using cmd_vel and odom.'
    )
    parser.add_argument(
        '--reinit-service',
        default='/reinitialize_global_localization',
        help='Service name for AMCL global relocalization',
    )
    parser.add_argument(
        '--cmd-vel-topic',
        default='/cmd_vel',
        help='Twist topic used to rotate the robot',
    )
    parser.add_argument(
        '--odom-topic',
        default='/odom',
        help='Odometry topic used to measure rotated yaw',
    )
    parser.add_argument(
        '--angular-speed',
        type=float,
        default=-0.5,
        help='Angular velocity in rad/s. Negative=CW, positive=CCW',
    )
    parser.add_argument(
        '--target-yaw',
        type=float,
        default=2.0 * math.pi,
        help='Target accumulated yaw in radians, default = 360 deg',
    )
    parser.add_argument(
        '--wait-after-reset',
        type=float,
        default=2.0,
        help='Seconds to wait after relocalization before spinning',
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    rclpy.init()

    node = ManualStartupSpin(
        reinit_service=args.reinit_service,
        cmd_vel_topic=args.cmd_vel_topic,
        odom_topic=args.odom_topic,
        angular_speed=args.angular_speed,
        target_yaw=args.target_yaw,
        wait_after_reset=args.wait_after_reset,
    )

    try:
        if not node.wait_for_odom():
            return 1

        if not node.call_reinitialize():
            return 2

        node.get_logger().info(
            f'Waiting {args.wait_after_reset:.1f}s before starting spin...'
        )
        end_time = time.time() + args.wait_after_reset
        while rclpy.ok() and time.time() < end_time:
            rclpy.spin_once(node, timeout_sec=0.1)

        if not node.rotate_until_done():
            return 3

        node.get_logger().info('Done.')
        return 0

    except KeyboardInterrupt:
        node.get_logger().warning('Interrupted by user.')
        node.publish_stop()
        return 130

    finally:
        node.publish_stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())