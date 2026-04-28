#!/usr/bin/env python3
import math

import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


class FakeTargetPosition(Node):
    def __init__(self) -> None:
        super().__init__('fake_target_position')
        self.declare_parameter('target_topic', '/target_position')
        self.declare_parameter('frame_id', 'base_link')
        self.declare_parameter('publish_period_sec', 0.2)
        self.declare_parameter('motion_mode', 'fixed')
        self.declare_parameter('fixed_x_m', 0.42)
        self.declare_parameter('fixed_y_m', 0.00)
        self.declare_parameter('fixed_z_m', 0.06)
        self.declare_parameter('sweep_center_x_m', 0.42)
        self.declare_parameter('sweep_center_y_m', 0.00)
        self.declare_parameter('sweep_center_z_m', 0.06)
        self.declare_parameter('sweep_radius_x_m', 0.03)
        self.declare_parameter('sweep_radius_y_m', 0.02)
        self.declare_parameter('sweep_omega_rad_s', 0.35)

        self.target_topic = self.get_parameter('target_topic').value
        self.frame_id = self.get_parameter('frame_id').value
        self.publish_period_sec = float(self.get_parameter('publish_period_sec').value)
        self.motion_mode = self.get_parameter('motion_mode').value
        self.fixed_x_m = float(self.get_parameter('fixed_x_m').value)
        self.fixed_y_m = float(self.get_parameter('fixed_y_m').value)
        self.fixed_z_m = float(self.get_parameter('fixed_z_m').value)
        self.sweep_center_x_m = float(self.get_parameter('sweep_center_x_m').value)
        self.sweep_center_y_m = float(self.get_parameter('sweep_center_y_m').value)
        self.sweep_center_z_m = float(self.get_parameter('sweep_center_z_m').value)
        self.sweep_radius_x_m = float(self.get_parameter('sweep_radius_x_m').value)
        self.sweep_radius_y_m = float(self.get_parameter('sweep_radius_y_m').value)
        self.sweep_omega_rad_s = float(self.get_parameter('sweep_omega_rad_s').value)

        self.pub = self.create_publisher(PointStamped, self.target_topic, 10)
        self.start_time_sec = self.get_clock().now().nanoseconds / 1e9
        self.create_timer(self.publish_period_sec, self._publish_target)

        self.get_logger().info(
            f'fake_target_position started: topic={self.target_topic} mode={self.motion_mode} frame={self.frame_id}'
        )

    def _publish_target(self) -> None:
        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        if self.motion_mode == 'sweep':
            now_sec = self.get_clock().now().nanoseconds / 1e9
            phase = (now_sec - self.start_time_sec) * self.sweep_omega_rad_s
            msg.point.x = self.sweep_center_x_m + self.sweep_radius_x_m * math.sin(phase)
            msg.point.y = self.sweep_center_y_m + self.sweep_radius_y_m * math.cos(phase)
            msg.point.z = self.sweep_center_z_m
        else:
            msg.point.x = self.fixed_x_m
            msg.point.y = self.fixed_y_m
            msg.point.z = self.fixed_z_m

        self.pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FakeTargetPosition()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
