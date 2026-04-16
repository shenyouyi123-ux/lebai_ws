import math

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


class FakePerception(Node):
    def __init__(self) -> None:
        super().__init__('fake_perception')
        self.declare_parameter('frame_id', 'base_link')
        self.declare_parameter('period_sec', 1.0)

        self.frame_id = self.get_parameter('frame_id').value
        period_sec = float(self.get_parameter('period_sec').value)
        self.pub = self.create_publisher(PoseStamped, '/inspection/target_pose', 10)
        self.tick = 0

        self.create_timer(period_sec, self._publish_pose)
        self.get_logger().info('Fake perception started.')

    def _publish_pose(self) -> None:
        t = self.tick * 0.1
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.pose.position.x = 0.45 + 0.02 * math.sin(t)
        msg.pose.position.y = 0.10 + 0.02 * math.cos(t)
        msg.pose.position.z = 0.25
        msg.pose.orientation.w = 1.0
        self.pub.publish(msg)
        self.tick += 1


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FakePerception()
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
