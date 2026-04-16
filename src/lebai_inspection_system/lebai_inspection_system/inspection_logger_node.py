import json
import os

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String


class InspectionLogger(Node):
    def __init__(self) -> None:
        super().__init__('inspection_logger')
        self.declare_parameter('output_file', '/tmp/lebai_inspection_result.jsonl')
        self.output_file = self.get_parameter('output_file').value
        os.makedirs(os.path.dirname(self.output_file), exist_ok=True)

        self.create_subscription(String, '/inspection/events', self._on_event, 50)
        self.get_logger().info(f'Writing inspection events to {self.output_file}')

    def _on_event(self, msg: String) -> None:
        payload = {
            'stamp_ns': self.get_clock().now().nanoseconds,
            'raw': msg.data,
        }
        with open(self.output_file, 'a', encoding='utf-8') as f:
            f.write(json.dumps(payload, ensure_ascii=False) + '\n')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = InspectionLogger()
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
