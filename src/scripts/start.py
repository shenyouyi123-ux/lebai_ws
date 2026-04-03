#!/usr/bin/env python3
import asyncio
import rclpy
from rclpy.node import Node
import lebai_sdk
import nest_asyncio

nest_asyncio.apply()

async def main_async():
    lebai_sdk.init()
    robot_ip = "10.20.17.1" 
    lebai = await lebai_sdk.connect(robot_ip, False) 
    await lebai.start_sys()
    return lebai

class LebaiRobotNode(Node):
    def __init__(self):
        super().__init__('lebai_robot_node')
        self.lebai = asyncio.run(main_async())
        self.get_logger().info(f'已连接到机器人 10.20.17.1')

def main(args=None):
    rclpy.init(args=args)
    node = LebaiRobotNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()