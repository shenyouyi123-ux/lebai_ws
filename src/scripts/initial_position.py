#!/usr/bin/env python3
import asyncio
import rclpy
from rclpy.node import Node
import lebai_sdk
import nest_asyncio

nest_asyncio.apply()  # 允许在已有事件循环中嵌套

class LebaiMoveJNode(Node):
    def __init__(self):
        super().__init__('lebai_movej_node')
        self.get_logger().info("节点已创建")

async def robot_task(node):
    lebai_sdk.init()
    robot_ip = "10.20.17.1"
    lebai = await lebai_sdk.connect(robot_ip, False)
    
    cartesian_pose = {
        'x': 0.08248248769464794,
        'y': -0.12110306026873664,
        'z': 0.8464760864717737,
        'rx': 1.6131962426804916, 
        'ry': -0.03089712178015767, 
        'rz': 1.5640181348370632
    }
    a, v, t, r = 5, 5, 0, 0.5
    
    await lebai.movej(cartesian_pose, a, v, t, r)
    node.get_logger().info("MoveJ运动指令已发送")
    await asyncio.sleep(0.1)  # 确保指令发出

async def main(args=None):
    rclpy.init(args=args)
    node = LebaiMoveJNode()
    await robot_task(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    asyncio.run(main())