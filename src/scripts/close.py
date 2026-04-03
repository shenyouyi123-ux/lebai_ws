#!/usr/bin/env python3
# close.py
import rclpy
from rclpy.node import Node
import lebai_sdk
import nest_asyncio
import asyncio

class SimpleClawControl(Node):
    def __init__(self):
        super().__init__('simple_claw_control')
        # 初始化 SDK 和 nest_asyncio
        lebai_sdk.init()
        nest_asyncio.apply()
        self.robot_ip = "10.20.17.1"  # 根据实际 IP 修改

    def run(self):
        # 创建新的事件循环并设置为当前循环
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        try:
            # 运行异步主函数，直到完成
            loop.run_until_complete(self.async_main())
        finally:
            loop.close()

    async def async_main(self):
        self.get_logger().info(f"正在连接机器人 {self.robot_ip} ...")
        # 等待异步连接完成，获取连接对象
        self.lebai = await lebai_sdk.connect(self.robot_ip, False)
        self.get_logger().info("机器人连接成功")

        self.get_logger().info("设置夹爪: 力度=100, 开度=100")
        self.lebai.set_claw(100, 100)

        # 等待夹爪动作完成（异步等待，不阻塞事件循环）
        await asyncio.sleep(2)
        self.get_logger().info("夹爪设置完成，节点即将退出")

def main(args=None):
    rclpy.init(args=args)
    node = SimpleClawControl()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()