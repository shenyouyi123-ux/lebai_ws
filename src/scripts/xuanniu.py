#!/usr/bin/env python3
import asyncio
import rclpy
from rclpy.node import Node
import lebai_sdk
import nest_asyncio
import math

nest_asyncio.apply()

class PickAndPlaceNode(Node):
    def __init__(self):
        super().__init__('pick_and_place_node')
        self.get_logger().info("节点已创建")
        lebai_sdk.init()

    async def wait_for_target_pose(self, target_pose, pos_threshold=0.005, rot_threshold=0.03, check_interval=0.1, timeout=30.0):
        """
        等待机器人到达目标位姿
        target_pose: 目标笛卡尔位姿字典 {'x':..., 'y':..., 'z':..., 'rx':..., 'ry':..., 'rz':...}
        pos_threshold: 位置误差阈值（米），默认5mm
        rot_threshold: 姿态误差阈值（弧度），默认约1.7度
        """
        start_time = asyncio.get_event_loop().time()
        
        while True:
            kin_data = await self.robot.get_kin_data()
            current = kin_data['actual_tcp_pose']
            
            # 计算位置误差
            dx = current['x'] - target_pose['x']
            dy = current['y'] - target_pose['y']
            dz = current['z'] - target_pose['z']
            pos_error = math.sqrt(dx*dx + dy*dy + dz*dz)
            
            # 计算姿态误差（简单欧拉角差值，注意角度范围处理）
            drx = abs(current['rx'] - target_pose['rx'])
            dry = abs(current['ry'] - target_pose['ry'])
            drz = abs(current['rz'] - target_pose['rz'])
            # 处理角度环绕（例如 -3.14 和 3.14 实际很近，但这里简化，因为实际变化不大）
            rot_error = max(drx, dry, drz)
            
            if pos_error < pos_threshold and rot_error < rot_threshold:
                self.get_logger().debug(f"已到达目标，位置误差={pos_error:.4f}m，姿态误差={rot_error:.4f}rad")
                break
            
            if asyncio.get_event_loop().time() - start_time > timeout:
                self.get_logger().warning(f"等待超时，当前误差 位置={pos_error:.4f} 姿态={rot_error:.4f}")
                break
            
            await asyncio.sleep(check_interval)
    
    async def set_claw_and_wait(self, force, amplitude, wait_sec=1.5):
        """设置夹爪并等待物理动作完成"""
        self.get_logger().info(f"设置夹爪: 力度={force}, 幅度={amplitude}")
        self.robot.set_claw(force, amplitude)
        await asyncio.sleep(wait_sec)
        self.get_logger().info("夹爪动作完成")

    async def async_execute(self):
        robot_ip = "10.20.17.1"
        self.get_logger().info(f"正在连接机器人 {robot_ip} ...")
        self.robot = await lebai_sdk.connect(robot_ip, False)
        self.get_logger().info("机器人连接成功")

        # 运动参数（速度适中）
        a, v, t, r = 1.5, 1.5, 0.0, 0.5

        # 1. 回到初始位置
        init_pose = {
            'x': 0.08248248769464794, 'y': -0.12110306026873664, 'z': 0.8464760864717737,
            'rx': 1.6131962426804916, 'ry': -0.03089712178015767, 'rz': 1.5640181348370632
        }
        self.get_logger().info("1. 移动到初始位置...")
        await self.robot.movej(init_pose, a, v, t, r)
        await self.wait_for_target_pose(init_pose)
        self.get_logger().info("已到达初始位置")

        # 2. 打开夹爪
        self.get_logger().info("2. 打开夹爪...")
        await self.set_claw_and_wait(force=100, amplitude=100, wait_sec=1.5)

        # 3. 移动到抓取位置（位置1）
        pose1 = {
            'x': -0.2918293635101673, 'y': 0.364749731735286, 'z': 0.2217362405786639,
            'rx': 3.1091533741497246, 'ry': -0.018513856234401944, 'rz': -2.3993969792216183
        }
        self.get_logger().info("3. 移动到抓取位置...")
        await self.robot.movej(pose1, a, v, t, r)
        await self.wait_for_target_pose(pose1)   # 关键：等待真正到达位置1
        self.get_logger().info("已到达抓取位置")

        # 4. 闭合夹爪抓取物体（力控，力度80避免夹坏）
        self.get_logger().info("4. 闭合夹爪（力控抓取）...")
        await self.set_claw_and_wait(force=80, amplitude=0, wait_sec=2.0)

        # 5. 移动到放置位置（位置2）
        pose2 = {
            'x': 0.34754010866919455, 'y': 0.32464738319660813, 'z': 0.20915625577485553,
            'rx': 3.127418252973623, 'ry': -0.012638534171530214, 'rz': 2.3782255437711903
        }
        self.get_logger().info("5. 移动到放置位置...")
        await self.robot.movej(pose2, a, v, t, r)
        await self.wait_for_target_pose(pose2)
        self.get_logger().info("已到达放置位置")

        # 6. 打开夹爪释放物体
        self.get_logger().info("6. 打开夹爪释放物体...")
        await self.set_claw_and_wait(force=100, amplitude=100, wait_sec=1.5)

        self.get_logger().info("全部动作完成！")

def main(args=None):
    rclpy.init(args=args)
    node = PickAndPlaceNode()
    try:
        asyncio.run(node.async_execute())
    except KeyboardInterrupt:
        node.get_logger().info("用户中断")
    except Exception as e:
        node.get_logger().error(f"发生异常: {e}")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()