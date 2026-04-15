"""
gripper_bridge_node.py

功能：
  将 MoveIt 发出的 FollowJointTrajectory Action（gripper_bridge_controller）
  转换为乐白夹爪的 SetGripper Service 调用。

转换逻辑：
  - gripper_r_joint1 范围：0（全闭）~ 1.0 rad（全开，硬件实测约 1.026 rad）
  - 乐白 set_claw amplitude 范围：0（全开）~ 100（全闭）
  - 换算：amplitude = joint_angle / 0.85 * 100
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from control_msgs.action import FollowJointTrajectory
from lebai_interfaces.srv import SetGripper


# 夹爪关节物理限位（rad），来自 URDF gripper_macro.xacro
GRIPPER_JOINT_MAX_RAD = 1.0
# 乐白 set_claw amplitude 满量程
LEBAI_AMPLITUDE_MAX = 100.0


def joint_angle_to_amplitude(angle_rad: float) -> float:
    """将关节角（rad）转换为乐白夹爪 amplitude（0~100）"""
    # 限幅，防止超出物理范围
    angle_rad = max(0.0, min(angle_rad, GRIPPER_JOINT_MAX_RAD))
    return angle_rad / GRIPPER_JOINT_MAX_RAD * LEBAI_AMPLITUDE_MAX


class GripperBridgeNode(Node):
    def __init__(self):
        super().__init__("gripper_bridge")

        # ── Service 客户端：调用乐白 IO 服务 ──────────────────────
        self.set_gripper_cli_ = self.create_client(
            SetGripper,
            "/io_service/set_gripper_position",
        )
        self.get_logger().info("等待 /io_service/set_gripper_position 服务...")
        self.set_gripper_cli_.wait_for_service(timeout_sec=10.0)
        self.get_logger().info("夹爪 IO 服务已就绪。")

        # ── Action Server：接收 MoveIt 的轨迹指令 ─────────────────
        # Action 名称必须与 moveit_controllers_real.yaml 中的 controller 名一致
        self._action_server = ActionServer(
            self,
            FollowJointTrajectory,
            "gripper_bridge_controller/follow_joint_trajectory",
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=ReentrantCallbackGroup(),
        )
        self.get_logger().info("夹爪桥接节点已启动，等待 MoveIt 指令...")

    def goal_callback(self, goal_request):
        """验证目标：必须包含 gripper_r_joint1"""
        if "gripper_r_joint1" not in goal_request.trajectory.joint_names:
            self.get_logger().error("目标中不含 gripper_r_joint1，拒绝。")
            return GoalResponse.REJECT
        if not goal_request.trajectory.points:
            self.get_logger().error("轨迹点为空，拒绝。")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info("收到取消请求。")
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        """执行：取轨迹最后一个点的目标角度，调用 SetGripper Service"""
        result = FollowJointTrajectory.Result()
        traj = goal_handle.request.trajectory

        # 找到 gripper_r_joint1 在 joint_names 中的索引
        try:
            joint_idx = traj.joint_names.index("gripper_r_joint1")
        except ValueError:
            self.get_logger().error("轨迹中找不到 gripper_r_joint1")
            result.error_code = FollowJointTrajectory.Result.INVALID_JOINTS
            goal_handle.abort()
            return result

        # 取轨迹最后一个点作为目标位置
        target_point = traj.points[-1]
        target_angle = target_point.positions[joint_idx]
        amplitude = joint_angle_to_amplitude(target_angle)

        self.get_logger().info(
            f"夹爪目标：{target_angle:.3f} rad → amplitude={amplitude:.1f}"
        )

        # 调用乐白 SetGripper Service
        req = SetGripper.Request()
        req.val = amplitude
        future = self.set_gripper_cli_.call_async(req)

        # 等待 Service 响应（同步等待，最多 5 秒）
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        if future.result() is None:
            self.get_logger().error("SetGripper Service 调用超时！")
            result.error_code = FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED
            goal_handle.abort()
            return result

        if not future.result().ret:
            self.get_logger().error("SetGripper Service 返回失败！")
            result.error_code = FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED
            goal_handle.abort()
            return result

        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
        goal_handle.succeed()
        self.get_logger().info("夹爪指令执行成功。")
        return result


def main(args=None):
    rclpy.init(args=args)
    node = GripperBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
