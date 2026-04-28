#!/usr/bin/env python3
"""
仿真夹爪 action server。

提供 /gripper_sim/follow_joint_trajectory (FollowJointTrajectory) 接口，
接受目标关节角度后等待 time_from_start 时长，然后返回 SUCCESSFUL。
接口名与真机 gripper_bridge 保持一致，切回真机时只需修改 gripper_action_name 参数。
"""
import time

import rclpy
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node


class GripperSimNode(Node):
    def __init__(self) -> None:
        super().__init__('gripper_sim_node')
        self.declare_parameter('action_name', '/gripper_sim/follow_joint_trajectory')
        action_name = self.get_parameter('action_name').value

        self._cb_group = ReentrantCallbackGroup()
        self._action_server = ActionServer(
            self,
            FollowJointTrajectory,
            action_name,
            execute_callback=self._execute_cb,
            goal_callback=self._goal_cb,
            cancel_callback=self._cancel_cb,
            callback_group=self._cb_group,
        )
        self.get_logger().info(f'仿真夹爪 action server 已启动: {action_name}')

    def _goal_cb(self, goal_request):
        return GoalResponse.ACCEPT

    def _cancel_cb(self, goal_handle):
        return CancelResponse.ACCEPT

    def _execute_cb(self, goal_handle):
        traj = goal_handle.request.trajectory
        duration_sec = 1.0
        if traj.points:
            last = traj.points[-1].time_from_start
            duration_sec = last.sec + last.nanosec / 1e9

        target_pos = traj.points[-1].positions[0] if traj.points else 0.0
        joint_names = traj.joint_names
        self.get_logger().info(
            f'仿真夹爪: joints={joint_names} target={target_pos:.3f} rad  '
            f'duration={duration_sec:.2f}s'
        )

        time.sleep(max(duration_sec, 0.1))

        result = FollowJointTrajectory.Result()
        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
        goal_handle.succeed()
        return result


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GripperSimNode()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
