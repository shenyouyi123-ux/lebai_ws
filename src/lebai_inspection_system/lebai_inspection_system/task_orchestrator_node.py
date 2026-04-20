import json
import math
import os
from typing import Any, Dict, List, Optional

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectoryPoint


class TaskOrchestrator(Node):
    def __init__(self) -> None:
        super().__init__('task_orchestrator')

        self.declare_parameter('arm_controller_name', 'manipulator_controller')
        self.declare_parameter('gripper_controller_name', 'gripper_bridge_controller')
        self.declare_parameter('arm_action_name', '')
        self.declare_parameter('gripper_action_name', '')
        self.declare_parameter('arm_joint_names', ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6'])
        self.declare_parameter('points_file', '')
        self.declare_parameter('point_duration_sec', 3.0)
        self.declare_parameter('pause_after_point_sec', 1.0)
        self.declare_parameter('auto_start', True)
        # rad/s — used to compute safe duration from current position to first point
        self.declare_parameter('max_joint_speed', 0.5)

        self.arm_controller_name = self.get_parameter('arm_controller_name').value
        self.gripper_controller_name = self.get_parameter('gripper_controller_name').value
        _arm_action_name = self.get_parameter('arm_action_name').value
        _gripper_action_name = self.get_parameter('gripper_action_name').value
        self.arm_joint_names = list(self.get_parameter('arm_joint_names').value)
        self.point_duration_sec = float(self.get_parameter('point_duration_sec').value)
        self.pause_after_point_sec = float(self.get_parameter('pause_after_point_sec').value)
        self.auto_start = bool(self.get_parameter('auto_start').value)
        self.max_joint_speed = float(self.get_parameter('max_joint_speed').value)

        self._current_joints: Optional[List[float]] = None
        self._joint_state_order: List[str] = []

        points_file = self.get_parameter('points_file').value
        if not points_file:
            points_file = os.path.join(
                get_package_share_directory('lebai_inspection_system'),
                'config',
                'inspection_points.yaml',
            )

        self.points = self._load_points(points_file)
        self.current_index = 0

        self.event_pub = self.create_publisher(String, '/inspection/events', 10)
        self.create_subscription(JointState, '/joint_states', self._on_joint_state, 10)

        arm_action = _arm_action_name or f'/{self.arm_controller_name}/follow_joint_trajectory'
        gripper_action = _gripper_action_name or f'/{self.gripper_controller_name}/follow_joint_trajectory'

        self.arm_action = ActionClient(self, FollowJointTrajectory, arm_action)
        self.gripper_action = ActionClient(self, FollowJointTrajectory, gripper_action)

        self._startup_timer = self.create_timer(0.5, self._try_start)
        self._started = False
        self._resume_timer = None

        self.get_logger().info(
            f'Loaded {len(self.points)} inspection points from {points_file}'
        )

    def _load_points(self, points_file: str) -> List[Dict[str, Any]]:
        if not os.path.exists(points_file):
            raise FileNotFoundError(f'Points file not found: {points_file}')

        with open(points_file, 'r', encoding='utf-8') as f:
            data = yaml.safe_load(f) or {}

        points = data.get('inspection_points', [])
        if not points:
            raise RuntimeError('inspection_points is empty.')

        for idx, p in enumerate(points):
            joints = p.get('joints', [])
            if len(joints) != len(self.arm_joint_names):
                raise RuntimeError(
                    f'Point #{idx} ({p.get("name", "unknown")}) joints size is {len(joints)}, '
                    f'expected {len(self.arm_joint_names)}.'
                )
        return points

    def _on_joint_state(self, msg: JointState) -> None:
        name_to_pos = dict(zip(msg.name, msg.position))
        positions = [name_to_pos.get(n) for n in self.arm_joint_names]
        if any(p is None for p in positions):
            return
        self._current_joints = [float(p) for p in positions]

    def _publish_event(self, event: str, payload: Dict[str, Any]) -> None:
        msg = String()
        msg.data = json.dumps({'event': event, **payload}, ensure_ascii=False)
        self.event_pub.publish(msg)

    def _try_start(self) -> None:
        if self._started:
            return
        if not self.auto_start:
            return

        if not self.arm_action.server_is_ready() or not self.gripper_action.server_is_ready():
            self.get_logger().info('Waiting action servers for arm/gripper...')
            return

        if self._current_joints is None:
            self.get_logger().info('Waiting for /joint_states...')
            return

        self.get_logger().info(f'Current joints: {[f"{v:.3f}" for v in self._current_joints]}')
        self._started = True
        self._startup_timer.cancel()
        self._publish_event('task_started', {'total_points': len(self.points)})
        self._run_next_point()

    def _run_next_point(self) -> None:
        if self.current_index >= len(self.points):
            self.get_logger().info('Inspection task completed.')
            self._publish_event('task_completed', {'total_points': len(self.points)})
            return

        point = self.points[self.current_index]
        name = point.get('name', f'p{self.current_index + 1}')
        joints = [float(v) for v in point.get('joints', [])]
        gripper = float(point.get('gripper_rad', 0.0))

        self.get_logger().info(f'Moving to point {name} ...')
        self._publish_event('move_start', {'point': name, 'index': self.current_index})

        arm_goal = FollowJointTrajectory.Goal()
        arm_goal.trajectory.joint_names = self.arm_joint_names

        # Use current joint positions as start so duration is computed from actual distance.
        start = self._current_joints if self._current_joints is not None else joints
        max_disp = max(abs(a - b) for a, b in zip(start, joints))
        duration = max(self.point_duration_sec, max_disp / self.max_joint_speed)

        arm_goal.trajectory.points = [
            self._make_point(start, 0.0),
            self._make_point(joints, duration),
        ]
        self.get_logger().info(f'Trajectory duration: {duration:.1f}s (max_disp={max_disp:.3f} rad)')

        send_future = self.arm_action.send_goal_async(arm_goal)
        send_future.add_done_callback(
            lambda f: self._on_arm_goal_response(f, name, gripper)
        )

    def _on_arm_goal_response(self, future, point_name: str, gripper_rad: float) -> None:
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error(f'Arm goal rejected at point {point_name}.')
            self._publish_event('move_rejected', {'point': point_name})
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda f: self._on_arm_result(f, point_name, gripper_rad)
        )

    def _on_arm_result(self, future, point_name: str, gripper_rad: float) -> None:
        result = future.result().result
        if result.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
            self.get_logger().error(
                f'Arm move failed at {point_name}, code={result.error_code}'
            )
            self._publish_event('move_failed', {'point': point_name, 'code': int(result.error_code)})
            return

        self.get_logger().info(f'Arm reached {point_name}, triggering gripper.')
        self._publish_event('move_done', {'point': point_name})

        gripper_goal = FollowJointTrajectory.Goal()
        gripper_goal.trajectory.joint_names = ['gripper_r_joint1']
        gripper_goal.trajectory.points = [
            self._make_point([gripper_rad], 1.0)
        ]

        send_future = self.gripper_action.send_goal_async(gripper_goal)
        send_future.add_done_callback(
            lambda f: self._on_gripper_goal_response(f, point_name)
        )

    def _on_gripper_goal_response(self, future, point_name: str) -> None:
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error(f'Gripper goal rejected at point {point_name}.')
            self._publish_event('gripper_rejected', {'point': point_name})
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda f: self._on_gripper_result(f, point_name)
        )

    def _on_gripper_result(self, future, point_name: str) -> None:
        result = future.result().result
        if result.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
            self.get_logger().error(
                f'Gripper action failed at {point_name}, code={result.error_code}'
            )
            self._publish_event('gripper_failed', {'point': point_name, 'code': int(result.error_code)})
            return

        self._publish_event('point_done', {'point': point_name, 'index': self.current_index})
        self.get_logger().info(f'Point {point_name} done.')

        # Update current joints to the target we just reached.
        point = self.points[self.current_index]
        self._current_joints = [float(v) for v in point.get('joints', [])]

        self.current_index += 1
        self._resume_timer = self.create_timer(self.pause_after_point_sec, self._resume_once)

    def _resume_once(self) -> None:
        if self._resume_timer is not None:
            self._resume_timer.cancel()
            self._resume_timer = None
        self._run_next_point()

    @staticmethod
    def _make_point(positions: List[float], duration_sec: float) -> JointTrajectoryPoint:
        p = JointTrajectoryPoint()
        p.positions = positions
        sec = int(duration_sec)
        nsec = int((duration_sec - sec) * 1e9)
        p.time_from_start = Duration(sec=sec, nanosec=nsec)
        return p


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TaskOrchestrator()
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
