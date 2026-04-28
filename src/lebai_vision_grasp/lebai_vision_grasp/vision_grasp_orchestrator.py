#!/usr/bin/env python3
import math
import threading
import time
from collections import deque
from dataclasses import dataclass
from typing import Deque, Optional, Tuple

import rclpy
from builtin_interfaces.msg import Duration as RosDuration
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import PointStamped, Pose, Quaternion
from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from moveit_msgs.msg import (
    BoundingVolume,
    CollisionObject,
    Constraints,
    JointConstraint,
    MoveItErrorCodes,
    OrientationConstraint,
    PlanningScene,
    PositionConstraint,
    RobotState,
)
from moveit_msgs.srv import ApplyPlanningScene, GetCartesianPath, GetPositionIK
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener
from trajectory_msgs.msg import JointTrajectoryPoint
from visualization_msgs.msg import Marker


@dataclass
class TargetSample:
    stamp_sec: float
    point_m: Tuple[float, float, float]


class VisionGraspOrchestrator(Node):
    def __init__(self) -> None:
        super().__init__('vision_grasp_orchestrator')
        self._cb_group = ReentrantCallbackGroup()

        self._declare_parameters()
        self._load_parameters()

        self._targets: Deque[TargetSample] = deque(maxlen=max(self.stable_window_size * 3, 20))
        self._latest_joint_state: Optional[JointState] = None
        self._busy_lock = threading.Lock()
        self._busy = False
        self._table_applied = False
        self._object_added_once = False
        self._grasped_marker: Optional[Marker] = None  # 抓取后持续重发的方块 marker

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=False)

        self.create_subscription(
            PointStamped,
            self.target_topic,
            self._on_target,
            10,
            callback_group=self._cb_group,
        )
        self.create_subscription(
            JointState,
            self.joint_states_topic,
            self._on_joint_state,
            10,
            callback_group=self._cb_group,
        )

        self.marker_pub = self.create_publisher(Marker, '~/markers', 10)
        self.execute_srv = self.create_service(
            Trigger,
            '~/execute_once',
            self._on_execute_once,
            callback_group=self._cb_group,
        )
        self.open_srv = self.create_service(
            Trigger,
            '~/open_gripper',
            self._on_open_gripper,
            callback_group=self._cb_group,
        )
        self.close_srv = self.create_service(
            Trigger,
            '~/close_gripper',
            self._on_close_gripper,
            callback_group=self._cb_group,
        )

        self.move_group_action = ActionClient(
            self,
            MoveGroup,
            self.move_group_action_name,
            callback_group=self._cb_group,
        )
        self.execute_trajectory_action = ActionClient(
            self,
            ExecuteTrajectory,
            self.execute_trajectory_action_name,
            callback_group=self._cb_group,
        )
        self.gripper_action = ActionClient(
            self,
            FollowJointTrajectory,
            self.gripper_action_name,
            callback_group=self._cb_group,
        )
        self.compute_ik_cli = self.create_client(
            GetPositionIK,
            self.compute_ik_service_name,
            callback_group=self._cb_group,
        )
        self.compute_cartesian_cli = self.create_client(
            GetCartesianPath,
            self.compute_cartesian_path_service_name,
            callback_group=self._cb_group,
        )
        self.apply_scene_cli = self.create_client(
            ApplyPlanningScene,
            self.apply_planning_scene_service_name,
            callback_group=self._cb_group,
        )

        if self.scene_apply_on_startup and (self.table_enabled or self.target_object_collision_enabled):
            self._scene_timer = self.create_timer(0.5, self._sync_scene_objects_once, callback_group=self._cb_group)
        else:
            self._scene_timer = None

        # 定时重发抓取后的方块 marker，确保 RViz 持续跟随末端显示
        self.create_timer(0.1, self._republish_grasped_marker, callback_group=self._cb_group)

        self.get_logger().info(
            'lebai_vision_grasp 已启动，等待 /target_position 与 MoveIt/夹爪接口。'
        )
        self.get_logger().info(
            f'target_topic={self.target_topic} move_group={self.move_group_action_name} '
            f'compute_ik={self.compute_ik_service_name}'
        )

    def _declare_parameters(self) -> None:
        self.declare_parameter('target_topic', '/target_position')
        self.declare_parameter('joint_states_topic', '/joint_states')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('tool_link', 'tool0')
        self.declare_parameter('group_name', 'manipulator')
        self.declare_parameter(
            'arm_joint_names',
            ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6'],
        )

        self.declare_parameter('move_group_action_name', '/move_action')
        self.declare_parameter('execute_trajectory_action_name', '/execute_trajectory')
        self.declare_parameter('compute_ik_service_name', '/compute_ik')
        self.declare_parameter('compute_cartesian_path_service_name', '/compute_cartesian_path')
        self.declare_parameter('apply_planning_scene_service_name', '/apply_planning_scene')
        self.declare_parameter('gripper_action_name', '/gripper_bridge_controller/follow_joint_trajectory')
        self.declare_parameter('gripper_joint_name', 'gripper_r_joint1')

        self.declare_parameter('target_unit_scale', 0.001)
        self.declare_parameter('target_timeout_sec', 0.6)
        self.declare_parameter('stable_window_size', 5)
        self.declare_parameter('stable_position_tolerance_m', 0.01)
        self.declare_parameter('simulation_mode', False)

        self.declare_parameter('grasp_offset_x_m', 0.0)
        self.declare_parameter('grasp_offset_y_m', 0.0)
        self.declare_parameter('grasp_offset_z_m', 0.0)
        self.declare_parameter('approach_height_m', 0.10)
        self.declare_parameter('lift_height_m', 0.12)
        self.declare_parameter('min_grasp_z_m', 0.03)
        self.declare_parameter('min_approach_z_m', 0.10)
        self.declare_parameter('min_lift_z_m', 0.12)

        self.declare_parameter('allowed_planning_time', 3.0)
        self.declare_parameter('num_planning_attempts', 5)
        self.declare_parameter('max_velocity_scaling', 0.20)
        self.declare_parameter('max_acceleration_scaling', 0.20)
        self.declare_parameter('ik_timeout_sec', 0.30)
        self.declare_parameter('goal_joint_tolerance', 0.01)
        self.declare_parameter('planner_id', 'RRTConnectkConfigDefault')

        self.declare_parameter('orientation_constraint_enabled', True)
        self.declare_parameter('orientation_tol_x_rad', 0.20)
        self.declare_parameter('orientation_tol_y_rad', 0.20)
        self.declare_parameter('orientation_tol_z_rad', 0.35)

        self.declare_parameter('cartesian_max_step_m', 0.005)
        self.declare_parameter('cartesian_jump_threshold', 0.0)
        self.declare_parameter('cartesian_fraction_threshold', 0.95)

        self.declare_parameter('gripper_open_rad', 1.0)
        self.declare_parameter('gripper_closed_rad', 0.0)
        self.declare_parameter('gripper_motion_sec', 1.0)
        self.declare_parameter('wait_after_gripper_sec', 0.3)

        self.declare_parameter('action_timeout_sec', 20.0)
        self.declare_parameter('service_timeout_sec', 5.0)
        self.declare_parameter('tf_timeout_sec', 1.0)

        self.declare_parameter('scene_apply_on_startup', True)
        self.declare_parameter('table_enabled', False)
        self.declare_parameter('table_id', 'table')
        self.declare_parameter('table_size_x_m', 0.8)
        self.declare_parameter('table_size_y_m', 1.2)
        self.declare_parameter('table_size_z_m', 0.04)
        self.declare_parameter('table_pose_x_m', 0.45)
        self.declare_parameter('table_pose_y_m', 0.0)
        self.declare_parameter('table_pose_z_m', -0.02)

        self.declare_parameter('target_object_enabled', True)
        self.declare_parameter('target_object_collision_enabled', False)
        self.declare_parameter('target_object_id', 'target_cube')
        self.declare_parameter('target_object_size_x_m', 0.04)
        self.declare_parameter('target_object_size_y_m', 0.04)
        self.declare_parameter('target_object_size_z_m', 0.04)
        self.declare_parameter('target_object_pose_offset_x_m', 0.0)
        self.declare_parameter('target_object_pose_offset_y_m', 0.0)
        self.declare_parameter('target_object_pose_offset_z_m', 0.0)

    def _load_parameters(self) -> None:
        gp = self.get_parameter
        self.target_topic = gp('target_topic').value
        self.joint_states_topic = gp('joint_states_topic').value
        self.base_frame = gp('base_frame').value
        self.tool_link = gp('tool_link').value
        self.group_name = gp('group_name').value
        self.arm_joint_names = list(gp('arm_joint_names').value)

        self.move_group_action_name = gp('move_group_action_name').value
        self.execute_trajectory_action_name = gp('execute_trajectory_action_name').value
        self.compute_ik_service_name = gp('compute_ik_service_name').value
        self.compute_cartesian_path_service_name = gp('compute_cartesian_path_service_name').value
        self.apply_planning_scene_service_name = gp('apply_planning_scene_service_name').value
        self.gripper_action_name = gp('gripper_action_name').value
        self.gripper_joint_name = gp('gripper_joint_name').value

        self.target_unit_scale = float(gp('target_unit_scale').value)
        self.target_timeout_sec = float(gp('target_timeout_sec').value)
        self.stable_window_size = int(gp('stable_window_size').value)
        self.stable_position_tolerance_m = float(gp('stable_position_tolerance_m').value)
        self.simulation_mode = bool(gp('simulation_mode').value)

        self.grasp_offset_x_m = float(gp('grasp_offset_x_m').value)
        self.grasp_offset_y_m = float(gp('grasp_offset_y_m').value)
        self.grasp_offset_z_m = float(gp('grasp_offset_z_m').value)
        self.approach_height_m = float(gp('approach_height_m').value)
        self.lift_height_m = float(gp('lift_height_m').value)
        self.min_grasp_z_m = float(gp('min_grasp_z_m').value)
        self.min_approach_z_m = float(gp('min_approach_z_m').value)
        self.min_lift_z_m = float(gp('min_lift_z_m').value)

        self.allowed_planning_time = float(gp('allowed_planning_time').value)
        self.num_planning_attempts = int(gp('num_planning_attempts').value)
        self.max_velocity_scaling = float(gp('max_velocity_scaling').value)
        self.max_acceleration_scaling = float(gp('max_acceleration_scaling').value)
        self.ik_timeout_sec = float(gp('ik_timeout_sec').value)
        self.goal_joint_tolerance = float(gp('goal_joint_tolerance').value)
        self.planner_id = gp('planner_id').value

        self.orientation_constraint_enabled = bool(gp('orientation_constraint_enabled').value)
        self.orientation_tol_x_rad = float(gp('orientation_tol_x_rad').value)
        self.orientation_tol_y_rad = float(gp('orientation_tol_y_rad').value)
        self.orientation_tol_z_rad = float(gp('orientation_tol_z_rad').value)

        self.cartesian_max_step_m = float(gp('cartesian_max_step_m').value)
        self.cartesian_jump_threshold = float(gp('cartesian_jump_threshold').value)
        self.cartesian_fraction_threshold = float(gp('cartesian_fraction_threshold').value)

        self.gripper_open_rad = float(gp('gripper_open_rad').value)
        self.gripper_closed_rad = float(gp('gripper_closed_rad').value)
        self.gripper_motion_sec = float(gp('gripper_motion_sec').value)
        self.wait_after_gripper_sec = float(gp('wait_after_gripper_sec').value)

        self.action_timeout_sec = float(gp('action_timeout_sec').value)
        self.service_timeout_sec = float(gp('service_timeout_sec').value)
        self.tf_timeout_sec = float(gp('tf_timeout_sec').value)

        self.scene_apply_on_startup = bool(gp('scene_apply_on_startup').value)
        self.table_enabled = bool(gp('table_enabled').value)
        self.table_id = gp('table_id').value
        self.table_size_x_m = float(gp('table_size_x_m').value)
        self.table_size_y_m = float(gp('table_size_y_m').value)
        self.table_size_z_m = float(gp('table_size_z_m').value)
        self.table_pose_x_m = float(gp('table_pose_x_m').value)
        self.table_pose_y_m = float(gp('table_pose_y_m').value)
        self.table_pose_z_m = float(gp('table_pose_z_m').value)

        self.target_object_enabled = bool(gp('target_object_enabled').value)
        self.target_object_collision_enabled = bool(gp('target_object_collision_enabled').value)
        self.target_object_id = gp('target_object_id').value
        self.target_object_size_x_m = float(gp('target_object_size_x_m').value)
        self.target_object_size_y_m = float(gp('target_object_size_y_m').value)
        self.target_object_size_z_m = float(gp('target_object_size_z_m').value)
        self.target_object_pose_offset_x_m = float(gp('target_object_pose_offset_x_m').value)
        self.target_object_pose_offset_y_m = float(gp('target_object_pose_offset_y_m').value)
        self.target_object_pose_offset_z_m = float(gp('target_object_pose_offset_z_m').value)

    def _on_target(self, msg: PointStamped) -> None:
        if msg.header.frame_id and msg.header.frame_id != self.base_frame:
            self.get_logger().warn(f'收到非 {self.base_frame} 坐标目标: {msg.header.frame_id}')
            return

        point_m = (
            float(msg.point.x) * self.target_unit_scale,
            float(msg.point.y) * self.target_unit_scale,
            float(msg.point.z) * self.target_unit_scale,
        )
        stamp_sec = self.get_clock().now().nanoseconds / 1e9
        self._targets.append(TargetSample(stamp_sec=stamp_sec, point_m=point_m))
        self._publish_point_marker(0, point_m, 0.03, 0.15, 0.85, 0.15, 'target')
        self._publish_target_object_marker(point_m)

    def _on_joint_state(self, msg: JointState) -> None:
        self._latest_joint_state = msg

    def _sync_scene_objects_once(self) -> None:
        try:
            table_ready = self._ensure_table_collision() if self.table_enabled else True
            object_ready = self._sync_target_object_collision() if self.target_object_collision_enabled else True
            if table_ready and object_ready and not self.target_object_collision_enabled:
                if self._scene_timer is not None:
                    self._scene_timer.cancel()
                    self._scene_timer = None
        except Exception as exc:
            self.get_logger().warn(f'场景碰撞体同步失败，将继续重试: {exc}')

    def _on_execute_once(self, _, response: Trigger.Response) -> Trigger.Response:
        if not self._acquire_busy():
            response.success = False
            response.message = '抓取节点忙碌中，请等待当前动作结束。'
            return response

        try:
            message = self._execute_grasp_sequence()
            response.success = True
            response.message = message
        except Exception as exc:
            self.get_logger().error(f'抓取流程失败: {exc}')
            response.success = False
            response.message = str(exc)
        finally:
            self._release_busy()
        return response

    def _on_open_gripper(self, _, response: Trigger.Response) -> Trigger.Response:
        try:
            self._send_gripper(self.gripper_open_rad)
            response.success = True
            response.message = '夹爪已打开。'
        except Exception as exc:
            response.success = False
            response.message = str(exc)
        return response

    def _on_close_gripper(self, _, response: Trigger.Response) -> Trigger.Response:
        try:
            self._send_gripper(self.gripper_closed_rad)
            response.success = True
            response.message = '夹爪已闭合。'
        except Exception as exc:
            response.success = False
            response.message = str(exc)
        return response

    def _execute_grasp_sequence(self) -> str:
        self._wait_for_interfaces_ready()
        self._ensure_table_collision()
        self._sync_target_object_collision()

        target = self._get_stable_target()
        if target is None:
            raise RuntimeError('当前没有稳定的 /target_position 目标，请先让 cube 稳定识别。')

        # Use pointing-down orientation (tool Z = (0,0,-1))
        grasp_quat = self._quat_pointing_down()
        tz_x, tz_y, tz_z = self._tool_z_axis(grasp_quat)

        # tool0 position such that fingers reach the target
        grasp_pose = self._make_pose(
            x=target[0] + self.grasp_offset_x_m - self.grasp_offset_z_m * tz_x,
            y=target[1] + self.grasp_offset_y_m - self.grasp_offset_z_m * tz_y,
            z=max(target[2] - self.grasp_offset_z_m * tz_z, self.min_grasp_z_m),
            orientation=grasp_quat,
        )
        # approach directly above grasp point
        approach_pose = self._make_pose(
            x=grasp_pose.position.x,
            y=grasp_pose.position.y,
            z=max(grasp_pose.position.z + self.approach_height_m, self.min_approach_z_m),
            orientation=grasp_quat,
        )
        lift_pose = self._make_pose(
            x=grasp_pose.position.x,
            y=grasp_pose.position.y,
            z=max(grasp_pose.position.z + self.lift_height_m, self.min_lift_z_m),
            orientation=grasp_quat,
        )

        self._publish_point_marker(1, self._pose_to_tuple(grasp_pose), 0.028, 0.90, 0.25, 0.25, 'grasp')
        self._publish_point_marker(2, self._pose_to_tuple(approach_pose), 0.028, 0.20, 0.55, 0.95, 'approach')
        self._publish_point_marker(3, self._pose_to_tuple(lift_pose), 0.028, 0.95, 0.75, 0.20, 'lift')

        self.get_logger().info(
            '开始抓取: '
            f'target=({target[0]:+.3f}, {target[1]:+.3f}, {target[2]:+.3f}) m '
            f'grasp=({grasp_pose.position.x:+.3f}, {grasp_pose.position.y:+.3f}, {grasp_pose.position.z:+.3f}) m'
        )

        self._send_gripper(self.gripper_open_rad)
        self._plan_and_execute_approach(approach_pose, grasp_quat)
        self._execute_cartesian_segment(approach_pose, grasp_pose, grasp_quat, 'descend')
        self._send_gripper(self.gripper_closed_rad)
        self._attach_cube_to_gripper()

        actual_pose = self._lookup_current_pose()
        actual_lift = self._make_pose(
            x=actual_pose.position.x,
            y=actual_pose.position.y,
            z=max(actual_pose.position.z + self.lift_height_m, self.min_lift_z_m),
            orientation=grasp_quat,
        )
        self._execute_cartesian_segment(None, actual_lift, grasp_quat, 'lift')

        return (
            '抓取流程执行完成。'
            f' target=({target[0]:+.3f}, {target[1]:+.3f}, {target[2]:+.3f}) m'
            f' grasp=({grasp_pose.position.x:+.3f}, {grasp_pose.position.y:+.3f}, {grasp_pose.position.z:+.3f}) m'
        )

    def _wait_for_interfaces_ready(self) -> None:
        t = self.service_timeout_sec
        if not self.move_group_action.wait_for_server(timeout_sec=t):
            raise RuntimeError(f'未连接到 MoveGroup action: {self.move_group_action_name}')
        if not self.execute_trajectory_action.wait_for_server(timeout_sec=t):
            raise RuntimeError(f'未连接到 ExecuteTrajectory action: {self.execute_trajectory_action_name}')
        if not self.gripper_action.wait_for_server(timeout_sec=t):
            raise RuntimeError(f'未连接到夹爪 action: {self.gripper_action_name}')
        if not self.compute_ik_cli.wait_for_service(timeout_sec=t):
            raise RuntimeError(f'未连接到 IK 服务: {self.compute_ik_service_name}')
        if not self.compute_cartesian_cli.wait_for_service(timeout_sec=t):
            raise RuntimeError(f'未连接到笛卡尔路径服务: {self.compute_cartesian_path_service_name}')
        if not self.apply_scene_cli.wait_for_service(timeout_sec=t):
            raise RuntimeError(f'未连接到 PlanningScene 服务: {self.apply_planning_scene_service_name}')
        if self._latest_joint_state is None:
            raise RuntimeError('尚未收到 /joint_states，无法构造当前机器人状态。')

    def _latest_target(self) -> Optional[Tuple[float, float, float]]:
        if not self._targets:
            return None
        sample = self._targets[-1]
        now_sec = self.get_clock().now().nanoseconds / 1e9
        if now_sec - sample.stamp_sec > self.target_timeout_sec:
            return None
        return sample.point_m

    def _get_stable_target(self) -> Optional[Tuple[float, float, float]]:
        if len(self._targets) < self.stable_window_size:
            return None

        samples = list(self._targets)[-self.stable_window_size:]
        now_sec = self.get_clock().now().nanoseconds / 1e9
        if now_sec - samples[-1].stamp_sec > self.target_timeout_sec:
            return None

        mean_x = sum(s.point_m[0] for s in samples) / len(samples)
        mean_y = sum(s.point_m[1] for s in samples) / len(samples)
        mean_z = sum(s.point_m[2] for s in samples) / len(samples)
        mean = (mean_x, mean_y, mean_z)
        max_dist = max(
            math.dist(s.point_m, mean)
            for s in samples
        )
        if max_dist > self.stable_position_tolerance_m:
            self.get_logger().warn(
                f'视觉目标未稳定，最近窗口最大偏差 {max_dist:.4f} m，阈值 {self.stable_position_tolerance_m:.4f} m。'
            )
            return None
        return mean

    def _lookup_current_pose(self) -> Pose:
        if not self.tf_buffer.can_transform(
            self.base_frame,
            self.tool_link,
            rclpy.time.Time(),
            timeout=Duration(seconds=self.tf_timeout_sec),
        ):
            raise RuntimeError(f'无法查询 TF: {self.base_frame} -> {self.tool_link}')

        try:
            transform = self.tf_buffer.lookup_transform(
                self.base_frame,
                self.tool_link,
                rclpy.time.Time(),
                timeout=Duration(seconds=self.tf_timeout_sec),
            )
        except TransformException as exc:
            raise RuntimeError(f'查询当前末端姿态失败: {exc}') from exc

        pose = Pose()
        pose.position.x = transform.transform.translation.x
        pose.position.y = transform.transform.translation.y
        pose.position.z = transform.transform.translation.z
        pose.orientation = transform.transform.rotation
        return pose

    def _make_pose(self, x: float, y: float, z: float, orientation: Quaternion) -> Pose:
        pose = Pose()
        pose.position.x = float(x)
        pose.position.y = float(y)
        pose.position.z = float(z)
        pose.orientation = self._copy_quaternion(orientation)
        return pose

    def _current_robot_state(self) -> RobotState:
        if self._latest_joint_state is None:
            raise RuntimeError('当前没有 joint_states。')

        msg = self._latest_joint_state
        state = RobotState()
        state.joint_state.header = msg.header
        state.joint_state.name = list(msg.name)
        state.joint_state.position = list(msg.position)
        state.joint_state.velocity = list(msg.velocity)
        state.joint_state.effort = list(msg.effort)
        state.is_diff = False
        return state

    def _build_orientation_constraints(self, orientation: Quaternion) -> Constraints:
        constraints = Constraints()
        if not self.orientation_constraint_enabled:
            return constraints

        oc = OrientationConstraint()
        oc.header.frame_id = self.base_frame
        oc.link_name = self.tool_link
        oc.orientation = self._copy_quaternion(orientation)
        oc.absolute_x_axis_tolerance = self.orientation_tol_x_rad
        oc.absolute_y_axis_tolerance = self.orientation_tol_y_rad
        oc.absolute_z_axis_tolerance = self.orientation_tol_z_rad
        oc.parameterization = OrientationConstraint.ROTATION_VECTOR
        oc.weight = 1.0
        constraints.orientation_constraints = [oc]
        return constraints

    def _compute_ik(self, pose: Pose, orientation: Quaternion) -> RobotState:
        request = GetPositionIK.Request()
        ik = request.ik_request
        ik.group_name = self.group_name
        ik.robot_state = self._current_robot_state()
        ik.constraints = self._build_orientation_constraints(orientation)
        ik.avoid_collisions = True
        ik.ik_link_name = self.tool_link
        ik.pose_stamped.header.frame_id = self.base_frame
        ik.pose_stamped.pose = pose
        ik.timeout = self._to_duration_msg(self.ik_timeout_sec)

        future = self.compute_ik_cli.call_async(request)
        response = self._wait_future(future, self.service_timeout_sec, 'compute_ik')
        if response.error_code.val != MoveItErrorCodes.SUCCESS:
            raise RuntimeError(f'IK 求解失败，error_code={response.error_code.val}')
        return response.solution

    def _build_joint_goal_constraints(self, target_state: RobotState) -> Constraints:
        name_to_pos = dict(zip(target_state.joint_state.name, target_state.joint_state.position))
        constraints = Constraints()
        for joint_name in self.arm_joint_names:
            if joint_name not in name_to_pos:
                raise RuntimeError(f'IK 解中缺少关节 {joint_name}')
            jc = JointConstraint()
            jc.joint_name = joint_name
            jc.position = float(name_to_pos[joint_name])
            jc.tolerance_above = self.goal_joint_tolerance
            jc.tolerance_below = self.goal_joint_tolerance
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)
        return constraints

    def _plan_and_execute_approach(self, approach_pose: Pose, orientation: Quaternion) -> None:
        # Use pose goal constraints — MoveIt handles IK internally with multiple seeds,
        # which is more robust than a single explicit IK call from the home seed.
        pos_c = PositionConstraint()
        pos_c.header.frame_id = self.base_frame
        pos_c.link_name = self.tool_link
        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [0.005]
        bv = BoundingVolume()
        bv.primitives = [sphere]
        center = Pose()
        center.position.x = approach_pose.position.x
        center.position.y = approach_pose.position.y
        center.position.z = approach_pose.position.z
        center.orientation.w = 1.0
        bv.primitive_poses = [center]
        pos_c.constraint_region = bv
        pos_c.weight = 1.0

        ori_c = OrientationConstraint()
        ori_c.header.frame_id = self.base_frame
        ori_c.link_name = self.tool_link
        ori_c.orientation = self._copy_quaternion(orientation)
        ori_c.absolute_x_axis_tolerance = 0.05
        ori_c.absolute_y_axis_tolerance = 0.05
        ori_c.absolute_z_axis_tolerance = 0.05
        ori_c.parameterization = OrientationConstraint.ROTATION_VECTOR
        ori_c.weight = 1.0

        goal_constraints = Constraints()
        goal_constraints.position_constraints = [pos_c]
        goal_constraints.orientation_constraints = [ori_c]

        goal = MoveGroup.Goal()
        goal.request.group_name = self.group_name
        goal.request.start_state = self._current_robot_state()
        goal.request.goal_constraints = [goal_constraints]
        goal.request.allowed_planning_time = self.allowed_planning_time
        goal.request.num_planning_attempts = self.num_planning_attempts
        goal.request.max_velocity_scaling_factor = self.max_velocity_scaling
        goal.request.max_acceleration_scaling_factor = self.max_acceleration_scaling
        goal.planning_options.plan_only = False
        goal.planning_options.look_around = True
        goal.planning_options.replan = True
        goal.request.planner_id = self.planner_id
        goal.planning_options.planning_scene_diff.is_diff = True

        send_future = self.move_group_action.send_goal_async(goal)
        goal_handle = self._wait_future(send_future, self.action_timeout_sec, 'move_group send_goal')
        if goal_handle is None or not goal_handle.accepted:
            raise RuntimeError('MoveGroup approach 目标被拒绝。')

        result_future = goal_handle.get_result_async()
        wrapped = self._wait_future(
            result_future,
            self.action_timeout_sec + self.allowed_planning_time,
            'move_group result',
        )
        result = wrapped.result
        if result.error_code.val != MoveItErrorCodes.SUCCESS:
            raise RuntimeError(f'approach 规划或执行失败，error_code={result.error_code.val}')

    def _execute_cartesian_segment(
        self,
        start_pose: Optional[Pose],
        end_pose: Pose,
        orientation: Quaternion,
        segment_name: str,
    ) -> None:
        request = GetCartesianPath.Request()
        request.header.frame_id = self.base_frame
        request.start_state = self._current_robot_state()
        request.group_name = self.group_name
        request.link_name = self.tool_link
        # start_pose=None 时只传终点，MoveIt 用 start_state 的 FK 作为隐式起点
        request.waypoints = [start_pose, end_pose] if start_pose is not None else [end_pose]
        request.max_step = self.cartesian_max_step_m
        request.jump_threshold = self.cartesian_jump_threshold
        request.avoid_collisions = True
        request.path_constraints = self._build_orientation_constraints(orientation)

        future = self.compute_cartesian_cli.call_async(request)
        response = self._wait_future(future, self.service_timeout_sec, f'{segment_name} cartesian path')
        if response.error_code.val != MoveItErrorCodes.SUCCESS:
            raise RuntimeError(f'{segment_name} 笛卡尔路径失败，error_code={response.error_code.val}')
        if response.fraction < self.cartesian_fraction_threshold:
            raise RuntimeError(
                f'{segment_name} 笛卡尔路径不完整，fraction={response.fraction:.3f} '
                f'< {self.cartesian_fraction_threshold:.3f}'
            )
        if not response.solution.joint_trajectory.points:
            raise RuntimeError(f'{segment_name} 笛卡尔路径为空。')

        goal = ExecuteTrajectory.Goal()
        goal.trajectory = response.solution
        send_future = self.execute_trajectory_action.send_goal_async(goal)
        goal_handle = self._wait_future(send_future, self.action_timeout_sec, f'{segment_name} execute send_goal')
        if goal_handle is None or not goal_handle.accepted:
            raise RuntimeError(f'{segment_name} ExecuteTrajectory 目标被拒绝。')

        result_future = goal_handle.get_result_async()
        wrapped = self._wait_future(result_future, self.action_timeout_sec, f'{segment_name} execute result')
        result = wrapped.result
        if result.error_code.val != MoveItErrorCodes.SUCCESS:
            raise RuntimeError(f'{segment_name} 执行失败，error_code={result.error_code.val}')

    def _send_gripper(self, target_rad: float) -> None:
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = [self.gripper_joint_name]

        point = JointTrajectoryPoint()
        point.positions = [float(target_rad)]
        point.time_from_start = self._to_duration_msg(self.gripper_motion_sec)
        goal.trajectory.points = [point]

        send_future = self.gripper_action.send_goal_async(goal)
        goal_handle = self._wait_future(send_future, self.action_timeout_sec, 'gripper send_goal')
        if goal_handle is None or not goal_handle.accepted:
            raise RuntimeError('夹爪动作目标被拒绝。')

        result_future = goal_handle.get_result_async()
        wrapped = self._wait_future(
            result_future,
            self.action_timeout_sec + self.gripper_motion_sec,
            'gripper result',
        )
        result = wrapped.result
        if result.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
            raise RuntimeError(f'夹爪执行失败，error_code={result.error_code}')

        if self.wait_after_gripper_sec > 0.0:
            time.sleep(self.wait_after_gripper_sec)

    def _ensure_table_collision(self) -> bool:
        if not self.table_enabled:
            return True
        if self._table_applied:
            return True
        if not self.apply_scene_cli.service_is_ready():
            return False

        pose = Pose()
        pose.position.x = self.table_pose_x_m
        pose.position.y = self.table_pose_y_m
        pose.position.z = self.table_pose_z_m
        pose.orientation.w = 1.0

        collision = CollisionObject()
        collision.header.frame_id = self.base_frame
        collision.id = self.table_id
        collision.primitives = [self._make_box_primitive(
            self.table_size_x_m,
            self.table_size_y_m,
            self.table_size_z_m,
        )]
        collision.primitive_poses = [pose]
        collision.operation = CollisionObject.ADD

        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects = [collision]

        request = ApplyPlanningScene.Request()
        request.scene = scene
        future = self.apply_scene_cli.call_async(request)
        response = self._wait_future(future, self.service_timeout_sec, 'apply_planning_scene')
        if not response.success:
            raise RuntimeError('PlanningScene 未接受桌面碰撞体。')

        self._table_applied = True
        self.get_logger().info(
            '桌面碰撞体已加载到 PlanningScene: '
            f'id={self.table_id} size=({self.table_size_x_m:.3f}, {self.table_size_y_m:.3f}, {self.table_size_z_m:.3f})'
        )
        return True

    def _sync_target_object_collision(self) -> bool:
        if not self.target_object_collision_enabled:
            return True
        if not self.apply_scene_cli.service_is_ready():
            return False

        target = self._latest_target()
        if target is None:
            return False

        pose = Pose()
        pose.position.x = target[0] + self.target_object_pose_offset_x_m
        pose.position.y = target[1] + self.target_object_pose_offset_y_m
        pose.position.z = target[2] + self.target_object_pose_offset_z_m
        pose.orientation.w = 1.0

        collision = CollisionObject()
        collision.header.frame_id = self.base_frame
        collision.id = self.target_object_id
        collision.primitives = [self._make_box_primitive(
            self.target_object_size_x_m,
            self.target_object_size_y_m,
            self.target_object_size_z_m,
        )]
        collision.primitive_poses = [pose]
        collision.operation = CollisionObject.ADD

        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects = [collision]

        request = ApplyPlanningScene.Request()
        request.scene = scene
        future = self.apply_scene_cli.call_async(request)
        response = self._wait_future(future, self.service_timeout_sec, 'apply target object')
        if not response.success:
            raise RuntimeError('PlanningScene 未接受目标物体碰撞体。')

        if not self._object_added_once:
            self.get_logger().info(
                '目标物体碰撞体已加载: '
                f'id={self.target_object_id} size='
                f'({self.target_object_size_x_m:.3f}, {self.target_object_size_y_m:.3f}, {self.target_object_size_z_m:.3f})'
            )
            self._object_added_once = True
        return True

    def _make_box_primitive(self, x_size: float, y_size: float, z_size: float) -> SolidPrimitive:
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = [x_size, y_size, z_size]
        return primitive

    def _attach_cube_to_gripper(self) -> None:
        if not self.target_object_enabled:
            return
        rm = Marker()
        rm.header.frame_id = self.base_frame
        rm.ns = 'target_object'
        rm.id = 10
        rm.action = Marker.DELETE
        self.marker_pub.publish(rm)

        m = Marker()
        m.header.frame_id = self.tool_link
        m.header.stamp = rclpy.time.Time().to_msg()  # stamp=0 让 RViz 始终接受
        m.ns = 'grasped_cube'
        m.id = 20
        m.type = Marker.CUBE
        m.action = Marker.ADD
        m.pose.position.z = self.grasp_offset_z_m
        m.pose.orientation.w = 1.0
        m.scale.x = self.target_object_size_x_m
        m.scale.y = self.target_object_size_y_m
        m.scale.z = self.target_object_size_z_m
        m.color.a = 0.90
        m.color.r = 0.95
        m.color.g = 0.55
        m.color.b = 0.15
        self._grasped_marker = m  # 保存引用，由定时器持续重发
        self.marker_pub.publish(m)
        self.get_logger().info('方块已附着到夹爪 tool0 坐标系，跟随末端运动。')

    def _republish_grasped_marker(self) -> None:
        if self._grasped_marker is not None:
            self.marker_pub.publish(self._grasped_marker)

    def _publish_target_object_marker(self, target_xyz: Tuple[float, float, float]) -> None:
        if not self.target_object_enabled:
            return

        marker = Marker()
        marker.header.frame_id = self.base_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'target_object'
        marker.id = 10
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position.x = target_xyz[0] + self.target_object_pose_offset_x_m
        marker.pose.position.y = target_xyz[1] + self.target_object_pose_offset_y_m
        marker.pose.position.z = target_xyz[2] + self.target_object_pose_offset_z_m
        marker.pose.orientation.w = 1.0
        marker.scale.x = self.target_object_size_x_m
        marker.scale.y = self.target_object_size_y_m
        marker.scale.z = self.target_object_size_z_m
        marker.color.a = 0.80
        marker.color.r = 0.95
        marker.color.g = 0.55
        marker.color.b = 0.15
        self.marker_pub.publish(marker)

    def _publish_point_marker(
        self,
        marker_id: int,
        point_xyz: Tuple[float, float, float],
        scale: float,
        r: float,
        g: float,
        b: float,
        ns: str,
    ) -> None:
        marker = Marker()
        marker.header.frame_id = self.base_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = ns
        marker.id = marker_id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = point_xyz[0]
        marker.pose.position.y = point_xyz[1]
        marker.pose.position.z = point_xyz[2]
        marker.pose.orientation.w = 1.0
        marker.scale.x = scale
        marker.scale.y = scale
        marker.scale.z = scale
        marker.color.a = 0.85
        marker.color.r = r
        marker.color.g = g
        marker.color.b = b
        self.marker_pub.publish(marker)

    def _wait_future(self, future, timeout_sec: float, name: str):
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and time.monotonic() < deadline:
            if future.done():
                return future.result()
            time.sleep(0.05)
        raise TimeoutError(f'{name} 超时（{timeout_sec:.1f}s）')

    def _to_duration_msg(self, duration_sec: float) -> RosDuration:
        sec = int(duration_sec)
        nanosec = int((duration_sec - sec) * 1e9)
        return RosDuration(sec=sec, nanosec=nanosec)

    def _copy_quaternion(self, quat: Quaternion) -> Quaternion:
        copied = Quaternion()
        copied.x = quat.x
        copied.y = quat.y
        copied.z = quat.z
        copied.w = quat.w
        return copied

    def _pose_to_tuple(self, pose: Pose) -> Tuple[float, float, float]:
        return (pose.position.x, pose.position.y, pose.position.z)

    @staticmethod
    def _tool_z_axis(q: Quaternion) -> Tuple[float, float, float]:
        """返回 tool0 Z 轴在 base_link 坐标系中的单位方向向量."""
        x, y, z, w = float(q.x), float(q.y), float(q.z), float(q.w)
        return (
            2.0 * x * z + 2.0 * y * w,
            2.0 * y * z - 2.0 * x * w,
            1.0 - 2.0 * x * x - 2.0 * y * y,
        )

    @staticmethod
    def _quat_pointing_down() -> Quaternion:
        """返回 tool0 Z 轴指向 base_link -Z（垂直向下）的四元数（绕 Y 轴旋转 180°）."""
        q = Quaternion()
        q.x = 0.0
        q.y = 1.0
        q.z = 0.0
        q.w = 0.0
        return q

    def _acquire_busy(self) -> bool:
        with self._busy_lock:
            if self._busy:
                return False
            self._busy = True
            return True

    def _release_busy(self) -> None:
        with self._busy_lock:
            self._busy = False


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VisionGraspOrchestrator()
    executor = MultiThreadedExecutor(num_threads=4)
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
