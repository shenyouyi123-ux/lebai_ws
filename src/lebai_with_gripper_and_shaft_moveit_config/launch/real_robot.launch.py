"""
real_robot.launch.py — 真实硬件启动文件

启动内容：
  1. lebai_driver robot_interface（机械臂驱动 + 夹爪 IO 服务）
  2. gripper_bridge_node（FollowJointTrajectory → SetGripper 桥接节点）
  3. robot_state_publisher（TF 树）
  4. static_transform_publisher（world → base_link 虚拟关节）
  5. move_group（MoveIt 规划核心，加载真实硬件控制器配置）
  6. rviz2（可视化）

用法：
  ros2 launch lebai_with_gripper_and_shaft_moveit_config real_robot.launch.py robot_ip:=10.20.17.1
"""

import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def load_yaml(package_name, file_path):
    pkg_path = get_package_share_directory(package_name)
    abs_path = os.path.join(pkg_path, file_path)
    with open(abs_path, "r") as f:
        return yaml.safe_load(f)


def generate_launch_description():
    # ── 参数声明 ──────────────────────────────────────────────────
    robot_ip_arg = DeclareLaunchArgument(
        "robot_ip",
        default_value="10.20.17.1",
        description="乐白机械臂控制器 IP 地址",
    )
    robot_ip = LaunchConfiguration("robot_ip")

    # ── MoveIt 基础配置（不含 trajectory_execution，手动构建）────────
    moveit_config = (
        MoveItConfigsBuilder(
            "lm3_with_shaft",
            package_name="lebai_with_gripper_and_shaft_moveit_config",
        )
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

    # ── 控制器配置：按官方乐白方式嵌套在 moveit_simple_controller_manager 下 ──
    # MoveItSimpleControllerManager 插件要求 controller_names 在此命名空间下
    controllers_yaml = load_yaml(
        "lebai_with_gripper_and_shaft_moveit_config",
        "config/moveit_controllers_real.yaml",
    )
    moveit_controllers = {
        "moveit_simple_controller_manager": controllers_yaml,
        "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
    }
    trajectory_execution = {
        "moveit_manage_controllers": True,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.01,
    }

    # ── 1. 乐白驱动（机械臂 + 夹爪 IO）──────────────────────────────
    lebai_driver_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("lebai_driver"),
                "launch",
                "robot_interface.launch.py",
            ])
        ]),
        launch_arguments={
            "robot_ip": robot_ip,
            "has_gripper": "true",
        }.items(),
    )

    # ── 2. 夹爪桥接节点 ───────────────────────────────────────────
    gripper_bridge_node = Node(
        package="lebai_gripper_bridge",
        executable="gripper_bridge",
        name="gripper_bridge",
        output="screen",
        parameters=[{"robot_ip": robot_ip}],
    )

    # ── 3. robot_state_publisher ──────────────────────────────────
    rsp_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        respawn=True,
        output="screen",
        parameters=[moveit_config.robot_description],
    )

    # ── 4. 静态 TF：world → base_link ────────────────────────────
    static_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["--frame-id", "world", "--child-frame-id", "base_link"],
    )

    # ── 5. move_group ─────────────────────────────────────────────
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            trajectory_execution,
            moveit_controllers,
            {
                "publish_robot_description_semantic": True,
                "allow_trajectory_execution": True,
                "publish_planning_scene": True,
                "publish_geometry_updates": True,
                "publish_state_updates": True,
                "publish_transforms_updates": True,
                "monitor_dynamics": False,
            },
        ],
    )

    # ── 6. RViz2 ─────────────────────────────────────────────────
    rviz_config = os.path.join(
        get_package_share_directory("lebai_with_gripper_and_shaft_moveit_config"),
        "config", "moveit.rviz",
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config],
        parameters=[
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
        ],
    )

    return LaunchDescription([
        robot_ip_arg,
        lebai_driver_node,
        gripper_bridge_node,
        rsp_node,
        static_tf_node,
        move_group_node,
        rviz_node,
    ])

