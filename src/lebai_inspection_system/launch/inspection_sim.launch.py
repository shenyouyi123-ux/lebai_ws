import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    points_file_arg = DeclareLaunchArgument(
        'points_file',
        default_value='',
        description='Inspection points yaml path. Empty means use package default.',
    )
    auto_start_arg = DeclareLaunchArgument(
        'auto_start',
        default_value='true',
        description='Whether task_orchestrator starts automatic inspection immediately.',
    )

    base_moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('lebai_with_gripper_and_shaft_moveit_config'),
                'launch',
                'demo.launch.py',
            )
        )
    )

    params_yaml = os.path.join(
        get_package_share_directory('lebai_inspection_system'),
        'config',
        'task_params.yaml',
    )

    task_orchestrator = Node(
        package='lebai_inspection_system',
        executable='task_orchestrator',
        name='task_orchestrator',
        output='screen',
        parameters=[
            params_yaml,
            {
                'arm_controller_name': 'manipulator_controller',
                'gripper_controller_name': 'gripper_controller',
                'points_file': LaunchConfiguration('points_file'),
                'auto_start': LaunchConfiguration('auto_start'),
            },
        ],
    )

    fake_perception = Node(
        package='lebai_inspection_system',
        executable='fake_perception',
        name='fake_perception',
        output='screen',
        parameters=[params_yaml],
    )

    inspection_logger = Node(
        package='lebai_inspection_system',
        executable='inspection_logger',
        name='inspection_logger',
        output='screen',
        parameters=[
            params_yaml,
            {
                'output_file': '/tmp/lebai_inspection_sim.jsonl',
            },
        ],
    )

    return LaunchDescription([
        points_file_arg,
        auto_start_arg,
        base_moveit_launch,
        fake_perception,
        task_orchestrator,
        inspection_logger,
    ])
