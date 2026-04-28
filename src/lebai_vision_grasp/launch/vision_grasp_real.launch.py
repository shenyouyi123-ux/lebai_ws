import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('lebai_vision_grasp')
    default_params = os.path.join(pkg_share, 'config', 'vision_grasp_params.yaml')
    real_robot_launch = os.path.join(
        get_package_share_directory('lebai_with_gripper_and_shaft_moveit_config'),
        'launch',
        'real_robot.launch.py',
    )

    robot_ip_arg = DeclareLaunchArgument(
        'robot_ip',
        default_value='10.20.17.1',
        description='Lebai robot controller IP.',
    )
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params,
        description='Parameter file for lebai_vision_grasp.',
    )
    start_robot_stack_arg = DeclareLaunchArgument(
        'start_robot_stack',
        default_value='false',
        description='Whether to include the existing real_robot.launch.py stack.',
    )

    include_real_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(real_robot_launch),
        condition=IfCondition(LaunchConfiguration('start_robot_stack')),
        launch_arguments={
            'robot_ip': LaunchConfiguration('robot_ip'),
        }.items(),
    )

    orchestrator_node = Node(
        package='lebai_vision_grasp',
        executable='vision_grasp_orchestrator',
        name='vision_grasp_orchestrator',
        output='screen',
        parameters=[LaunchConfiguration('params_file')],
    )

    return LaunchDescription([
        robot_ip_arg,
        params_file_arg,
        start_robot_stack_arg,
        include_real_robot,
        orchestrator_node,
    ])
