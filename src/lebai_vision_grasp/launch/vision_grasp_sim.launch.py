import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('lebai_vision_grasp')
    default_params = os.path.join(pkg_share, 'config', 'vision_grasp_sim_params.yaml')
    moveit_demo_launch = os.path.join(
        get_package_share_directory('lebai_with_gripper_and_shaft_moveit_config'),
        'launch',
        'demo.launch.py',
    )

    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params,
        description='Parameter file for simulation grasp workflow.',
    )

    base_moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(moveit_demo_launch),
    )

    fake_target_node = Node(
        package='lebai_vision_grasp',
        executable='fake_target_position',
        name='fake_target_position',
        output='screen',
        parameters=[LaunchConfiguration('params_file')],
    )

    gripper_sim_node = Node(
        package='lebai_vision_grasp',
        executable='gripper_sim_node',
        name='gripper_sim_node',
        output='screen',
    )

    orchestrator_node = Node(
        package='lebai_vision_grasp',
        executable='vision_grasp_orchestrator',
        name='vision_grasp_orchestrator',
        output='screen',
        parameters=[LaunchConfiguration('params_file')],
    )

    return LaunchDescription([
        params_file_arg,
        base_moveit_launch,
        gripper_sim_node,
        fake_target_node,
        orchestrator_node,
    ])
