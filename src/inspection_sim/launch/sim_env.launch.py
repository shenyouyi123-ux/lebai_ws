import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # 1. 启动空载的 Gazebo 世界
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
    )

    # 2. 找到你的船舶轴系 URDF 文件路径 (请确保这里的文件名和你的真实文件名一致！)
    shaft_urdf_path = os.path.join(
        get_package_share_directory('inspection_sim'),
        'urdf',
        'ship_shaft.urdf' # <--- 注意：如果你上一步保存的名字不一样，请修改这里
    )

    # 3. 把船舶轴系生成到 Gazebo 中
    spawn_shaft = Node(
        package='gazebo_ros', 
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'ship_shaft', 
            '-file', shaft_urdf_path,
            '-x', '1.0',  # 轴系放在机械臂正前方 1 米处
            '-y', '0.0',
            '-z', '1.0'
        ],
        output='screen'
    )

    # 4. 启动乐白官方的机械臂显示节点（带夹爪版本）
    lebai_display = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('lebai_lm3_support'), 'launch', 'display_lm3_with_gripper.launch.py')])
    )

    # 5. 从官方话题中拦截机械臂模型，并强行生成到 Gazebo 中
    spawn_lebai_arm = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'lebai_lm3',
            '-topic', 'robot_description', # 核心魔法：从这里抓取官方模型
            '-x', '0.0',
            '-y', '1.0',
            '-z', '0.0'
        ],
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        lebai_display,
        spawn_lebai_arm,
        spawn_shaft
    ])
