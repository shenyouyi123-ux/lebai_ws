from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("lm3_with_shaft", package_name="lebai_with_gripper_and_shaft_moveit_config").to_moveit_configs()
    return generate_demo_launch(moveit_config)
