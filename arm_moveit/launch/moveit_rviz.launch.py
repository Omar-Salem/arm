from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_moveit_rviz_launch
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    moveit_config = (MoveItConfigsBuilder("arm", package_name="arm_moveit")
                     .robot_description_semantic(os.path.join(
        get_package_share_directory('arm_moveit'), 'config', 'arm.srdf'
    ))
                     .to_moveit_configs())
    return generate_moveit_rviz_launch(moveit_config)
