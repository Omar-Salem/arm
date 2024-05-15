from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_spawn_controllers_launch
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    moveit_config = (MoveItConfigsBuilder("arm", package_name="arm_moveit")

                     .robot_description_semantic(os.path.join(
        get_package_share_directory('arm_moveit'), 'config', 'arm.srdf'
    ))
                     .to_moveit_configs())
    return generate_spawn_controllers_launch(moveit_config)
