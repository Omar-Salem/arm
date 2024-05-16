from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_moveit_rviz_launch
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition
import os


def generate_launch_description():
    package_name = 'arm_control'
    share_dir = get_package_share_directory(package_name)
    xacro_file = os.path.join(share_dir, 'urdf', 'arm.xacro')

    robot_description_config = xacro.process_file(xacro_file, mappings={'is_sim': 'true'})
    robot_urdf = robot_description_config.toxml()

    moveit_config = (MoveItConfigsBuilder("arm", package_name="arm_moveit_config")
                     .robot_description(xacro_file)
                     .to_moveit_configs())
    return generate_moveit_rviz_launch(moveit_config)
