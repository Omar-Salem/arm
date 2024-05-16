import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

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
    is_sim = LaunchConfiguration('is_sim')

    moveit_config = (
        MoveItConfigsBuilder("arm", package_name="arm_moveit_config")
        .robot_description(os.path.join(get_package_share_directory('arm_control'), 'urdf', 'arm.xacro'))
        .robot_description_semantic(file_path='config/arm.srdf')
        .trajectory_execution(file_path='config/moveit_controllers.yaml')
        .to_moveit_configs()
    )

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {'use_sim_time': is_sim},
            {'publish_robot_description_semantic': True}
        ],
        arguments=["--ros-args", "--log-level", "info"],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="log",
        respawn=False,
        arguments=["-d", os.path.join(get_package_share_directory('arm_moveit_config'), 'config', 'moveit.rviz')],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
            # moveit_config.planning_pipelines,
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                name='is_sim',
                default_value='True'
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('arm_control'), 'launch', 'gazebo.launch.py'
                )]), launch_arguments={
                    'use_rviz': 'False'
                }.items()
            ),
            move_group_node,
            rviz_node
        ]
    )
