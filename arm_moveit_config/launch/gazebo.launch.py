import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    arm_moveit_package = get_package_share_directory('arm_moveit_config')

    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('arm_control'), 'launch', 'gazebo.launch.py'
                )]), launch_arguments={
                    'use_rviz': 'False'
                }.items()
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    arm_moveit_package, 'launch', 'move_group.launch.py'
                )])
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    arm_moveit_package, 'launch', 'moveit_rviz.launch.py'
                )])
            )
        ]
    )
