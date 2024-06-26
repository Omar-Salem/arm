import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition


def generate_launch_description():
    package_name = 'arm_control'
    share_dir = get_package_share_directory(package_name)
    xacro_file = os.path.join(share_dir, 'urdf', 'arm.xacro')
    robot_description_config = xacro.process_file(xacro_file, mappings={'is_sim': 'true'})
    robot_urdf = robot_description_config.toxml()

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[
            {
                'robot_description': robot_urdf,
                'use_sim_time': True
            }
        ]
    )

    controller_nodes = create_controller_nodes()

    rviz_config_file = os.path.join(share_dir, 'config', 'display.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen',
        condition=IfCondition(LaunchConfiguration("use_rviz")),
    )

    return LaunchDescription(
        create_gazebo_nodes() +
        [
            DeclareLaunchArgument("use_rviz", default_value='True'),
            robot_state_publisher_node,
            rviz_node
        ] +
        controller_nodes
    )


def create_controller_nodes() -> list:
    """

    :rtype: list
    """
    robot_controller_names = ['joint_state_broadcaster', 'arm_controller', 'hand_controller']
    robot_controller_spawners = []
    for controller in robot_controller_names:
        robot_controller_spawners += [
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[controller]
            )
        ]
    return robot_controller_spawners


def create_gazebo_nodes() -> list:
    """

    :rtype: list
    """
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
    )
    spawn_entity = Node(package='gazebo_ros',
                        executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'arm'],
                        output='screen')
    return [gazebo, spawn_entity]
