import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_name = "arm_control"
    share_dir = get_package_share_directory(package_name)
    xacro_file = os.path.join(share_dir, "urdf", "arm.xacro")
    robot_description_config = xacro.process_file(
        xacro_file, mappings={"is_sim": "true"}
    )
    robot_urdf = robot_description_config.toxml()

    controller_nodes = create_controller_nodes()

    arm_description_launch = [
        os.path.join(get_package_share_directory("arm_description"), "launch"),
        "/gazebo.launch.py",
    ]

    rviz_config_file_arg = DeclareLaunchArgument(
        name="rviz_config_file",
        default_value=os.path.join(share_dir, "config", "display.rviz"),
    )
    return LaunchDescription(
        [
            rviz_config_file_arg,
            GroupAction(
                actions=[
                    SetParameter(name="use_sim_time", value=True),
                    #         Node(
                    #     package="odometry_test",
                    #     executable="square_follower",
                    # ),
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(arm_description_launch),
                        launch_arguments={
                            "robot_urdf": robot_urdf,
                            "use_gui": "True",
                            "use_joint_state_publisher": "False",
                #             "world":os.path.join(
                #     get_package_share_directory("arm_description"),
                #     "worlds",
                #     "walls",
                # ),
                            "rviz_config_file": LaunchConfiguration("rviz_config_file"),
                        }.items(),
                    ),
                ]
                + controller_nodes
            ),
        ]
    )


def create_controller_nodes() -> list:
    """

    :rtype: list
    """
    controllers_params = PathJoinSubstitution(
        [
            FindPackageShare("arm_control"),
            "config",
            "arm_controllers.yaml",
        ]
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )
    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "arm_controller",
            "--param-file",
            controllers_params,
        ],
    )
    return [joint_state_broadcaster_spawner, arm_controller_spawner]
