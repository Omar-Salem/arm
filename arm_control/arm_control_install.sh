#!/bin/bash

colcon build --packages-select arm_control
source install/setup.bash
ros2 launch arm_control control.launch.py

colcon build --packages-select arm_control
source install/setup.bash
ros2 launch arm_control gazebo_control.launch.py


ros2 topic pub -r 10 /joint_trajectory_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory '{joint_names:["shoulder_joint"],points: [{positions:[1]}]}'
