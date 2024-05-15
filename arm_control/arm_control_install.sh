#!/bin/bash

cd arm_ws
colcon build --packages-select arm_control
source install/setup.bash
ros2 launch arm_control control.launch.py

cd arm_ws
colcon build --packages-select arm_control
source install/setup.bash
ros2 launch arm_control gazebo_control.launch.py


ros2 topic pub --once /joint_trajectory_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory '{joint_names:["base_joint","shoulder_joint"],points: [{positions:[1,2]}]}'
