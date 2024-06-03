#!/bin/bash

cd arm_ws
colcon build --packages-select arm_control
source install/setup.bash
ros2 launch arm_control control.launch.py

cd arm_ws
killall -9 gazebo & killall -9 gzserver & killall -9 gzclient
killall -9 rviz
colcon build --packages-select arm_control
source install/setup.bash
ros2 launch arm_control gazebo.launch.py


ros2 topic pub --once /arm_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory '{joint_names:["joint1","joint2","joint3","joint4","joint5","joint6","joint7"],points: [{positions:[2,4,6,9,1,1,2]}]}'
ros2 topic pub --once /hand_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory '{joint_names:["finger_joint"],points: [{positions:[-45]}]}'
