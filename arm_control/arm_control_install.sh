#!/bin/bash

## Real
# Arm
cd ~/arm_ws && colcon build --packages-select arm_control && source install/setup.bash && ros2 launch arm_control control.launch.py

# Laptop
cd ~/arm_ws && colcon build --packages-select arm_control && source install/setup.bash && ros2 launch arm_control control.launch.py micro_ros_port:=/dev/ttyUSB0 use_gui:=True


## Sim
cd ~/arm_ws && colcon build --packages-select arm_control && source install/setup.bash && ros2 launch arm_control gazebo.launch.py


ros2 topic pub --once /arm_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory '{joint_names:["joint_1","joint_2"],points: [{positions:[3.141,0]}]}'
ros2 topic pub --once /hand_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory '{joint_names:["finger_joint"],points: [{positions:[-45]}]}'
