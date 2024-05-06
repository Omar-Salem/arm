#!/bin/bash

colcon build --packages-select arm_control
source install/setup.bash
ros2 launch arm_control control.launch.py

colcon build --packages-select arm_control
source install/setup.bash
ros2 launch arm_control gazebo_control.launch.py


ros2 topic pub -r 10 /forward_command_controller/commands std_msgs/msg/Float64MultiArray "layout:
  dim: []
  data_offset: 0
data: [1,2]"
