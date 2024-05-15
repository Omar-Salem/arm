#!/bin/bash

cd arm_ws
colcon build --packages-select arm_moveit
source install/setup.bash
ros2 launch arm_moveit gazebo.launch.py