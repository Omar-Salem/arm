#!/bin/bash

cd arm_ws
colcon build --packages-select arm_moveit
source install/setup.bash
ros2 launch arm_moveit demo.launch.py

cd arm_ws
colcon build --packages-select arm_moveit
source install/setup.bash
ros2 launch arm_moveit move_group.launch.py

cd arm_ws
colcon build --packages-select arm_moveit
source install/setup.bash
ros2 launch arm_moveit gazebo.launch.py