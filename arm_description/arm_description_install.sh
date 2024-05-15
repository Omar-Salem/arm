#!/bin/bash

cd arm_ws
colcon build --packages-select arm_description
source install/setup.bash
ros2 launch arm_description display.launch.py

cd arm_ws
colcon build --packages-select arm_description
source install/setup.bash
ros2 launch arm_description gazebo.launch.py