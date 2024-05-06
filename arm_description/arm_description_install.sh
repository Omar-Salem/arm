#!/bin/bash
colcon build --packages-select arm_description
source install/setup.bash
ros2 launch arm_description display.launch.py


colcon build --packages-select arm_description
source install/setup.bash
ros2 launch arm_description gazebo.launch.py