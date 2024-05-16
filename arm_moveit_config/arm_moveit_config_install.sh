#!/bin/bash

cd arm_ws
colcon build --packages-select arm_moveit_config
source install/setup.bash
ros2 launch arm_moveit_config demo.launch.py


cd arm_ws

killall -9 gazebo & killall -9 gzserver & killall -9 gzclient
killall -9 rviz
colcon build --packages-select arm_moveit_config
source install/setup.bash
ros2 launch arm_moveit_config gazebo.launch.py