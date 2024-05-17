#!/bin/bash

ros2 launch moveit_setup_assistant setup_assistant.launch.py

cd arm_ws
rm -rf build log install
colcon build


cd arm_ws
killall -9 gazebo & killall -9 gzserver & killall -9 gzclient
killall -9 rviz
colcon build --packages-select arm_moveit_config
source install/setup.bash
ros2 launch arm_moveit_config gazebo.launch.py