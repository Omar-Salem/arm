#!/bin/bash

cd arm_ws
colcon build --packages-select arm_moveit && source install/setup.bash && ros2 run arm_moveit hello_moveit --ros-args -p use_sim_time:=true