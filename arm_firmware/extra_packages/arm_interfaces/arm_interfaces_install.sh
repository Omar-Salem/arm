#!/bin/bash

cd ~/arm_ws
colcon build --packages-select arm_interfaces
. install/setup.bash
ros2 interface show arm_interfaces/msg/Motors