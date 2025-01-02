#!/bin/bash

# flash basic first
# check connections
# check battery is on

# clean platformio
# flash esp32
pio pkg install # Install dependencies
pio run --target upload # Flash the firmware


ls /dev/ttyUSB* | grep ttyUSB0 #check /dev/ttyUSB0
lsusb | grep CP210x

# terminal 1
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0

# terminal 2
cd ~/arm_ws
colcon build --packages-select arm_interfaces
source install/setup.bash 
ros2 topic list -t | grep arm/motors #check for /arm/motors_cmd and /arm/motors_state

# positions
ros2 topic pub --once /arm/motors_cmd arm_interfaces/msg/Motors "base_link: 6.28" 
