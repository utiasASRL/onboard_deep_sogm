#!/bin/bash

ROS_1_DISTRO=noetic
source "/opt/ros/$ROS_1_DISTRO/setup.bash"
. "../catkin_ws/install_isolated/setup.bash"
source "/opt/ros/foxy/setup.bash"

. install/setup.bash

# pkill roscore
# pkill rosmaster
# pkill gzclient
# pkill gzserver
# pkill rviz

# echo "Waiting"

# until rostopic list; do sleep 0.5; done #wait until rosmaster has started

# echo "Go"

nohup ros2 run ros1_bridge dynamic_bridge > "nohup.txt" 2>&1 & ros2 run deep_sogm collider
# nohup ros2 run deep_sogm collider > "nohup.txt" 2>&1 & ros2 run ros1_bridge dynamic_bridge
