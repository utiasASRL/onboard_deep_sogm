#!/bin/bash

# ROS_1_DISTRO=noetic
# source "/opt/ros/$ROS_1_DISTRO/setup.bash"
# source "/opt/ros/eloquent/setup.bash"

cd ..

. install/setup.bash

# pkill roscore
# pkill rosmaster
# pkill gzclient
# pkill gzserver
# pkill rviz

# echo "Waiting"

# until rostopic list; do sleep 0.5; done #wait until rosmaster has started

# echo "Go"


nohup ros2 run ros1_bridge dynamic_bridge > "nohup2.txt" 2>&1 & ros2 run py_pubsub tf2_listener
# nohup ros2 run py_pubsub tf2_listener > "nohup2.txt" 2>&1 & ros2 run ros1_bridge static_bridge