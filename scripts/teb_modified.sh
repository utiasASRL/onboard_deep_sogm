#!/bin/bash


ROS_1_DISTRO=noetic
source "/opt/ros/$ROS_1_DISTRO/setup.bash"
. "../catkin_ws/devel/setup.bash"

echo "Running move_base with modified TEB"
roslaunch jackal_navigation teb_modified.launch

