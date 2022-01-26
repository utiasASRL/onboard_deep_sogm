#!/bin/bash

ROS_1_DISTRO=noetic

source "/opt/ros/$ROS_1_DISTRO/setup.bash"
. "../catkin_ws/install_isolated/setup.bash"

rviz -d teb-exp.rviz