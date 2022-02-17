#!/bin/bash

cd ..

ROS_1_DISTRO=noetic

source "/opt/ros/$ROS_1_DISTRO/setup.bash"
. "../catkin_ws/install_isolated/setup.bash"

rviz -d rviz/colli-exp.rviz