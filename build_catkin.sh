#!/bin/bash

ROS_1_DISTRO=noetic

source "/opt/ros/$ROS_1_DISTRO/setup.bash"

# Test msg 
echo "---------------    tf   ---------------"
rosmsg list | grep "tf"
echo "---------------------------------------"

# Build
cd ../catkin_ws
catkin_make_isolated --install

# Test msg 
source install_isolated/setup.bash
echo " "
echo "--------------- VoxGrid ---------------"
rosmsg list | grep "VoxGrid"
rosmsg list | grep "Obstacle"
echo "---------------------------------------"