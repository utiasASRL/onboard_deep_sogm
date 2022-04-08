#!/bin/bash

ROS_1_DISTRO=noetic

source "/opt/ros/$ROS_1_DISTRO/setup.bash"

# Test msg 
echo "---------------    tf   ---------------"
rosmsg list | grep "tf"
echo "---------------------------------------"

# Build
cd ../../catkin_ws
catkin_make_isolated --install

# Test msg 
source devel/setup.bash
echo " "
echo "--------------- Zeus ---------------"
rosmsg list | grep "Detections3D"
rosmsg list | grep "BoundingBox3D"
echo "---------------------------------------"
