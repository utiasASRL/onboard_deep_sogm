#!/bin/bash


source "/opt/ros/melodic/setup.bash"

cd ../catkin_ws

catkin_make_isolated --install

# Test msg 
echo ""
echo "###############################"
echo "ROS1 msg"

source install_isolated/setup.bash
rosmsg list | grep "VoxGrid"