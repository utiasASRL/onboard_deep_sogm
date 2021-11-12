#!/bin/bash

source "/opt/ros/melodic/setup.bash"

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
echo "---------------------------------------"