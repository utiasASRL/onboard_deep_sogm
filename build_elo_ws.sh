#!/bin/bash

source "/opt/ros/foxy/setup.bash"

# Test msg 
echo "---------------    test tf   ---------------"
ros2 interface list | grep "tf"
echo "--------------------------------------------"

# Build
colcon build --symlink-install --packages-skip ros1_bridge

# Test msg 
source install/setup.bash
echo " "
echo "--------------- test VoxGrid ---------------"
ros2 interface list | grep "VoxGrid"
echo "--------------------------------------------"