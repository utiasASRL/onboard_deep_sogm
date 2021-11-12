#!/bin/bash

source "/opt/ros/foxy/install/setup.bash"

colcon build --symlink-install --packages-skip ros1_bridge

# Test msg 
echo ""
echo "#######################################"
echo ""


source install/setup.bash

echo "--------------- VoxGrid ---------------"
ros2 interface list | grep "VoxGrid"
echo "---------------    tf   ---------------"
ros2 interface list | grep "tf"
echo "---------------------------------------"