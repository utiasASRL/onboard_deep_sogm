#!/bin/bash

source "/opt/ros/eloquent/setup.bash"

colcon build --symlink-install --packages-skip ros1_bridge

# Test msg 
echo ""

echo "#######################################"

echo "ROS2 VoxGrid"

source install/setup.bash

ros2 interface list | grep "VoxGrid"