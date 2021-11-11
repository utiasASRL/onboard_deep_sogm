#!/bin/bash

source "/opt/ros/foxy/install/setup.bash"

colcon build --symlink-install --packages-skip ros1_bridge msg_interfaces

# Test msg 
echo ""

echo "#######################################"

echo "ROS2 VoxGrid"

source install/setup.bash

ros2 interface list | grep "VoxGrid"