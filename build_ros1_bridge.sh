#!/bin/bash

# source "/opt/ros/eloquent/setup.bash"
source "/opt/ros/melodic/setup.bash"

# Source your ROS 2 installation:
. "/opt/ros/foxy/install/setup.bash"

# And if you have a ROS 1 overlay workspace, something like:
. "../catkin_ws/install_isolated/setup.bash"

# And if you have a ROS 2 overlay workspace, something like:
. "install/local_setup.bash"

cd ../bridge_ws

colcon build --cmake-force-configure

ros2 interface list | grep "VoxGrid"

# Verify the custom types were recognized by the bridge, by printing all pairs of bridged types. 
# The custom types should be listed:
# echo ""
# echo "#####################################"
# echo "Bridged VoxGrid?"
# source "install/setup.bash"
ros2 run ros1_bridge dynamic_bridge --print-pairs | grep "Vox"