#!/bin/bash

ROS_1_DISTRO=noetic

cd ..

# source "/opt/ros/eloquent/setup.bash"
source "/opt/ros/$ROS_1_DISTRO/setup.bash"

# And if you have a ROS 1 overlay workspace, something like:
. "../catkin_ws/devel/setup.bash"

# Source your ROS 2 installation:
. "/opt/ros/foxy/setup.bash"

# And if you have a ROS 2 overlay workspace, something like:
. "install/local_setup.bash"

#
echo ""
echo $CMAKE_PREFIX_PATH
echo ""

# Parallel build command
# colcon build --symlink-install --packages-select ros1_bridge --cmake-force-configure --cmake-args -DCMAKE_BUILD_TYPE=Release

# If the exectution fails because computer is not powerful enough use this command instead :
MAKEFLAGS="-j1 -l1" colcon build --executor sequential --symlink-install --packages-select ros1_bridge --cmake-force-configure --cmake-args -DCMAKE_BUILD_TYPE=Release


# Verify the custom types were recognized by the bridge, by printing all pairs of bridged types. 
# The custom types should be listed:
# echo ""
# echo "#####################################"
# echo "Bridged VoxGrid?"
# source "install/setup.bash"
# ros2 run ros1_bridge dynamic_bridge --print-pairs | grep "Vox"