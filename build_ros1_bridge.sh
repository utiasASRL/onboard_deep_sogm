#!/bin/bash


source "/opt/ros/eloquent/setup.bash"
source "/opt/ros/melodic/setup.bash"

# Source your ROS 2 installation:
. "/opt/ros/eloquent/local_setup.bash"

# And if you have a ROS 1 overlay workspace, something like:
. "../catkin_ws/devel/setup.bash"

# And if you have a ROS 2 overlay workspace, something like:
. "install/local_setup.bash"

colcon build --symlink-install --packages-select ros1_bridge --cmake-force-configure
