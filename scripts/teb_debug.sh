#!/bin/bash

# Source ros here
ROS_1_DISTRO=noetic
source "/opt/ros/$ROS_1_DISTRO/setup.bash"
. "../../catkin_ws/install_isolated/setup.bash"

echo "Running TEB debug"
# roslaunch teb_local_planner test_optim_node.launch &

# until rostopic list; do sleep 0.5; done #wait until rosmaster has started 

rosrun teb_local_planner costmap_from_rosbag.py
 
exit 1



