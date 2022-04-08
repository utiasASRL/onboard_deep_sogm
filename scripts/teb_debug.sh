#!/bin/bash

# Source ros here
ROS_1_DISTRO=noetic
source "/opt/ros/$ROS_1_DISTRO/setup.bash"
. "../../catkin_ws/devel/setup.bash"

echo "waiting for ros master"
until rostopic list; do sleep 0.5; done #wait until rosmaster has started 

echo "Running TEB debug"
roslaunch teb_local_planner test_optim_node.launch &

echo "Start reconfigure"
rosrun rqt_reconfigure rqt_reconfigure &

echo "Start python rosbag publisher"
rosrun teb_local_planner costmap_from_rosbag.py
 
exit 1



