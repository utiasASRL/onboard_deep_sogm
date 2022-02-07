#!/bin/bash


# First kill move_base in case of emergency
ROS_1_DISTRO=noetic
source "/opt/ros/$ROS_1_DISTRO/setup.bash"
rosnode kill move_base

all_nodes=""
for node_name in "frame_update" "record_" "follow_waypoints"
do
    node=$(rosnode list | grep "$node_name")
    if [ -n "$node" ]; then
        if [ ${node:0:1} = "/" ]; then
            all_nodes="$all_nodes $node"
        fi
    fi
done

if [ -n "$all_nodes" ]; then
    rosnode kill $all_nodes
else
    echo "No experiment node running"
fi