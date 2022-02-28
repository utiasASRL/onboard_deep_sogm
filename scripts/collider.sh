#!/bin/bash

# Get to main folder
cd ..

# Source ros/ros2
ROS_1_DISTRO=noetic
source "/opt/ros/$ROS_1_DISTRO/setup.bash"
. "../catkin_ws/install_isolated/setup.bash"
source "/opt/ros/foxy/setup.bash"

. install/setup.bash

# Open terminals or nohup
nohup=false

if [ "$nohup" = true ] ; then

    # Start bridge in background and collider here
    nohup ros2 run ros1_bridge dynamic_bridge > "nohup_bridge.txt" 2>&1 & ros2 run deep_sogm collider

    # Start collider in background and bridge here 
    # nohup ros2 run deep_sogm collider > "nohup_sogm.txt" 2>&1 & ros2 run ros1_bridge dynamic_bridge

else

    # Start bridge in another terminal and collider here
    xterm -bg black -fg lightgray -e ros2 run ros1_bridge dynamic_bridge &
    ros2 run deep_sogm collider

fi
