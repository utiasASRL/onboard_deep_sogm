#!/bin/bash

ROS_1_DISTRO=noetic
source "/opt/ros/$ROS_1_DISTRO/setup.bash"
source "/opt/ros/foxy/install/setup.bash"

#. install/setup.bash

# pkill roscore
# pkill rosmaster
# pkill gzclient
# pkill gzserver
# pkill rviz

# echo "Waiting"

# until rostopic list; do sleep 0.5; done #wait until rosmaster has started

echo ""
echo "Go"
echo ""

colcon build --symlink-install --packages-select "$@"

