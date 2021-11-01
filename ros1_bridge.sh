#!/bin/bash

source "/opt/ros/melodic/setup.bash"
source "/opt/ros/eloquent/setup.bash"

. install/setup.bash

# pkill roscore
# pkill rosmaster
# pkill gzclient
# pkill gzserver
# pkill rviz

# echo "Waiting"

# until rostopic list; do sleep 0.5; done #wait until rosmaster has started

# echo "Go"

ros2 run ros1_bridge dynamic_bridge & ros2 run py_pubsub listener

