#!/bin/bash

source "/opt/ros/melodic/setup.bash"
source "/opt/ros/eloquent/setup.bash"

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

