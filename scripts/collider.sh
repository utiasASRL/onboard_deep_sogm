#!/bin/bash

nohup_mode=false
nav_without_sogm=false
model_path=""

while getopts ntm: option
do
case "${option}"
in
n) nohup_mode=true;;
t) nav_without_sogm=true;;
m) mapfile=${OPTARG};;
esac
done

# Source ros/ros2
ROS_1_DISTRO=noetic
source "/opt/ros/$ROS_1_DISTRO/setup.bash"
. "../../catkin_ws/devel/setup.bash"
source "/opt/ros/foxy/setup.bash"
. "../install/setup.bash"

# Get command
sogm_command="ros2 launch deep_sogm sogm_launch.py"
sogm_command="$sogm_command nav_without_sogm:=$nav_without_sogm"
sogm_command="$sogm_command model_path:=$model_path"

echo "command:    $sogm_command"

source "/opt/ros/foxy/setup.bash"
. "../install/setup.bash"

if [ "$nohup_mode" = true ] ; then

    NOHUP_FILE="../../nohup_logs/nohup_bridge.txt"
    nohup ros2 run ros1_bridge dynamic_bridge > "$NOHUP_FILE" 2>&1 &

    NOHUP_FILE="../../nohup_logs/nohup_sogm.txt"
    nohup $sogm_command > "$NOHUP_FILE" 2>&1 &


else
    xterm -bg black -fg lightgray -xrm "xterm*allowTitleOps: false" -T "Ros1-Bridge" -n "Ros1-Bridge" -hold \
        -e ros2 run ros1_bridge dynamic_bridge &
    xterm -bg black -fg lightgray -geometry 160x20+30+10 -xrm "xterm*allowTitleOps: false" -T "SOGM Prediction" -n "SOGM Prediction" -hold \
        -e $sogm_command &
    # $sogm_command
fi
