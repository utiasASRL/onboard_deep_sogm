#!/bin/bash

echo ""
echo ""
echo ""
echo "    /--------------------------\\"
echo "   <  Replaying Rosbag in Rviz  >"
echo "    \\--------------------------/"
echo ""
echo ""

# Arg to specify if we record this run or not
path="$HOME/results/rosbag_data"
rosbag="2022-04-11_18-51-49.bag"

# Get arguments
while getopts b: option
do
case "${option}"
in
b) rosbag=${OPTARG};;
esac
done

# Source ros here
cd ..
ROS_1_DISTRO=noetic
source "/opt/ros/$ROS_1_DISTRO/setup.bash"
. "../catkin_ws/devel/setup.bash"

echo "$ROS_MASTER_URI"
# ROS_MASTER_URI="http://localhost:11311"
echo "$ROS_MASTER_URI"

# Create a ROS master
xterm -bg black -fg lightgray -geometry 160x20+30+10 -xrm "xterm*allowTitleOps: false" -T "roscore" -n "roscore" -hold \
    -e roscore &

echo " "
echo "Waiting for roscore initialization ..."
echo " "
until rostopic list; do sleep 0.5; done #wait until rosmaster has started 

# launch a robot decsription
xterm -bg black -fg lightgray -geometry 160x20+30+10 -xrm "xterm*allowTitleOps: false" -T "description" -n "description" -hold \
    -e roslaunch jackal_description description.launch &

# rosparam set /use_sim_time true
xterm -bg black -fg lightgray -geometry 160x20+30+10 -xrm "xterm*allowTitleOps: false" -T "rviz" -n "rviz" -hold \
    -e rviz -d rviz/replay-stairs.rviz &

# echo ""
# read -p "When ready to replay. Press any key" choice
# echo ""
# tmp=false
# case "$choice" in 
#     * ) tmp=false;;
# esac

sleep 2.0

rosbag play --clock -r 1.0 "$path/$rosbag" 
 
exit 1



