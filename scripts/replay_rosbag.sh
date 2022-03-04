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
rosbag="2022-03-01_14-36-02.bag"

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
. "../catkin_ws/install_isolated/setup.bash"

rosparam set /use_sim_time true

echo ""
read -p "When ready to replay. Press any key" choice
echo ""
tmp=false
case "$choice" in 
    * ) tmp=false;;
esac

rosparam get /use_sim_time

rviz -d rviz/colli-exp.rviz &

echo ""
read -p "When ready to replay. Press any key" choice
echo ""
tmp=false
case "$choice" in 
    * ) tmp=false;;
esac

rosbag play --clock -r 1 "$path/$rosbag" 
 
exit 1



