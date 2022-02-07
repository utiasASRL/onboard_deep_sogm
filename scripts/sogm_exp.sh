#!/bin/bash

# Arg to specify if we record this run or not
record=false
colli=false
nohup=false

# Get arguments
while getopts nrc option
do
case "${option}"
in
n) nohup=true;;
r) record=true;;
c) colli=true;;
esac
done


# We should start pointslam on the xavier, to remove some of the CPU load
ROS_1_DISTRO=noetic
source "/opt/ros/$ROS_1_DISTRO/setup.bash"
. "../../catkin_ws/install_isolated/setup.bash"


# Waiting for pointslam initialization
echo ""
echo "Waiting for pointslam initialization ..."
map_topic=$(rostopic list | grep "/map")
until [ -n "$map_topic" ] 
do 
    sleep 0.5
    map_topic=$(rostopic list | grep "/map")
done 
echo "OK"


# Now start move_base
move_base_command="roslaunch jackal_navigation teb_normal.launch"
if [ "$colli" = true ] ; then
    move_base_command="roslaunch jackal_navigation teb_modified.launch"
fi

echo "Running move_base : $move_base_command"
if [ "$nohup" = true ] ; then
    nohup $move_base_command > "nohup_teb.txt" 2>&1 &
else
    xterm -bg black -fg lightgray -e $move_base_command &
fi


# Abort run in case the waypoints were not good
if [ "$new_waypoints" = false ] ; then
    echo "Aborting run"
    ./stop_exp.sh
    exit 1
fi


# Start bridge and collider
if [ "$colli" = true ] ; then
    source "/opt/ros/foxy/setup.bash"
    . "../install/setup.bash"

    if [ "$nohup" = true ] ; then
        nohup ros2 run ros1_bridge dynamic_bridge > "nohup_bridge.txt" 2>&1 & 
        nohup ros2 run deep_sogm collider > "nohup_sogm.txt" 2>&1 &
    else
        xterm -bg black -fg lightgray -e ros2 run ros1_bridge dynamic_bridge &
        xterm -bg black -fg lightgray -e ros2 run deep_sogm collider &
        # ros2 run deep_sogm collider
        
    fi
fi

# Record run
if [ "$record" = true ] ; then
    echo "Record Rosbag"
    nohup ./rosbag_record.sh > "nohup_record.txt" 2>&1 &
fi


echo ""
read -p "Experiment running. Press any key to stop everything" choice
echo ""
tmp=false
case "$choice" in 
    * ) tmp=false;;
esac







