#!/bin/bash

# Arg to specify if we record this run or not
record=false
sogm=false
nohup=false
waypoints="default_no_given"

# Get arguments
while getopts nrsw: option
do
case "${option}"
in
n) nohup=true;;
r) record=true;;
s) sogm=true;;
w) waypoints=${OPTARG};;
esac
done

# First of all start pointslam on Xavier board
point_slam_command="cd catkin_ws/scripts/ && ./point_slam.sh"
ssh_command="ssh -i $HOME/.ssh/id_rsa administrator@cpr-tor59-xav01 $point_slam_command"
echo "Running PointSlam: $ssh_command"
if [ "$nohup" = true ] ; then
    nohup $ssh_command > "nohup_point_slam.txt" 2>&1 &
else
    xterm -bg black -fg lightgray -xrm "xterm*allowTitleOps: false" -T "PointSlam" -n "PointSlam" -hold \
        -e $ssh_command &
fi

# Source ros here
ROS_1_DISTRO=noetic
source "/opt/ros/$ROS_1_DISTRO/setup.bash"
. "../../catkin_ws/install_isolated/setup.bash"

# Waiting for pointslam initialization
echo ""
echo "Waiting for PointSlam initialization ..."
until [ -n "$map_topic" ] 
do 
    sleep 0.1
    map_topic=$(rostopic list -p | grep "/map")
done 
until [[ -n "$point_slam_msg" ]]
do 
    sleep 0.1
    point_slam_msg=$(rostopic echo -n 1 /map | grep "frame_id")
done 
echo "OK"

# Now start move_base
move_base_command="roslaunch jackal_navigation teb_normal.launch"
if [ "$sogm" = true ] ; then
    move_base_command="roslaunch jackal_navigation teb_modified.launch"
fi

echo "Running move_base : $move_base_command"
if [ "$nohup" = true ] ; then
    nohup $move_base_command > "nohup_teb.txt" 2>&1 &
else
    xterm -bg black -fg lightgray -xrm "xterm*allowTitleOps: false" -T "Move Base" -n "Move Base" -hold \
        -e $move_base_command &
fi


# Abort run in case the waypoints were not good
if [ "$new_waypoints" = false ] ; then
    echo "Aborting run"
    ./stop_exp.sh
    exit 1
fi


# Start bridge and collider
if [ "$sogm" = true ] ; then
    source "/opt/ros/foxy/setup.bash"
    . "../install/setup.bash"

    if [ "$nohup" = true ] ; then
        nohup ros2 run ros1_bridge dynamic_bridge > "nohup_bridge.txt" 2>&1 & 
        nohup ros2 run deep_sogm collider > "nohup_sogm.txt" 2>&1 &
    else
        xterm -bg black -fg lightgray -xrm "xterm*allowTitleOps: false" -T "Ros1-Bridge" -n "Ros1-Bridge" -hold \
            -e ros2 run ros1_bridge dynamic_bridge &
        xterm -bg black -fg lightgray -xrm "xterm*allowTitleOps: false" -T "SOGM Prediction" -n "SOGM Prediction" -hold \
            -e ros2 run deep_sogm collider &
        # ros2 run deep_sogm collider
    fi

    # Before going further wait for collider to be running
    source "/opt/ros/$ROS_1_DISTRO/setup.bash"
    . "../../catkin_ws/install_isolated/setup.bash"
    echo ""
    echo "Waiting for SOGM predictions ..."
    until [ -n "$sogm_topic" ] 
    do 
        sleep 0.1
        sogm_topic=$(rostopic list -p | grep "/plan_costmap_3D")
    done 
    until [[ -n "$sogm_msg" ]]
    do 
        sleep 0.1
        sogm_msg=$(rostopic echo -n 1 /plan_costmap_3D | grep "frame_id")
    done 
    echo "OK"

fi

# Start waypoint follower
if [ $waypoints != "default_no_given" ]; then

    # First of all start pointslam on Xavier board
    ssh -i $HOME/.ssh/id_rsa administrator@cpr-tor59-xav01 "cd catkin_ws/scripts/ && ./teb_experiment.sh -w $waypoints"
        
    echo ""
    read -p "When ready to run. Press any key" choice
    echo ""
    tmp=false
    case "$choice" in 
        * ) tmp=false;;
    esac
    rostopic pub /start_journey std_msgs/Empty -1

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

# Stop everything
./stop_exp.sh







