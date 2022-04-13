#!/bin/bash
echo ""
echo ""
echo ""
echo "    /------------------------\\"
echo "   <  SOGM experiment master  >"
echo "    \\------------------------/"
echo ""
echo ""

# Arg to specify if we record this run or not
record=false
sogm=false
nav_without_sogm=false
nohup=false
waypoints="default_no_given"
mapfile=""

# Get arguments
while getopts nrstw:m: option
do
case "${option}"
in
n) nohup=true;;
r) record=true;;
s) sogm=true;;
t) nav_without_sogm=true;;
w) waypoints=${OPTARG};;
m) mapfile=${OPTARG};;
esac
done

# Which map are we using
if [ "$mapfile" = "" ] ; then
    mapfile="map_Myhal_1.ply"
fi





# TODO: Now test running with sogm downstairs, everything should be working quite well. Just ensure that you have the latest trained model 








# Get corresponding trained session
trained_session=""
chkp_name=""
if [ "$mapfile" = "map_Myhal_1.ply" ] ; then
    trained_session="Log_2022-03-23_21-08-49"
    chkp_name="chkp_0580.tar"

elif [ "$mapfile" = "map_Myhal_5.ply" ] ; then
    trained_session="Log_2022-01-21_16-44-32"
    chkp_name="chkp_0600.tar"

else
    echo ""
    echo "ERROR: Unkown map file, aborting run"
    echo ""
    exit 1
fi

# Full path
model_path="$HOME/results/pretrained_logs/$trained_session/checkpoints/$chkp_name"

# First of all start pointslam on Xavier board
point_slam_command="cd catkin_ws/scripts/ && ./point_slam.sh"
if [ "$sogm" = true ] ; then
    point_slam_command="$point_slam_command -f"
fi
point_slam_command="$point_slam_command -m \"$mapfile\""
ssh_command="ssh -i $HOME/.ssh/id_rsa administrator@cpr-tor59-xav01 $point_slam_command"
echo ""
echo "Running PointSlam via ssh. Command used:"
echo "$ssh_command"
if [ "$nohup" = true ] ; then
    nohup $ssh_command > "nohup_point_slam.txt" 2>&1 &
else
    xterm -bg black -fg lightgray -geometry 120x30+40+20 -xrm "xterm*allowTitleOps: false" -T "PointSlam" -n "PointSlam" -hold \
        -e $ssh_command &
fi
echo "OK"
echo ""
echo "------------------------------------------"
echo ""


# Source ros here
echo ""
echo "Sourcing ROS Noetic"
ROS_1_DISTRO=noetic
source "/opt/ros/$ROS_1_DISTRO/setup.bash"
. "../../catkin_ws/devel/setup.bash"
echo "OK"
echo ""
echo "------------------------------------------"
echo ""


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
echo ""
echo "------------------------------------------"
echo ""

# Now start move_base
#echo ""
#echo "Running move_base. Command used:"
#move_base_command="roslaunch jackal_navigation teb_normal.launch"
#if [ "$sogm" = true ] && [ "$nav_without_sogm" = false ]; then
#    move_base_command="roslaunch jackal_navigation teb_modified.launch"
#fi
#echo "$move_base_command"
#
#if [ "$nohup" = true ] ; then
#    nohup $move_base_command > "nohup_teb.txt" 2>&1 &
#else
#    xterm -bg black -fg lightgray -xrm "xterm*allowTitleOps: false" -T "Move Base" -n "Move Base" -hold \
#        -e $move_base_command &
#fi
#
#echo "OK"
#echo ""
#echo "------------------------------------------"
#echo ""

# Start bridge and collider
if [ "$sogm" = true ] ; then
    echo ""
    echo "Running deep_sogm collider. Command used:"
    
    sogm_command="ros2 launch deep_sogm sogm_launch.py"
    sogm_command="$sogm_command nav_without_sogm:=$nav_without_sogm"
    sogm_command="$sogm_command model_path:=$model_path"

    echo "$sogm_command"
    
    source "/opt/ros/foxy/setup.bash"
    . "../install/setup.bash"

    if [ "$nohup" = true ] ; then
        nohup ros2 run ros1_bridge dynamic_bridge > "nohup_bridge.txt" 2>&1 & 
        nohup $sogm_command > "nohup_sogm.txt" 2>&1 &
    else
        xterm -bg black -fg lightgray -xrm "xterm*allowTitleOps: false" -T "Ros1-Bridge" -n "Ros1-Bridge" -hold \
            -e ros2 run ros1_bridge dynamic_bridge &
        xterm -bg black -fg lightgray -geometry 160x20+30+10 -xrm "xterm*allowTitleOps: false" -T "SOGM Prediction" -n "SOGM Prediction" -hold \
            -e $sogm_command &
        # $sogm_command
    fi
    echo "OK"
    echo ""
    echo "------------------------------------------"
    echo ""

    # Before going further wait for collider to be running
    source "/opt/ros/$ROS_1_DISTRO/setup.bash"
    . "../../catkin_ws/devel/setup.bash"
    echo ""
    echo "Waiting for SOGM predictions ..."
    until [ -n "$sogm_topic" ] 
    do 
        sleep 0.1
        sogm_topic=$(rostopic list -p | grep "/classified_points")
    done 
    until [[ -n "$sogm_msg" ]]
    do 
        sleep 0.1
        sogm_msg=$(rostopic echo -n 1 /classified_points | grep "frame_id")
    done 
    
    echo "OK"
    echo ""
    echo "------------------------------------------"
    echo ""

fi

# Start waypoint follower
if [ $waypoints != "default_no_given" ]; then

    echo ""
    echo "Running waypoints node via ssh:"
    echo ""

    # First of all start pointslam on Xavier board
    ssh -i $HOME/.ssh/id_rsa administrator@cpr-tor59-xav01 "cd catkin_ws/scripts/ && ./teb_experiment.sh -w $waypoints"
        
    echo ""
    read -p "When ready to run. Press any key" choice
    echo ""
    tmp=false
    case "$choice" in 
        * ) tmp=false;;
    esac

    # Record run
    if [ "$record" = true ] ; then
        echo "Record Rosbag"
        nohup ./rosbag_record.sh > "nohup_record.txt" 2>&1 &
    fi

    # Start waypoints
    rostopic pub /start_journey std_msgs/Empty -1

    echo "OK"
    echo ""
    echo "------------------------------------------"
    echo ""

else

    # Record run
    if [ "$record" = true ] ; then
        echo "Record Rosbag"
        nohup ./rosbag_record.sh > "nohup_record.txt" 2>&1 &
    fi

fi

# Now start object_detection and crowd_planner nodes
echo ""
echo "Starting Object Detection / Crowd Planner"
perception_command="roslaunch crowd_planner crowd_planner.launch"
echo "$perception_command"

if [ "$nohup" = true ] ; then
    nohup $move_base_command > "nohup_teb.txt" 2>&1 &
else
    xterm -bg black -fg lightgray -xrm "xterm*allowTitleOps: false" -T "Move Base" -n "Move Base" -hold \
        -e $move_base_command &
fi

echo "OK"
echo ""
echo "------------------------------------------"
echo ""

echo ""
read -p "Experiment running. Press any key to stop everything" choice
echo ""
tmp=false
case "$choice" in 
    * ) tmp=false;;
esac

# Stop everything
./stop_exp.sh


