#!/bin/bash

########
# Init #
########

echo " "
echo " "
echo -e "\033[1;32m+--------------------------------+\033[0m"
echo -e "\033[1;32m|   Navigation with TEB + SOGM   |\033[0m"
echo -e "\033[1;32m+--------------------------------+\033[0m"
echo " "
echo "This scripts starts a navigatio session with TEB planner, and point_slam in SLAM mode"


# Global source
ROS_1_DISTRO=noetic
source "/opt/ros/$ROS_1_DISTRO/setup.bash"
export ROS_HOSTNAME=polus-laptop
export ROS_MASTER_URI=http://cpr-tor59-02:11311

# Xterm variables
export TERM=term-256color
echo "xterm*background: black" >> .Xresources
echo "xterm*foreground: lightgray" >> .Xresources
echo "xterm*faceName: monospace:pixelsize=14" >> .Xresources

# local ws source
. "../../catkin_ws/devel/setup.bash"


##############
# Input args #
##############

nohup_mode=false
show_rviz=false
sogm=false
waypoints="default_no_given"
nav_without_sogm=false
record_bag=false
mapfile=""
trained_session=""
chkp_name=""

while getopts nvstrw:m: option
do
case "${option}"
in
n) nohup_mode=true;;
v) show_rviz=true;;
s) sogm=true;;
t) nav_without_sogm=true;;
r) record_bag=true;;
w) waypoints=${OPTARG};;
m) mapfile=${OPTARG};;
esac
done

# Which map are we using
if [ "$mapfile" = "AppleMap" ] ; then
    mapfile="$PWD/../../results/maps/map_Apple1.ply"

    # trained_session="Log_2022-05-27_16-46-35"     # old
    # chkp_name="chkp_0300.tar"

    trained_session="Log_2022-12-10_19-40-30"       # New
    chkp_name="chkp_0240.tar"

    
fi

# Model Full path
model_path="../../results/pretrained_logs/$trained_session/checkpoints/$chkp_name"


##################
# Initial checks #
##################

echo " "
echo -e "\033[1;4;34mWaiting for velodyne messages\033[0m"

# Wait until rosmaster has started (Timeout 2 seconds)
rostopics=""
for i in {1..20}
do
    if [[ -z "$rostopics" ]] ; then
        rostopics="$(rostopic list)"
        sleep 0.1
    fi

    if [[ ! -z "$rostopics" ]] ; then
        break
    fi
done
if [[ -z "$rostopics" ]] ; then
    echo -e "\033[1;31mError: cannot list rostopics\033[0m"
    echo " "
    exit
fi

# Wait until we recieve point_clouds message from velodyne (Timeout 10 seconds)
velo_state_msg=""
for i in {1..5}
do
    if [[ -z "$velo_state_msg" ]] ; then
        velo_state_msg=$(timeout 2 rostopic echo -n 1 /velodyne_points | grep "header")
    fi

    if [[ ! -z "$velo_state_msg" ]] ; then
        break
    fi
done


if [[ -z "$velo_state_msg" ]] ; then
    echo -e "\033[1;33mWarning: no /velodyne_points message recieved.\033[0m"
    echo -e "\033[1;33mStarting velodyne node on orin (cpr-tor-xav02).\033[0m"

    # Start rosbag record on orin
    velo_command="cd 0-VelodyneMapping && ./start_velodyne.sh"
    ssh_command="ssh -i $HOME/.ssh/id_rsa polus@cpr-tor59-xav02 $velo_command"
    $ssh_command
    echo "OK"

    # Check again for messages
    velo_state_msg=""
    for i in {1..5}
    do
        if [[ -z "$velo_state_msg" ]] ; then
            velo_state_msg=$(timeout 2 rostopic echo -n 1 /velodyne_points | grep "header")
        fi

        if [[ ! -z "$velo_state_msg" ]] ; then
            break
        fi
    done

    # Stop if still nothing
    if [[ -z "$velo_state_msg" ]] ; then
        echo -e "\033[1;31mError: No point cloud message recieved from lidar\033[0m"
        echo " "
        exit
    fi
fi

echo "OK"


##############
# Start slam #
##############


echo " "
echo " "
echo -e "\033[1;4;34mStarting point_slam\033[0m"

# Command to start a simple slam node
loc_launch="roslaunch point_slam point_slam.launch init_map_path:=$mapfile"

if [ "$nohup_mode" = true ] ; then

    echo -e "In nohup mode"
    NOHUP_FILE="../../results/nohup_logs/nohup_pointmap.txt"
    nohup $loc_launch > "$NOHUP_FILE" 2>&1 &

    echo -e "Process running. See log saved at $NOHUP_FILE"
    echo " "

else

    xterm -bg black -fg lightgray -geometry 160x20+5+5 \
    -xrm "xterm*allowTitleOps: false" -T "Localization" -n "Localization" -hold \
    -e $loc_launch &
    
fi

# Waiting for pointslam initialization
echo " "
echo " "
echo -e "\033[1;4;34mWaiting for PointSlam initialization ...\033[0m"
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


#############
# Start Nav #
#############

echo " "
echo -e "\033[1;4;34mStarting navigation with move_base\033[0m"

# Now start move_base
move_base_command="roslaunch tor59_jackal teb_normal.launch"
if [ "$sogm" = true ] && [ "$nav_without_sogm" = false ]; then
    move_base_command="roslaunch tor59_jackal teb_modified.launch"
fi
echo "Command used: $move_base_command"

if [ "$nohup_mode" = true ] ; then

    NOHUP_MOVE_FILE="../../results/nohup_logs/nohup_teb.txt"
    nohup $move_base_command > "$NOHUP_MOVE_FILE" 2>&1 &

else

    xterm -bg black -fg lightgray -geometry 160x20+5+300 \
        -xrm "xterm*allowTitleOps: false" \
        -T "Move Base" \
        -n "Move Base" \
        -hold \
        -e $move_base_command &
fi


######################
# Start Deep Network #
######################


if [ "$sogm" = true ] ; then
        
    echo " "
    echo -e "\033[1;4;34mRunning deep_sogm collider\033[0m"

    sogm_command="ros2 launch deep_sogm sogm_launch.py"
    sogm_command="$sogm_command nav_without_sogm:=$nav_without_sogm"
    sogm_command="$sogm_command model_path:=$model_path"

    echo "$sogm_command"
    
    source "/opt/ros/foxy/setup.bash"
    . "../install/setup.bash"

    if [ "$nohup_mode" = true ] ; then
        NOHUP_FILE="../../results/nohup_logs/nohup_bridge.txt"
        nohup ros2 run ros1_bridge dynamic_bridge > "$NOHUP_FILE" 2>&1 & 
        NOHUP_FILE="../../results/nohup_logs/nohup_sogm.txt"
        nohup $sogm_command > "$NOHUP_FILE" 2>&1 &
    else
        xterm -bg black -fg lightgray -geometry 80x12+5-5 -xrm "xterm*allowTitleOps: false" \
            -T "Ros1-Bridge" -n "Ros1-Bridge" -hold \
            -e ros2 run ros1_bridge dynamic_bridge &
        xterm -bg black -fg lightgray -geometry 160x45+5+595 -xrm "xterm*allowTitleOps: false" \
            -T "SOGM Prediction" -n "SOGM Prediction" -hold \
            -e $sogm_command &
        # $sogm_command
    fi

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

fi


###########################
# Start Waypoint follower #
###########################

# Start waypoint follower
if [ $waypoints != "default_no_given" ]; then

    echo " "
    echo -e "\033[1;4;34mRunning follow_waypoints\033[0m"

    NOHUP_FILE="../../results/nohup_logs/nohup_waypoint.txt"
    waypoint_command="roslaunch follow_waypoints follow_waypoints.launch"
    waypoint_command="$waypoint_command waypoint_file:=$waypoints"
    nohup roslaunch follow_waypoints follow_waypoints.launch waypoint_file:="$waypoints" > "$NOHUP_FILE" 2>&1 &

    # Check if the waypoint file exists
    if [ ! -f "../../catkin_ws/src/follow_waypoints/saved_path/$waypoints.csv" ]; then

        echo " "
        echo -e "\033[1;4;34mNew waypoints creation:\033[0m"
        echo "New waypoints file: $waypoints.csv"
        echo "Use rviz 2D Pose estimate tools to create a succession of waypoints."
        echo "Are you satisfied with your waypoints? [y/n]"
        read -p "Answer:" choice
        echo ""
        new_waypoints=false
        case "$choice" in 
            y|Y ) new_waypoints=true;;
            n|N ) new_waypoints=false;;
            * ) new_waypoints=false;;
        esac

        if [ "$new_waypoints" = true ] ; then

            echo "New run saved. Waiting for user input to start run."
            rostopic pub /path_ready std_msgs/Empty -1
        else
            echo "Aborting run"
            ./stop_exp.sh
            exit
        fi

    else

        echo "Using saved waypoints: $waypoints.csv"

    fi

    # Wait for user to start experiment
    echo ""
    read -p "When ready to run. Press any key" choice
    echo ""
    tmp=false
    case "$choice" in 
        * ) tmp=false;;
    esac

    # Start waypoints
    rostopic pub /start_journey std_msgs/Empty -1


  
else

    echo ""
    echo "No waypoint given. This tour wont be recorded"
    echo "Use rviz 2D Nav goal or the controller to move the robot."
    echo ""

fi


####################################
# Wait for user to stop experiment #
####################################


# Optional record
if [ "$record_bag" = true ] ; then

    echo " "
    echo " "
    echo -e "\033[1;4;34mRecording rosbag\033[0m"

    # Start rosbag record on orin
    record_command="cd 0-VelodyneMapping && ./record_rosbag.sh"
    ssh_command="ssh -i $HOME/.ssh/id_rsa polus@cpr-tor59-xav02 $record_command"

    NOHUP_RECORD_FILE="../../results/nohup_logs/record_rosbag_log.txt"
    nohup $ssh_command > "$NOHUP_RECORD_FILE" 2>&1 &
    echo "OK"
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


