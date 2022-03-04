#!/bin/bash
echo ""
echo ""
echo ""
echo "    /---------------------------\\"
echo "   <  Mapping a new environment  >"
echo "    \\---------------------------/"
echo ""
echo ""


# First of all start pointslam on Xavier board
point_slam_command="cd catkin_ws/scripts/ && ./point_slam.sh -m"
ssh_command="ssh -i $HOME/.ssh/id_rsa administrator@cpr-tor59-xav01 $point_slam_command"
echo ""
echo "Running PointSlam via ssh. Command used:"
echo "$ssh_command"

xterm -bg black -fg lightgray -xrm "xterm*allowTitleOps: false" -T "PointSlam" -n "PointSlam" -hold \
    -e $ssh_command &
        
echo "OK"
echo ""
echo "------------------------------------------"
echo ""

# Source ros here
echo ""
echo "Sourcing ROS Noetic"
ROS_1_DISTRO=noetic
source "/opt/ros/$ROS_1_DISTRO/setup.bash"
. "../../catkin_ws/install_isolated/setup.bash"
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
echo ""
echo "Running move_base. Command used:"
move_base_command="roslaunch jackal_navigation teb_normal.launch"
echo "$move_base_command"

xterm -bg black -fg lightgray -xrm "xterm*allowTitleOps: false" -T "Move Base" -n "Move Base" -hold \
    -e $move_base_command &

echo "OK"
echo ""
echo "------------------------------------------"
echo ""


echo ""
read -p "When ready to record. Press any key" choice
echo ""
tmp=false
case "$choice" in 
    * ) tmp=false;;
esac

echo "OK"
echo ""
echo "------------------------------------------"
echo ""


# Record run
echo "Record Rosbag"
nohup ./rosbag_record.sh > "nohup_record.txt" 2>&1 &

echo ""
read -p "Experiment running. Press any key to stop everything" choice
echo ""
tmp=false
case "$choice" in 
    * ) tmp=false;;
esac

# Stop everything
./stop_exp.sh







