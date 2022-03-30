#!/bin/bash

# Get to main folder
cd ..

# Source ros/ros2
ROS_1_DISTRO=noetic
source "/opt/ros/$ROS_1_DISTRO/setup.bash"
. "../catkin_ws/install_isolated/setup.bash"
source "/opt/ros/foxy/setup.bash"

. install/setup.bash

# Open terminals or nohup
nohup=false

sogm_command="ros2 launch deep_sogm simu_launch.py"

if [ "$nohup" = true ] ; then

    # Start bridge in background and collider here
    nohup ros2 run ros1_bridge dynamic_bridge > "nohup_bridge.txt" 2>&1 & $sogm_command

    # Start collider in background and bridge here 
    # nohup ros2 run deep_sogm collider > "nohup_sogm.txt" 2>&1 & ros2 run ros1_bridge dynamic_bridge

else

    # Start bridge in another terminal and collider here
    xterm -bg black -fg lightgray -e ros2 run ros1_bridge dynamic_bridge &
    $sogm_command

fi


# if [ "$nohup" = true ] ; then
#     # Before going further wait for collider to be running
#     source "/opt/ros/$ROS_1_DISTRO/setup.bash"
#     . "../../catkin_ws/install_isolated/setup.bash"
#     echo ""
#     echo "Waiting for SOGM predictions ..."
#     until [ -n "$sogm_topic" ] 
#     do 
#         sleep 0.1
#         sogm_topic=$(rostopic list -p | grep "/classified_points")
#     done 
#     until [[ -n "$sogm_msg" ]]
#     do 
#         sleep 0.1
#         sogm_msg=$(rostopic echo -n 1 /classified_points | grep "frame_id")
#     done 

#     echo "OK"
#     echo ""
# fi


































