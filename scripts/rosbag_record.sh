#!/bin/bash

ROS_1_DISTRO=noetic
source "/opt/ros/$ROS_1_DISTRO/setup.bash"
. "../../catkin_ws/install_isolated/setup.bash"

# Current Date
now=`date +%Y-%m-%d_%H-%M-%S`
rosbag_path="$HOME/results/rosbag_data/$now.bag"

rosbag record -O $rosbag_path /clock \
                            /velodyne_points \
                            /map \
                            /tf \
                            /tf_static \
                            /move_base/global_costmap/costmap \
                            /move_base/local_costmap/costmap \
                            /move_base/result \
                            /move_base/current_goal \
                            /optimal_path \
                            /classified_points  \
                            /static_visu  \
                            /dynamic_visu  \
                            /plan_costmap_3D \
                            /move_base/TebLocalPlannerROS/local_plan \
                            /move_base/TebLocalPlannerROS/global_plan \
                            /move_base/TebLocalPlannerROS/teb_markers \
                            /move_base/TebLocalPlannerROS/teb_poses \
                            /move_base/TebLocalPlannerROS/obstacles \
                            /move_base/TebLocalPlannerROS/teb_markers




