#!/bin/bash

ROS_1_DISTRO=noetic
source "/opt/ros/$ROS_1_DISTRO/setup.bash"
. "../../catkin_ws/devel/setup.bash"

# Current Date
now=`date +%Y-%m-%d_%H-%M-%S`
rosbag_path="$HOME/results/rosbag_data/$now.bag"

echo ""
echo ""
echo "Recording to: $rosbag_path"
echo ""
echo ""

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
                            /dummy_plan_costmap_3D \
                            /dummy_obstacles \
                            /move_base/TebLocalPlannerROS/local_plan \
                            /move_base/TebLocalPlannerROS/global_plan \
                            /move_base/TebLocalPlannerROS/teb_markers \
                            /move_base/TebLocalPlannerROS/teb_poses \
                            /move_base/TebLocalPlannerROS/obstacles \
                            /move_base/TebLocalPlannerROS/teb_pose_layer \
                            /move_base/TebLocalPlannerROS/sogm_delay \
			    /Debug/BevObjects \
/Debug/Occupancy \
/Debug/PerspObjects \
/Object/Detections3D \
/Object/RawDetections3D \
/Object/RawDetections3D_3 \
/Object/RawDetections3D_4 \
/Object/RawDetections3D_occupancy \
/Object/RawDetections3D_secondary \
/camera/rgb/image_raw \
/cmd_vel \
/move_base_simple/goal \
/debug/objects \
/jackal_velocity_controller/odom
                            




