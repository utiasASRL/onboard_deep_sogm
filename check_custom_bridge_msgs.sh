#!/bin/bash

# Verify the custom types were recognized by the bridge, by printing all pairs of bridged types. 
# The custom types should be listed:

#cd ../bridge_ws

source "install/setup.bash"

echo ""
echo "Bridged tf?"
ros2 run ros1_bridge dynamic_bridge --print-pairs | grep "tf"

echo ""
echo "Bridged VoxGrid?"
ros2 run ros1_bridge dynamic_bridge --print-pairs | grep "Vox"

echo ""
echo "Bridged PointCloud?"
ros2 run ros1_bridge dynamic_bridge --print-pairs | grep "PointCloud"

echo ""
echo "Bridged ObstacleArrayMsg?"
ros2 run ros1_bridge dynamic_bridge --print-pairs | grep "ObstacleArrayMsg"

echo ""