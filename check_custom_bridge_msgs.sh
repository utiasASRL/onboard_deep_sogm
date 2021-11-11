#!/bin/bash

# Verify the custom types were recognized by the bridge, by printing all pairs of bridged types. 
# The custom types should be listed:

cd ../bridge_ws

echo "Bridged VoxGrid?"

pwd

source "install/setup.bash"

ros2 run ros1_bridge dynamic_bridge --print-pairs