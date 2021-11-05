#!/bin/bash

source "/opt/ros/eloquent/setup.bash"

colcon build --symlink-install --packages-skip ros1_bridge
