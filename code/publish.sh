#!/bin/bash
# 1. Load ROS2 jazzy environment
source /opt/ros/jazzy/local_setup.bash

# 2. Load current package's install environment
source ./install/local_setup.bash

# 2. Load library path
export LD_LIBRARY_PATH=./install/radar_muniu/lib/radar_muniu:$LD_LIBRARY_PATH

# 4. Launch ROS2 node
ros2 launch radar_muniu radar_launch.py
