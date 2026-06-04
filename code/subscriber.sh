#!/bin/bash
# 1. Load ROS2 jazzy environment
source /opt/ros/jazzy/local_setup.bash

# Load current package's install environment
source ./install/local_setup.bash

# 3. Launch ROS2 node
ros2 run radar_subscriber radar_subscriber