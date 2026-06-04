#!/bin/bash

source /opt/ros/jazzy/setup.bash

colcon build --packages-select radar_msgs --event-handlers console_direct+

colcon build --packages-select radar_muniu --event-handlers console_direct+

colcon build --packages-select radar_subscriber --event-handlers console_direct+
