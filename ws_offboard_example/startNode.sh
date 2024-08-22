#!/bin/bash
# This script provided to simplify environment configuration and launching of node
# Simply call ./startNode.sh to source configuration scripts and call ROS2 launch script

echo "Starting Offboard Control Node..."
source /opt/ros/humble/setup.bash
source install/local_setup.bash
ros2 launch px4_offboard offboard_actuator.launch.py


