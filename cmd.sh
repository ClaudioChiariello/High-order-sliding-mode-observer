#!/usr/bin/env bash

# Check if both arguments are provided
if [ "$#" -ne 2 ]; then
    echo "Usage: $0 <linear_x> <angular_z>"
    echo "Example: $0 0.5 0.2"
    exit 1
fi

LINEAR_X=$1
ANGULAR_Z=$2

echo "Publishing cmd_vel: linear.x = $LINEAR_X, angular.z = $ANGULAR_Z"

# Publish once (--once) to /cmd_vel
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: $LINEAR_X, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: $ANGULAR_Z}}"
