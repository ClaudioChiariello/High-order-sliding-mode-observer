#!/bin/bash
# Usage: ./pub_cmd.sh <linear_x> <angular_z>

LINEAR_X=${1:-0.0}
ANGULAR_Z=${2:-0.0}

ros2 topic pub -r 1 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: $LINEAR_X, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: $ANGULAR_Z}}"
