#!/usr/bin/env bash
# Run full pick and place simulation on ROS2 (Humble). Assumes ROS2 and workspace built.
set -e

source /opt/ros/humble/setup.bash

ros2 launch pick_place pick_place_ignition.launch.py
