#!/usr/bin/env bash
set -e
# Build the package and run basic checks (requires ROS2 environment)
source /opt/ros/humble/setup.bash

colcon build --packages-select pick_place --symlink-install
source install/setup.bash

# Lint and docs
ament_lint_auto --only package "pick_place" || true

echo "Build complete. Run: ros2 launch pick_place pick_place_ignition.launch.py"
