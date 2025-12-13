#!/usr/bin/env bash
# Build and run pick_place inside a Docker container. Requires Docker installed.
set -e
IMAGE_NAME=pick_place:humble

docker build -t ${IMAGE_NAME} ./docker

# Run container with GUI support (X11) - this is host-specific
# Allow root to access X11 (safer: limit to local user)
xhost +si:localuser:root

# Run the container and execute build+launch inside it. The container will fail if no host X11 is available.
docker run --rm -it \
  --env="DISPLAY" \
  --env="XAUTHORITY=$XAUTHORITY" \
  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  --volume="$XAUTHORITY:$XAUTHORITY" \
  --device /dev/dri \
  --group-add video \
  --volume="$(pwd):/workspace/src" \
  --network host \
  ${IMAGE_NAME} /bin/bash -lc "source /opt/ros/humble/setup.bash && cd /workspace && rm -rf build install log || true && colcon build --symlink-install && source install/setup.bash && ros2 launch pick_place pick_place_ignition.launch.py"

# Revoke access
xhost -si:localuser:root
