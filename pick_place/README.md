# pick_place

Pick and place simulation package for UR5 robot with a vacuum gripper using MoveIt2 and Ignition Gazebo.

## Overview
- ROS2 package using MoveIt2 planning and Ignition Gazebo (ros_gz).
- Provides a simple sequence: move to pick approach, pick (simulated attach), lift, move to place, detach (simulated), move away.
- Simulation world and URDF are included.

## Quickstart (ROS2 Humble)

1. Source ROS2 and workspace:

```bash
# If first time
mkdir -p ~/dev_ws/src
cd ~/dev_ws
colcon build --symlink-install
# Source ROS2
source /opt/ros/humble/setup.bash
# Source workspace (after build)
source install/setup.bash
```

2. Clone this package into `~/dev_ws/src` and build:

```bash
cd ~/dev_ws/src
ln -s /home/mayank/rem/devrob/pick_place ./pick_place
cd ~/dev_ws
colcon build --symlink-install
source install/setup.bash
```

3. Launch simulation and MoveIt2:

```bash
ros2 launch pick_place pick_place_ignition.launch.py
```

4. The `pick_place_cpp_node` will execute the pick-and-place sequence automatically after MoveIt and controllers are ready.

## Docker (Optional)
A `docker` directory at the repository root contains a Dockerfile and helper scripts for building and running the package in a contained environment. Use the root-level scripts in `../scripts/` for launching or building the container.

Build and run the container (X11 GUI forwarding required):

```bash
# From repository root
docker build -t pick_place:humble ./docker
./scripts/run_in_docker.sh
```

<!-- Recording help removed; use your preferred desktop capture tools if needed. -->

## Notes and Known Limitations
- Object attachment/detachment is simulated. To enable real joint creation/removal in Ignition, install model-attachment plugins or implement pose-following controllers.
- MoveIt planning parameters are configurable via launch or parameters.

## File Layout
- `include/pick_place/pick_place_node.hpp`: Header declaring the node class.
- `src/pick_place_node.cpp`: Main node implementation and `main()`.
- `launch/pick_place_ignition.launch.py`: Launch file to start Ignition + MoveIt2.
- `urdf/`, `config/`, `worlds/` - robot description and world assets.

## Contributing
Please open issues or PRs if you'd like to improve the simulation, add a real attachment plugin, or extend the controller support.
