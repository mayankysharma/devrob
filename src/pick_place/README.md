# Pick and Place

ROS 2 package for automated pick and place operations using a UR5 manipulator in Gazebo Ignition simulation. The package integrates MoveIt for motion planning and ROS 2 Control for robot control.

## Overview

This package provides a complete simulation environment for pick and place tasks with:
- UR5 6-DOF manipulator
- MoveIt motion planning with OMPL
- Gazebo Ignition physics simulation
- ROS 2 Control integration
- Automated pick and place sequence execution

## Prerequisites

- ROS 2 Humble
- MoveIt 2
- Gazebo Ignition (Gazebo Sim)
- UR robot description package

Install dependencies:

```bash
sudo apt install ros-humble-desktop \
  ros-humble-moveit \
  ros-humble-ros-gz-sim \
  ros-humble-ros-gz-bridge \
  ros-humble-ros-gz-interfaces \
  ros-humble-ur-description \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-moveit-configs-utils
```

## Building

```bash
cd /path/to/workspace
source /opt/ros/humble/setup.bash
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select pick_place
source install/setup.bash
```

## Usage

Launch the complete simulation with pick and place execution:

```bash
source install/setup.bash
ros2 launch pick_place pick_place_ignition.launch.py
```

The launch file starts Gazebo Ignition, spawns the UR5 robot, initializes MoveIt, and automatically executes the pick and place sequence.

### Launch Arguments

- `pick_position` (default: `[0.6, 0.0, 0.025]`): Target pick position [x, y, z] in meters
- `place_position` (default: `[-0.5, 0.0, 0.025]`): Target place position [x, y, z] in meters
- `x`, `y`, `z` (default: `0.0`): Robot spawn position
- `use_sim_time` (default: `true`): Enable simulation time

Example with custom positions:

```bash
ros2 launch pick_place pick_place_ignition.launch.py \
  pick_position:="[0.7, 0.1, 0.025]" \
  place_position:="[-0.6, 0.1, 0.025]"
```

## Pick and Place Sequence

The automated sequence executes the following steps using a two-phase approach pattern:

**Pick Phase:**
1. Move to pick approach pose (15 cm above pick position)
2. Move to pick pose (2 cm above object)
3. Attach object to end effector via fixed joint
4. Lift object vertically (20 cm clearance)

**Place Phase:**
5. Move to place approach pose (15 cm above place position)
6. Move to place pose
7. Detach object
8. Retract to safe position

The robot follows a two-step motion pattern: first moving to an approach pose for safe positioning, then executing the final motion to the target pose. This applies to both picking (approach → pick) and placing (approach → place) operations.

## Package Structure

```
pick_place/
├── CMakeLists.txt
├── package.xml
├── config/              # MoveIt and controller configurations
├── launch/              # Launch files
├── src/                 # C++ source code
├── urdf/                # Robot description files
└── worlds/              # Gazebo world files
```

## Configuration

- Robot description: `urdf/ur5_vacuum.urdf.xacro`
- MoveIt SRDF: `config/ur5_manipulator.srdf`
- Controller config: `config/ur5_controllers.yaml`
- World file: `worlds/pick_place_world.sdf`

## Known Limitations

**Gripper Service**: The gripper service is not configured due to issues encountered with attaching objects to the end effector. Object attachment is currently handled via fixed joints created through Gazebo services, which may have reliability limitations. A proper gripper service implementation is planned for future development.

## Troubleshooting

**MoveIt planning fails**: Verify robot is spawned correctly and target poses are within workspace limits.

**Controller errors**: Check controller configuration in `config/ur5_controllers.yaml` and ensure joint state broadcaster is running.



## License

MIT
