# Pick and Place ROS2 Package

A ROS2 Humble package for pick and place simulation using UR5 robot in Gazebo Ignition.

## Features

- UR5 robot (no gripper - using wrist_3_link as end effector)
- MoveIt integration for motion planning
- Gazebo Ignition simulation
- C++ and Python implementations
- Collision-free trajectory planning
- Parameterized pick/place positions

## Prerequisites

### Required ROS2 Packages

```bash
# Core ROS 2
sudo apt install ros-humble-desktop

# MoveIt 2
sudo apt install ros-humble-moveit

# Gazebo Ignition (Gazebo Sim)
sudo apt install ros-humble-ros-gz-sim ros-humble-ros-gz-bridge ros-humble-ros-gz-interfaces

# Robot descriptions
sudo apt install ros-humble-ur-description

# ROS 2 Control
sudo apt install ros-humble-ros2-control ros-humble-ros2-controllers

# Additional dependencies
sudo apt install ros-humble-xacro ros-humble-robot-state-publisher ros-humble-moveit-configs-utils
```

## Building the Package

1. **Navigate to workspace:**
   ```bash
   cd /home/mayank/devrob
   ```

2. **Source ROS 2:**
   ```bash
   source /opt/ros/humble/setup.bash
   ```

3. **Install dependencies:**
   ```bash
   rosdep update
   rosdep install --from-paths src --ignore-src -r -y
   ```

4. **Build the package:**
   ```bash
   colcon build --packages-select pick_place
   ```

5. **Source the workspace:**
   ```bash
   source install/setup.bash
   ```

## Running the Simulation

### Launch Gazebo Ignition with UR5 and MoveIt

```bash
cd /home/mayank/devrob
source install/setup.bash
ros2 launch pick_place spawn_ur5_ignition.launch.py
```

### Run Pick and Place (C++)

In a new terminal:
```bash
cd /home/mayank/devrob
source install/setup.bash
ros2 run pick_place pick_place_cpp
```

### Run Pick and Place (Python)

In a new terminal:
```bash
cd /home/mayank/devrob
source install/setup.bash
ros2 run pick_place pick_place_py.py
```

## Parameters

The pick and place nodes support the following parameters:

- `pick_position` (default: `[0.5, 0.0, 0.85]`): X, Y, Z position for picking
- `place_position` (default: `[-0.5, 0.0, 0.85]`): X, Y, Z position for placing
- `object_name` (default: `"pick_object"`): Name of the object to pick
- `planning_group` (default: `"ur5_manipulator"`): MoveIt planning group name

### Example with custom parameters:

```bash
ros2 run pick_place pick_place_cpp --ros-args \
  -p pick_position:="[0.6, 0.1, 0.85]" \
  -p place_position:="[-0.6, 0.1, 0.85]"
```

## Package Structure

```
pick_place/
├── CMakeLists.txt          # Build configuration
├── package.xml             # Package manifest
├── README.md              # This file
├── config/                # Configuration files
│   ├── initial_positions.yaml
│   ├── kinematics.yaml
│   ├── moveit_controllers.yaml
│   ├── ompl_planning.yaml
│   └── ur.srdf
├── launch/                # Launch files
│   └── spawn_ur5_ignition.launch.py
├── scripts/               # Python scripts
│   └── pick_place_py.py
├── src/                   # C++ source files
│   └── pick_place_cpp.cpp
├── urdf/                  # Robot description
│   ├── ur5_vacuum.urdf.xacro
│   └── vacuum_gripper.xacro
└── worlds/                # Gazebo world files
    └── pick_place_world.sdf
```

## Pick and Place Sequence

The pick and place sequence follows these steps:

1. **Move to pick approach pose** - Move 15cm above the pick position
2. **Move to pick pose** - Move to the pick position (2cm above object)
3. **Attach object** - Create a fixed joint between wrist_3_link and object
4. **Lift object** - Move 20cm up while holding the object
5. **Move to place approach pose** - Move 15cm above the place position
6. **Move to place pose** - Move to the place position
7. **Detach object** - Remove the joint to release the object
8. **Move away** - Move 30cm up to clear the area

## Troubleshooting

### MoveIt planning fails
- Check that the robot is properly spawned in Gazebo
- Verify that the planning group name matches the SRDF configuration
- Ensure the target poses are within the robot's workspace

### Object attachment fails
- Verify the object name matches the model name in the world file
- Check that the wrist_3_link name is correct
- Ensure the Ignition Gazebo services are available

### Controller issues
- Make sure all controllers are properly loaded
- Check the controller configuration in `config/ur5_controllers.yaml`
- Verify the robot description is correctly published

## Notes

- The robot is positioned on the ground (z=0) by default
- No gripper - objects are attached directly to wrist_3_link via fixed joints
- Collision avoidance is handled by MoveIt's planning scene
- The Python implementation is simplified - use C++ for full functionality

## License

MIT

