cmake_minimum_required(VERSION 3.8)
project(pick_place)

# Use modern C++ standard
set(CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(ros_gz_interfaces REQUIRED)
find_package(control_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)

# C++ executable for pick and place
# Name the node executable 'pick_place_cpp_node' to match the launch file
add_executable(pick_place_cpp_node src/pick_place_node.cpp)

ament_target_dependencies(pick_place_cpp_node
  rclcpp
  rclcpp_action
  moveit_ros_planning_interface
  moveit_msgs
  geometry_msgs
  sensor_msgs
  tf2_geometry_msgs
  shape_msgs
  ros_gz_interfaces
  control_msgs
  trajectory_msgs
)

# Ensure target has access to local headers
target_include_directories(pick_place_cpp_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# Install executables
install(TARGETS
  pick_place_cpp_node
  DESTINATION lib/${PROJECT_NAME}
)

# No package-installed helper scripts; scripts are located at repository root.

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)


# Install URDF, config, models, worlds
install(DIRECTORY
  config
  worlds
  urdf
  DESTINATION share/${PROJECT_NAME}
)

# Install header files
install(DIRECTORY include/
  DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()