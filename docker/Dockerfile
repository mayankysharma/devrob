FROM osrf/ros:humble-desktop

# Setup locale
RUN apt-get update && apt-get install -y locales && locale-gen en_US.UTF-8
ENV LANG en_US.UTF-8
ENV LANGUAGE en_US:en
ENV LC_ALL en_US.UTF-8

# Install basic build tools and dependencies
RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    python3-pip \
    wget \
    git \
    vim \
    curl \
    gnupg2 \
    lsb-release \
    && rm -rf /var/lib/apt/lists/*

# Install ROS2 packages useful for this package
RUN apt-get update && apt-get install -y \
    ros-humble-moveit \
    ros-humble-moveit-configs-utils \
    ros-humble-ros-gz-bridge \
    ros-humble-ros-gz-sim \
    ros-humble-ros-gz-interfaces \
    ros-humble-ros2-control \
    ros-humble-controller-manager \
    ros-humble-ros2-controllers \
    ros-humble-ros2-control-test-assets \
    ros-humble-ur-description \
    ros-humble-rclpy \
    ros-humble-xacro \
    ros-humble-robot-state-publisher \
    ros-humble-joint-state-publisher \
    ros-humble-control-msgs \
    ros-humble-shape-msgs \
    python3-numpy \
    libopenblas-dev \
    ros-humble-ros2cli \
    ros-humble-ros2param \
    ros-humble-gz-ros2-control \
    ros-humble-gazebo-ros2-control \
    && rm -rf /var/lib/apt/lists/*

# Create a workspace and copy source
WORKDIR /workspace
RUN mkdir -p /workspace/src
COPY . /workspace/src/pick_place

# Build workspace
WORKDIR /workspace
RUN /bin/bash -lc "source /opt/ros/humble/setup.bash && colcon build --symlink-install"

# Default command
CMD ["/bin/bash"]
