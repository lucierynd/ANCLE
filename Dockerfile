# Use the official ROS Humble base image for ARM64
FROM ros:humble-ros-base-jammy

# Set the working directory
WORKDIR /ros2_ws

# Install additional dependencies
RUN apt-get update && apt-get install -y \
    build-essential \
    python3-pip \
    ros-humble-desktop \
    ros-humble-rviz2 \
    ros-humble-pcl-conversions \
    curl \
    gnupg \
    lsb-release && \
    rm -rf /var/lib/apt/lists/*

# Install colcon
RUN pip install -U colcon-common-extensions

# Create and build the workspace
WORKDIR /d2_ws
RUN mkdir -p src && \
    cd src && \
    git clone https://github.com/CygLiDAR-ROS/cyglidar_d2.git

# Source ROS and build
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build --symlink-install"

# Set up entrypoint
CMD ["/bin/bash"]
