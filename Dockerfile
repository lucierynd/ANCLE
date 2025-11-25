FROM ros:humble-perception

# =============================================================
# Core development and ROS dependencies
# =============================================================

RUN apt-get update && apt-get install -y \
    build-essential \
    python3-pip \
    ros-humble-desktop \
    ros-humble-rviz2 \
    ros-humble-pcl-conversions \
    curl \
    nano \
    gnupg \
    python3-smbus2 \
    lsb-release && \
    rm -rf /var/lib/apt/lists/*

# Install colcon
RUN pip install -U colcon-common-extensions

# Fast Lio dependencies
RUN apt-get update && apt-get install -y \
    libpcl-dev \
    libeigen3-dev \
    ros-humble-pcl-ros

# =============================================================
# Setup cyglidar driver
# =============================================================

# Create and build the workspace
WORKDIR /d2_ws
RUN mkdir -p src && \
    cd src && \
    git clone https://github.com/CygLiDAR-ROS/cyglidar_d2.git

RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build --symlink-install"
RUN echo "source /d2_ws/install/setup.bash" >> ~/.bashrc

# =============================================================
# Setup ANCLE package
# =============================================================
WORKDIR /ancle

COPY ./src /ancle/src
COPY ./tools /ancle/tools

# =============================================================
# Setup Fast LIO
# =============================================================
WORKDIR /ros2_ws

# Livox driver
RUN mkdir -p src && \
    cd src && \
    git clone https://github.com/Ericsii/livox_ros_driver2.git && \
    git clone https://github.com/Livox-SDK/Livox-SDK2.git && \
    cd Livox-SDK2 && \
    mkdir build && cd build && \
    cmake .. && make && \
    make install && \
    cd /ros2_ws && \
    /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build --symlink-install"


# Fast LIO
WORKDIR /ros2_ws/src

COPY ./src/FAST_LIO ./FAST_LIO

RUN cd .. && \
    rosdep install --from-paths src --ignore-src -y && \
    /bin/bash -c "source /opt/ros/humble/setup.bash && \
                  colcon build --symlink-install && \
                  source /ros2_ws/install/setup.bash"
    
# =============================================================

# Source ROS and build
WORKDIR /ros2_ws
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build --symlink-install"
RUN echo "source /ros2_ws/install/setup.bash" >> ~/.bashrc

# Set up entrypoint
CMD ["/bin/bash"]
