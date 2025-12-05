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
    ros-humble-robot-localization \
    curl \
    nano \
    gnupg \
    wget \
    python3-smbus2 \
    lsb-release && \
    rm -rf /var/lib/apt/lists/*

# Install colcon
RUN pip install -U colcon-common-extensions

# =============================================================
# Setup slam_toolbox
# =============================================================

# RUN LINE BELOW TO REMOVE debconf ERRORS (MUST RUN BEFORE ANY apt-get CALLS)
RUN echo 'debconf debconf/frontend select Noninteractive' | debconf-set-selections

RUN apt-get update && apt-get upgrade -y && apt-get install -y --no-install-recommends apt-utils

# Clone slam_toolbox
WORKDIR /ros2_ws/src
RUN git clone -b humble https://github.com/SteveMacenski/slam_toolbox.git

# rosdep
WORKDIR /ros2_ws
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && rosdep update && rosdep install -y -r --from-paths src --ignore-src --rosdistro=humble"

# =============================================================
# RPLIDAR C1 SETUP
# =============================================================

WORKDIR /ros2_ws/src
RUN git clone -b ros2 https://github.com/Slamtec/rplidar_ros.git

# =============================================================
# RF2O SETUP
# =============================================================

RUN git clone https://github.com/MAPIRlab/rf2o_laser_odometry.git

# =============================================================
# ANCLE custom package setup
# =============================================================

COPY src/ancle_pkg /ros2_ws/src/ancle_pkg 

# =============================================================
# Final setup
# =============================================================
# Source ROS and build
WORKDIR /ros2_ws
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build --symlink-install"
RUN echo "source /ros2_ws/install/setup.bash" >> ~/.bashrc

# Set up entrypoint
CMD ["/bin/bash"]