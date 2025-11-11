FROM arm64v8/ros:noetic-perception-focal

# Use noninteractive mode for apt
ENV DEBIAN_FRONTEND=noninteractive

# =============================================================
# Core development and ROS dependencies
# =============================================================
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    git \
    wget \
    curl \
    libeigen3-dev \
    libpcl-dev \
    ros-noetic-rviz \
    ros-noetic-pcl-ros \
    ros-noetic-eigen-conversions \
    libatlas-base-dev \
    libgoogle-glog-dev \
    libsuitesparse-dev \
    libglew-dev \
    python3-pip \
    python3-tk \
    libopencv-dev \
    python3-opencv \
    nano \
    python3-smbus \
    && rm -rf /var/lib/apt/lists/*

# =============================================================
# Python packages
# =============================================================
RUN pip3 install --no-cache-dir matplotlib

# =============================================================
# Install Ceres Solver 2.0.0
# =============================================================
WORKDIR /home
RUN wget https://github.com/ceres-solver/ceres-solver/archive/refs/tags/2.0.0.tar.gz && \
    tar zxf 2.0.0.tar.gz && rm 2.0.0.tar.gz && \
    mkdir ceres-solver-2.0.0/build && cd ceres-solver-2.0.0/build && \
    cmake -DCMAKE_BUILD_TYPE=Release .. && \
    make -j$(nproc) && make install && \
    rm -rf /home/ceres-solver-2.0.0

# =============================================================
# Setup Catkin workspace + livox driver
# =============================================================
RUN mkdir -p /home/catkin_ws/src
WORKDIR /home/catkin_ws/src

# Livox ROS driver (v2.6.0)
RUN wget https://github.com/Livox-SDK/livox_ros_driver/archive/refs/tags/v2.6.0.tar.gz && \
    tar zxf v2.6.0.tar.gz && rm v2.6.0.tar.gz

# Build workspace
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && cd /home/catkin_ws && catkin_make"

# =============================================================
# Setup cyglidar driver
# =============================================================
RUN mkdir -p /home/d2_ws/src
WORKDIR /home/d2_ws/src

RUN git clone -b ROS1 https://github.com/CygLiDAR-ROS/cyglidar_d2.git && \
    /bin/bash -c "source /opt/ros/noetic/setup.bash && cd /home/d2_ws && catkin_make"


# =============================================================
# Load ROS environment automatically
# =============================================================
WORKDIR /home

COPY ./ros_entrypoint.sh /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
