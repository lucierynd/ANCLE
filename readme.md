# Autonomous Navigation in Cluttered Littoral Environments

# Structure:

```bash
┌─────────────┐     ┌──────────────────────┐     ┌─────────────────┐
│  PLidar C1  │────▶│  rf2o_laser_odometry │────▶│                 │
│   /scan     │     │  (laser odom)        │     │  robot_locali-  │     ┌───────────────┐
└─────────────┘     └──────────────────────┘     │  sation (EKF)   │────▶│ SLAM Toolbox  │
                                                 │                 │     │  /odom        │
┌─────────────┐                                  │  Fused /odom    │     └───────────────┘
│    IMU      │─────────────────────────────────▶│                 │
│   /imu      │                                  └─────────────────┘
└─────────────┘
```

```bash
┌─────────────┐     ┌──────────────────────┐     ┌─────────────────┐
│  PLidar C1  │────▶│  rf2o_laser_odometry │────▶│  SLAM Toolbox   │
│   /scan     │     │  (laser odom)        │     │  /odom          │
└─────────────┘     └──────────────────────┘     └─────────────────┘
```

---

# Instructions:

## Docker stuff:

Build docker image:

```bash
docker build -f tools/Docker/dockerfile_ros_humble_slam_toolbox/Dockerfile -t ancle_slam_toolbox_humble_image .
```

Starting container:

```bash
docker run -it --rm\
  --net=host \
  --env="DISPLAY=$DISPLAY" \
  --env="QT_X11_NO_MITSHM=1" \
  --env="XAUTHORITY=$XAUTH" \
  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  --runtime nvidia \
  --gpus all \
  --privileged \
  --device=/dev/ttyUSB0:/dev/ttyUSB0 \
  --device /dev/i2c-7 \
  -v /dev:/dev \
  -v /home/robotuna/ANCLE:/home/ANCLE \
  --name ancle_slam_toolbox_humble_container \
  ancle_slam_toolbox_humble_image \
  bash
```

Container in new terminal window:

```bash
docker exec -it ancle_slam_toolbox_humble_container bash
```

## Run lidar inertial slam

Basic launch:

```bash
ros2 launch ancle_pkg lidar_inertial_slam_launch.py
```

Or without rviz:

```bash
ros2 launch ancle_pkg lidar_inertial_slam_launch.py use_rviz:=false
```

## Run lidar only slam

Basic launch:

```bash
ros2 launch ancle_pkg lidar_only_slam_launch.py 
```

Or without rviz:

```bash
ros2 launch ancle_pkg lidar_only_slam_launch.py use_rviz:=false
```