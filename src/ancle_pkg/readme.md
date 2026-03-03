# ANCLE package

ANCLE perception pipeline adapted to run on a jetson Orin Nano with the following sensors:
- Cyglidar D2 (3D solid state LiDAR)
- RPLiDAR C1 (2D 360 FOV rotating LiDAR)
- iNEMO inertial module ISM330DHCX (IMU communicating via I2C)


## Docker stuff:

Build the docker container from the ANCLE main folder:

```bash
docker build -f tools/Docker/dockerfile_ros_humble_ancle/Dockerfile -t ancle_humble_image .
```

Starting container:

```bash
docker run -it --rm \
	--net=host \
	--env="DISPLAY=$DISPLAY" \
	--env="QT_X11_NO_MITSHM=1" \
	--env="XAUTHORITY=$XAUTH" \
	--volume="/tmp/.X11-unix:/tmp/. X11-unix:rw" \
	--runtime nvidia \
	--gpus all \
	--device=/dev/cyglidar:/dev/cyglidar \
	--device=/dev/rplidar:/dev/rplidar \
	--device=$(readlink -f /dev/cyglidar) \
	--device=$(readlink -f /dev/rplidar) \
	--device=/dev/i2c-7:/dev/i2c-7 \
	--name ancle_humble_container \
	ancle_humble_image \
	bash
```

Container in new terminal window:

```bash
docker exec -it ancle_humble_container bash
```

**Running lidar inertial 2D slam**

Basic launch:

```bash
ros2 launch ancle_pkg lidar.inertial.slam.launch.py
```

Or without rviz:

```bash
ros2 launch ancle_pkg lidar.inertial.slam.launch.py use_rviz:=false
```

**Running ANCLE pipeline**

Basic launch for odometry only:

```bash
ros2 launch ancle_pkg ancle.launch.py 
```

Launching with mapping part of the pipeline:

```bash
ros2 launch ancle_pkg ancle.launch.py mapping:=true
```

Or without rviz:

```bash
ros2 launch ancle_pkg ancle.launch.py use_rviz:=false
```
