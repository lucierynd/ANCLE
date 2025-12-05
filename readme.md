# Autonomous Navigation in Cluttered Littoral Environments

### Docker stuff

**Build the image**

```
docker build . -t ancle_slam_toolbox_ros_humble_image
```

**Allow display**

```
xhost +local:root
```

**Run the container**

```
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