# Autonomous Navigation in Cluttered Littoral Environments

## Perception Pipeline

**Multi lidar inertial SLAM**

![ANCLE Perception pipeline](tools/images/ANCLE_pipeline.png)

## Instructions:

### Using the packages

The pipeline can be eiter used within a Gazebo simulation [[AUV Simulation Package](./src/auv_simulation_pkg)] or directly with sensors [[ANCLE Package](./src/ancle_pkg)]. Both package are assiocated with a docker image for a smooth set up, images are specifically built for ARM64 system such as Jetson Nano. Specific intructions on docker and package use can be found within corresponding folders.

### Running rviz on another computer

Another docker image has been created in order to visualize output from the pipeline directly on another computer (using ROS communication over WIFI).

Build the docker container from the ANCLE main folder:

```bash
docker build -f tools/Docker/dockerfile_ros_humble_ext_viz/Dockerfile  -t ancle_ext_viz_image .
```

Run the container: 

```bash
docker run -it --rm\
  --net=host \
  --env="DISPLAY=$DISPLAY" \
  --volume="/tmp/.X11-unix:/tmp/. X11-unix:rw" \
  -v /home/robotuna/ANCLE:/home/ANCLE \
  --name ancle_ext_viz_container \
  ancle_ext_viz_image \
  bash
```

Export domain ID on both computer within containers:

```
export ROS_DOMAIN_ID=12
```