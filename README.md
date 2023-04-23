# Requirements

Package requires ros-melodic, opencv, pcl, eigen and their dependencies. There is a dockerfile for building a docker image for package usage.

## ROS-melodic.

ROS-melodic required.

## OpenCV

OpenCV required.

# Docker support

Package uses nvidia/cuda image.
- Install docekr engine.
- Install CUDA (version 12.01).
- Check this out: https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html. 
Install the nvidia-container-toolkit package (and dependencies) after updating the package listing.

Build image:

`docker build -t ros1_ws <path_to_repo_folder>/docker/dockerfile`

Run image:

`sudo chmod +x <path_to_repo_folder>/docker/run_image.sh`

`sh <path_to_repo_folder>/docker/run_image.sh`

# Package

## Nodes

### image_parser_node

#### Usage

#### Subscribes

#### Publishes

#### Outputs