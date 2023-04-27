# Bag parser

Use this package to convert ros-melodic bag files (.bag) into a files (images as .png files, point clouds as .pcl files)

# Requirements

Package requires:
- ros-melodic
- openCV
- eigen
- pcl
There is a dockerfile for building a docker image for package usage.

# Docker

Package uses nvidia/cuda image.
- Install docker engine.
- Install CUDA (version 12.01 tested).

Check this out: https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html. 
Install the nvidia-container-toolkit package (and dependencies) after updating the package listing.

### Build image
You can build a docker image by command below:
```
docker build -t <your_image_name> -f docker/dockerfile .
```

### Run image:
You can start builded docker image by command below:
```
sudo chmod +x <path_to_repo_folder>/docker/run_image.sh
sh <path_to_repo_folder>/docker/run_image.sh
```
# Nodes

## image_parser_node

Saves images (.png format) and camera information (.yaml format) into a filesystem.
- `image_topic` - It's a `sensor_msgs/Image` topic to subscribe.
- `cinfo_topic` - It's a `sensor_msgs/CamraInfo` topic to subscribe.
- `output_path` - Directory to use for output files.

### Usage

Run node:

`rosrun bag_parser image_parser_node image_topic:=<topic> cinfo_topic:=<topic> output_path:=<path>`