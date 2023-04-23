# Docker support

- Install docekr engine.
- Install CUDA (version 12.01).
- Check this out: https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html. 
Install the nvidia-container-toolkit package (and dependencies) after updating the package listing:

`sudo apt-get update`

`sudo apt-get install -y nvidia-container-toolkit`

`sudo nvidia-ctk runtime configure --runtime=docker`

`sudo systemctl restart docker`