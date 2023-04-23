 #!/bin/bash

docker run -it --rm \
    -v /home/ildar/ros1_ws:/ros1_ws \
    --runtime=nvidia --gpus all \
    ros1_ws:latest