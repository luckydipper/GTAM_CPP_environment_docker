#!/bin/bash

xhost +local:root
docker run -it --rm \
    --env="DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --volume="./GTSAM_tutorial:/root/GTSAM_tutorial" \
    gtsam_tutorial:0.6
xhost -local:root
