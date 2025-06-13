#!/bin/bash

xhost +

DIR="$(cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd)"
docker run -it \
    --runtime=nvidia \
    --gpus all \
    -e NVIDIA_DRIVER_CAPABILITIES=all \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix/:/tmp/.X11-unix/ \
    -v /dev/dri:/dev/dri \
    -v $DIR/../../:/workspace/:rw \
    --name general \
    general:0.1.0
