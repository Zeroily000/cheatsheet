#!/bin/bash

xhost +

DIR="$(cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd)"
docker run -it \
    --gpus all \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix/:/tmp/.X11-unix/ \
    -v $DIR/../../:/workspace/:rw \
    --name boiled \
    dumpling:cuttle
