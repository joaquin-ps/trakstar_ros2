#!/bin/bash
set -e

IMAGE_NAME=trakstar_ros2_22.04
CONTAINER_NAME=trakstar_ros2_container

# Run interactively with USB passthrough (if you want to test with real device)
docker run -it --rm \
    --net=host \
    --privileged \
    --device=/dev/bus/usb:/dev/bus/usb \
    -v $(pwd)/trakstar_ws:/trakstar_ws \
    --name $CONTAINER_NAME \
    $IMAGE_NAME