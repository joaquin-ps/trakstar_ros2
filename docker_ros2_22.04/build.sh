#!/bin/bash
set -e

IMAGE_NAME=trakstar_ros2_22.04

docker build -t $IMAGE_NAME -f docker_ros2_22.04/Dockerfile .
