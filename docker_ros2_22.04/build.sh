#!/bin/bash
set -e

IMAGE_NAME=trakstar_ros2_22.04

# Check if user is in docker group
if ! groups $USER | grep -q docker; then
    echo "Warning: User $USER is not in the docker group."
    echo "To fix this, run:"
    echo "  sudo usermod -aG docker $USER"
    echo "  newgrp docker"
    echo "Then log out and back in, or restart your terminal."
    echo ""
    echo "Continuing with build (may require sudo)..."
fi

docker build -t $IMAGE_NAME -f docker_ros2_22.04/Dockerfile .
