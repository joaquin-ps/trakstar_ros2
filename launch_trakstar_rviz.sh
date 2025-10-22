#!/bin/bash

# Launch Trakstar with RViz visualization
# This script starts the Trakstar driver and RViz for visualization

echo "Starting Trakstar ROS2 driver with RViz visualization..."

# Check if user is in docker group
if ! groups $USER | grep -q docker; then
    echo "Warning: User $USER is not in the docker group."
    echo "To fix this, run:"
    echo "  sudo usermod -aG docker $USER"
    echo "  newgrp docker"
    echo "Then log out and back in, or restart your terminal."
    echo ""
    echo "Continuing with launch (may require sudo)..."
fi

# Allow X11 connections from Docker containers
echo "Setting up X11 forwarding..."
xhost +local:docker

# Build the workspace first
echo "Building Trakstar workspace..."
docker run --rm -v $(pwd)/trakstar_ws:/trakstar_ws \
    trakstar_ros2_22.04 \
    bash -c "cd /trakstar_ws && source /opt/ros/humble/setup.bash && colcon build"

# Start the Trakstar driver in the background
echo "Launching Trakstar driver..."
docker run -d --rm --net=host --privileged --device=/dev/bus/usb:/dev/bus/usb \
    -v $(pwd)/trakstar_ws:/trakstar_ws \
    --name trakstar_driver_container \
    trakstar_ros2_22.04 \
    bash -c "cd /trakstar_ws && source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 launch trakstar trakstar_driver.launch.py publish_tf:=true"

# Wait a moment for the driver to start
sleep 3

# Start RViz with a pre-configured configuration
echo "Launching RViz..."
docker run -it --rm --net=host \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -v $(pwd)/trakstar_ws:/trakstar_ws \
    --name trakstar_rviz_container \
    trakstar_ros2_22.04 \
    bash -c "cd /trakstar_ws && source /opt/ros/humble/setup.bash && source install/setup.bash && rviz2 -d src/trakstar/rviz/trakstar.rviz"

# Cleanup function
cleanup() {
    echo "Stopping containers..."
    docker stop trakstar_driver_container 2>/dev/null || true
    docker stop trakstar_rviz_container 2>/dev/null || true
}

# Set up cleanup on script exit
trap cleanup EXIT
