# =========================================================
# ROS2 Dockerfile for Trakstar (Ubuntu 22.04 + Humble)
# =========================================================
FROM ros:humble-ros-base

ENV DEBIAN_FRONTEND=noninteractive

# Install dependencies
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    # libusb-1.0-0-dev \
    libusb-0.1-4 \
    libusb-dev \
    udev \
    usbutils \
    && rm -rf /var/lib/apt/lists/*

# Set the ROS2 workspace
WORKDIR /trakstar_ws/src

# Copy only the ROS2 workspace folder
COPY trakstar_ws/src/trakstar /trakstar_ws/src/trakstar

# Copy udev rules
COPY 99-trakstar.rules /etc/udev/rules.d/99-libusb.rules

# Reload udev rules inside container
RUN udevadm control --reload-rules && udevadm trigger || true

# Build the ROS2 workspace
WORKDIR /trakstar_ws
RUN . /opt/ros/humble/setup.sh 

# Auto-source ROS2 and workspace on container start
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc 

WORKDIR /trakstar_ws
CMD ["/bin/bash"]
