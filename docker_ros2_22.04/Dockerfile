# =========================================================
# ROS2 Dockerfile for Trakstar (Ubuntu 22.04 + Humble)
# =========================================================
FROM ros:humble-ros-base

ENV DEBIAN_FRONTEND=noninteractive

# ---------------------------------------------------------
# Install system and ROS dependencies
# ---------------------------------------------------------
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    # libusb-1.0-0-dev \
    libusb-0.1-4 \
    libusb-dev \
    udev \
    usbutils \
    iproute2 \
    net-tools \
    python3-colcon-common-extensions \
    python3-rosdep \
    # To get ros2 messages outside the container
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-rosidl-generator-c \
    ros-humble-rosidl-generator-cpp \
    ros-humble-rosidl-generator-py \
    ros-humble-rosidl-typesupport-c \
    ros-humble-rosidl-typesupport-cpp \
    ros-humble-rosidl-default-generators \
    ros-humble-rosidl-default-runtime \
    ros-humble-rviz2 \
    ros-humble-rviz-common \
    ros-humble-rviz-default-plugins \
    && rm -rf /var/lib/apt/lists/*

# ---------------------------------------------------------
# Set environment variables for ROS2
# ---------------------------------------------------------
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV ROS_DOMAIN_ID=0

# ---------------------------------------------------------
# Set the ROS2 workspace
# ---------------------------------------------------------
WORKDIR /trakstar_ws/src

# Copy your ROS2 package
COPY trakstar_ws/src/trakstar /trakstar_ws/src/trakstar

# ---------------------------------------------------------
# Udev rules
# ---------------------------------------------------------
COPY 99-trakstar.rules /etc/udev/rules.d/99-libusb.rules
RUN udevadm control --reload-rules && udevadm trigger || true

# ---------------------------------------------------------
# Build setup and auto-source
# ---------------------------------------------------------
WORKDIR /trakstar_ws
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc
# RUN echo "source /trakstar_ws/install/setup.bash" >> /root/.bashrc

CMD ["/bin/bash"]
