# Trakstar ROS2

ROS2 wrapper for Trakstar based on: 

- https://github.com/ChristophJud/ATC3DGTracker 
- https://github.com/seanyun/trakstar_ros

## Quick Start

Install Docker following the [official instructions](https://docs.docker.com/engine/install/ubuntu/) and [Linux post-install](https://docs.docker.com/engine/install/linux-postinstall/).

Build and run with Docker:

```bash
# Build container
./docker_ros2_22.04/build.sh

# Start container
./docker_ros2_22.04/start.sh

# Build and launch
cd /trakstar_ws
colcon build
source install/setup.bash
ros2 launch trakstar trakstar_driver.launch.py publish_tf:=true
```

## Data Access

### Published Topics

- `/trakstar/transforms` - Processed transform data
- `/trakstar/transforms_raw` - Raw transform data  
- `/tf` - TF2 transforms for each tracker

### Message Format

```msg
std_msgs/Header header
geometry_msgs/Transform[4] transform
uint8 n_tracker
```

### Coordinate Frames

- `trakstar_base` - Parent frame
- `trakstar0`, `trakstar1`, etc. - Individual tracker frames

### Example Usage

```bash
# Test hardware connection
./build/trakstar/PointATC3DG_test

# View transform data
ros2 topic echo /trakstar/transforms

# View TF tree
ros2 run tf2_tools view_frames

# Launch RViz for visualization
xhost +local:docker
./launch_trakstar_rviz.sh
```