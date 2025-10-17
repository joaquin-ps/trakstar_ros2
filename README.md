# Trakstar ROS2

ROS2 wrapper for trakstar driver based on: 

- https://github.com/ChristophJud/ATC3DGTracker 
- https://github.com/seanyun/trakstar_ros

### Docker:

Install Docker following the [official instructions](https://docs.docker.com/engine/install/ubuntu/) and [Linux post-install](https://docs.docker.com/engine/install/linux-postinstall/).

## Runnning with docker:

From project root directory, build container: 
```
./docker_ros2_22.04/build.sh
```

Start container with usb priviledges: 
```
./docker_ros2_22.04/start.sh
```

Build trakstar code: 
```
cd /trakstar_ws
colcon build
source install/setup.bash
```

Test trakstar: 
```
./build/trakstar/PointATC3DG_test  
```

Launch node:
```
ros2 launch trakstar trakstar_driver.launch.py publish_tf:=true
```

## To Do:

- Cleanup ROS 2 wrapper, verify it is working properly. 