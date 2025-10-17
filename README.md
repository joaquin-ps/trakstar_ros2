# Trakstar ROS2

ROS2 wrapper for trakstar driver based on: 

- https://github.com/ChristophJud/ATC3DGTracker 
- https://github.com/seanyun/trakstar_ros


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
```

Test trakstar: 
```
./build/trakstar/PointATC3DG_test  
```

## To Do:

- Get ros2 to find the package (ros2 pkg list).
- Create wrapper to tf publisher. 