#!/bin/bash

# Build and test script for Trakstar ROS2

echo "Building Trakstar ROS2 package..."

cd trakstar_ws

# Clean previous build
echo "Cleaning previous build..."
rm -rf build/ install/ log/

# Build the package
echo "Building..."
colcon build --packages-select trakstar

if [ $? -eq 0 ]; then
    echo "Build successful!"
    
    # Source the workspace
    source install/setup.bash
    
    echo "Build completed. You can now run:"
    echo "  ros2 launch trakstar trakstar_driver.launch.py publish_tf:=true"
    echo ""
    echo "Or test with:"
    echo "  python3 test_trakstar.py"
    
else
    echo "Build failed!"
    exit 1
fi
