cmake_minimum_required(VERSION 3.8)
project(trakstar)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Generate messages
find_package(rosidl_default_generators REQUIRED)

include_directories(
  include
  /usr/include/usb
)

# Generate message interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TrakstarMsg.msg"
  DEPENDENCIES std_msgs geometry_msgs
)

# Create the trakstar library
add_library(trakstar_lib
  src/PointATC3DG.cpp
)

target_link_libraries(trakstar_lib usb)
ament_target_dependencies(trakstar_lib
  rclcpp
  std_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

# Create the trakstar_driver node
add_executable(trakstar_driver
  src/trakstar_driver.cpp
)

target_link_libraries(trakstar_driver
  trakstar_lib
  usb
)

ament_target_dependencies(trakstar_driver
  rclcpp
  std_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

# Link against generated message interfaces
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(trakstar_driver "${cpp_typesupport_target}")

# Create the test executable
add_executable(PointATC3DG_test
  src/PointATC3DG.cpp
  tools/PointATC3DG_test.cpp
)

target_link_libraries(PointATC3DG_test usb)
ament_target_dependencies(PointATC3DG_test rclcpp)

# Install targets
install(TARGETS
  trakstar_lib
  DESTINATION lib
)

install(TARGETS
  trakstar_driver
  PointATC3DG_test
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
  FILES_MATCHING PATTERN "*.launch.py"
)

ament_export_include_directories(include)
ament_export_libraries(trakstar_lib)
ament_export_dependencies(
  rclcpp
  std_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

# ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(rosidl_default_runtime)

ament_package()
