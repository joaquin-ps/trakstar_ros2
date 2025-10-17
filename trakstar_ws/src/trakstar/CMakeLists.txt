cmake_minimum_required(VERSION 3.8)
project(trakstar)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

include_directories(
  include
  /usr/include/usb
)

add_executable(PointATC3DG_test
  src/PointATC3DG.cpp
  tools/PointATC3DG_test.cpp
)

# Plain-style linking to libusb-0.1
target_link_libraries(PointATC3DG_test usb)

# Now call ament_target_dependencies (plain style internally)
ament_target_dependencies(PointATC3DG_test rclcpp)

install(TARGETS
  PointATC3DG_test
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()
