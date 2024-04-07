cmake_minimum_required(VERSION 3.8)
project(ros2_introduction)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(image_functions_sdfr REQUIRED)
find_package(brightness_msgs REQUIRED)

include_directories(include)

add_executable(brightness_detection src/brightness_detection.cpp)
ament_target_dependencies(
  brightness_detection
  rclcpp
  sensor_msgs
  example_interfaces
  image_functions_sdfr
)
add_executable(brightness_detection_better src/brightness_detection_better.cpp)
ament_target_dependencies(
  brightness_detection_better
  rclcpp
  sensor_msgs
  image_functions_sdfr
  brightness_msgs
)

# Install targets
install(TARGETS
  brightness_detection
  brightness_detection_better
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
