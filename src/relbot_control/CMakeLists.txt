cmake_minimum_required(VERSION 3.8)
project(relbot_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(relbot_vision REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(image_functions_sdfr REQUIRED)

include_directories(include)

add_executable(velocity_controller src/velocity_controller.cpp)
ament_target_dependencies(
  velocity_controller
  rclcpp
  geometry_msgs
  sensor_msgs
  relbot_vision
  cv_bridge
  OpenCV
  image_functions_sdfr
)

add_executable(differential_controller src/differential_controller.cpp)
ament_target_dependencies(
  differential_controller
  rclcpp
  geometry_msgs
  example_interfaces
)

install(TARGETS
  velocity_controller
  differential_controller
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
