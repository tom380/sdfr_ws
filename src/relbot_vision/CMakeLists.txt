cmake_minimum_required(VERSION 3.8)
project(relbot_vision)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(image_functions_sdfr REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
"msg/BoundingBox.msg"
"msg/BallDetection.msg"
)

include_directories(include)

file(GLOB ball_detector_functions
  "src/ball_detector_functions/*.cpp")

add_executable(ball_detector
  src/ball_detector.cpp
  ${ball_detector_functions})
ament_target_dependencies(
  ball_detector
  rclcpp
  sensor_msgs
  cv_bridge
  OpenCV
  image_functions_sdfr
)

install(TARGETS
  ball_detector
  DESTINATION lib/${PROJECT_NAME}
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(ball_detector "${cpp_typesupport_target}")

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
