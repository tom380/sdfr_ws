# Package ros2_introduction

This ROS2 package contains nodes made for `Assignment 6.2` of the Software Development for Robotics course. Which look at an image topic and determines the average brightness, which get published on a topic again.

## Usage

1. Add the package to your ROS 2 workspace's src folder (e.g. `sdfr_ws/src`).
2. Build the package from the workspace folder
   ```bash
   cd ~/sdfr_ws
   colcon build --packages-select ros2_introduction
   ```
3. Open a new terminal and source the setup
   ```bash
   cd ~/sdfr_ws
   source install/setup.bash
   ```
4. Run the nodes from this package as described in the nodes section

## Dependencies

- `rclcpp`
- `sensor_msgs`
- `brightness_msgs`

## Package Contents

### Nodes

`brightness_detection`: Calculates the average pixel brightness of an image.  

- **Subscribes to**
  - `/image` of type [`sensor_msgs/msg/Image`](http://docs.ros.org/en/api/sensor_msgs/html/msg/Image.html)

- **Publishes to**  
  - `/light_level` of type [`example_interfaces/msg/UInt8`](https://github.com/ros2/example_interfaces/blob/humble/msg/UInt8.msg)  
  - `/brightness_status` of type [`example_interfaces/msg/String`](https://github.com/ros2/example_interfaces/blob/humble/msg/String.msg)  

- **Execution**
  ```bash
  ros2 run ros2_introduction brightness_detection
  ```
&nbsp;

`brightness_detection_better`: Calculates the average pixel brightness of an image.

- **Subscribes to**
  - `/image` of type [`sensor_msgs/msg/Image`](http://docs.ros.org/en/api/sensor_msgs/html/msg/Image.html)

- **Publishes to**  
  - `/brightness` of type [`brightness_msgs/msg/Brightness`](../brightness_msgs/msg/Brightness.msg)

- **Execution**
  ```bash
  ros2 run ros2_introduction brightness_detection_better
  ```