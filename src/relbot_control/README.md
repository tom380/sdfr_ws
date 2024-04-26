# Package relbot_vision

This ROS2 package contains nodes made for `Assignment 6.4` of the Software Development for Robotics course. Which was made to control RELbot.

## Usage

1. Add the package to your ROS 2 workspace's src folder (e.g. `sdfr_ws/src`).
2. Build the package from the workspace folder
   ```bash
   cd ~/sdfr_ws
   colcon build --packages-select relbot_control
   ```
3. Open a new terminal and source the setup
   ```bash
   cd ~/sdfr_ws
   source install/setup.bash
   ```
4. Run the nodes from this package as described in the nodes section

## Dependencies

- `rclcpp`
- `geometry_msgs`
- `sensor_msgs`
- `relbot_vision`
- `image_functions_sdfr`

## Package Contents

### Nodes

`velocity_controller`: Finds a ball in the given image, either with the HoughCircles function or hue and blob detection.

- **Subscribes to**
  - `/ball_detector/detection` of type [`relbot_vision/msg/BallDetection`](../relbot_vision/msg/BallDetection.msg)
  - `/ball_detector/debug_image` of type [`sensor_msgs/msg/Image`](http://docs.ros.org/en/api/sensor_msgs/html/msg/Image.html)

- **Publishes to**  
  - `/velocity_controller/velocity` of type [`geometry_msgs/msg/TwistStamped`](https://docs.ros2.org/latest/api/geometry_msgs/msg/TwistStamped.html)  
  - `/velocity_controller/debug_image` of type [`sensor_msgs/msg/Image`](http://docs.ros.org/en/api/sensor_msgs/html/msg/Image.html)

- **Parameters**
  - `detection_topic`: Topic that the node subscribes to for detection data of the ball.  
  Defaults to `/ball_detector/detection`
  - `image_topic`: Topic that the node subscribes to for image data to generate a debug image.  
  Defaults to `/ball_detector/debug_image`
  - `debug`: Whether to publish a debug image or not. Should either be `true` or `false`  
  Defaults to `false`
  - `target_size`: Target size of the ball.    
  Defaults to `200`
  - `target_position`: Target position of the ball, along the x axis.    
  Defaults to `320`
  - `linear_gain`: The proportional gain from the size error to the linear velocity. In $m\cdot s^{-1}$ per pixel.   
  Defaults to `0.01`
  - `angular_gain`: The proportional gain from the position error to the angular velocity. In $rad\cdot s^{-1}$ per pixel.   
  Defaults to `1`

- **Execution**
  ```bash
  ros2 run relbot_control velocity_controller --ros-args -p detection_topic:=<detection_topic> -p image_topic:=<image_topic> -p debug:=<debug> -p target_size:=<target_size> target_position:=<target_position> linear_gain:=<linear_gain> angular_gain:=<angular_gain>
  ```  
&nbsp;

`differential_controller`: Finds a ball in the given image, either with the HoughCircles function or hue and blob detection.

- **Subscribes to**
  - `/velocity_controller/velocity` of type [`geometry_msgs/msg/TwistStamped`](https://docs.ros2.org/latest/api/geometry_msgs/msg/TwistStamped.html)  

- **Publishes to**  
  - `/input/left_motor/setpoint_vel` of type [`example_interfaces/msg/Float64`](https://github.com/ros2/example_interfaces/blob/humble/msg/Float64.msg)  
  - `/input/right_motor/setpoint_vel` of type [`example_interfaces/msg/Float64`](https://github.com/ros2/example_interfaces/blob/humble/msg/Float64.msg)  

- **Parameters**
  - `intra_wheel_width`: The distance between the two wheels of the RELbot. In meters.  
  Defaults to `0.209`
  - `wheel_radius`: The radius of two wheels of the RELbot. In meters.  
  Defaults to `0.0505`

- **Execution**
  ```bash
  ros2 run relbot_control differential_controller --ros-args -p intra_wheel_width:=<intra_wheel_width> -p wheel_radius:=<wheel_radius> 
  ```