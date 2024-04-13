# Package relbot_vision

This ROS2 package contains nodes made for `Assignment 6.3` of the Software Development for Robotics course. Which was made to find a ball in an image.

## Usage

1. Add the package to your ROS 2 workspace's src folder (e.g. `sdfr_ws/src`).
2. Build the package from the workspace folder
   ```bash
   cd ~/sdfr_ws
   colcon build --packages-select relbot_vision
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
- `cv_bridge`
- `OpenCV`

## Package Contents

### Messages
`BallDetection`: Contains data about a detected ball in an image
- `found`: True if a ball has been found
- `bounding_box`: The bounding box surrounding the found ball

`BoundingBox`: Bounding box indicating where a object is in an image
- `centre_x`: The x coordinate of the centre of the bounding box
- `centre_y`: The y coordinate of the centre of the bounding box
- `width`: The width of the bounding box
- `height`: The height of the bounding box

### Nodes

`ball_detector`: Finds a ball in the given image, either with the HoughCircles function or hue and blob detection.

- **Subscribes to**
  - `/image` of type [`sensor_msgs/msg/Image`](http://docs.ros.org/en/api/sensor_msgs/html/msg/Image.html)

- **Publishes to**  
  - `/ball_detector/detection` of type [`relbot_vision/msg/BallDetection`](../relbot_vision/msg/BallDetection.msg)  
  - `/ball_detector/debug_image` of type [`example_interfaces/msg/String`](http://docs.ros.org/en/api/sensor_msgs/html/msg/Image.html)

- **Parameters**
  - `image_topic`: Topic that the node subscribes to for image data of the ball.  
  Defaults to `/image`
  - `method`: Method of detecting the ball. Currently the only methods are `HOUGH_CIRCLES` and `BLOB`.  
  Defaults to `BLOB`
  - `debug`: Whether to publish a debug image or not. Should either be `true` or `false`  
  Defaults to `false`
  - `hue`: Hue of the ball. Should be in the range of `0-180`.  
  Defaults to `60` (green)

- **Execution**
  ```bash
  ros2 run relbot_vision ball_detector --ros-args -p image_topic:=<image_topic> -p method:=<method> -p debug:=<debug> -p hue:=<hue>
  ```