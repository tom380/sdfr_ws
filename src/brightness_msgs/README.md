# Package brightness_msgs

This ROS2 package contains a custom message definition for `Assignment 6.2` of the Software Development for Robotics course. Specifically made for the `ros2_introduction` package, to publish brightness data.

## Usage

1. Add the package to your ROS 2 workspace's src folder (e.g. `sdfr_ws/src`).
2. Run `colcon build --packages-select brightness_msgs` from your workspace folder.
3. To use `brightness_msgs` in other packages, add it as dependency:
   * In `<package>/package.xml` add the following:
     * `<depend>brightness_msgs</depend>`.
   * In `<package>/CMakeLists.txt` add the following:
     * `find_package(brightness_msgs REQUIRED)`
     * In `ament_target_dependencies` for the respective target, so: `ament_target_dependencies(<target> [other dependencies] brightness_msgs)`

4. To make use of the message type in other packages include/import in the code.  
   **Python**
   ```python
   from brightness_msgs import Brightness
   ```
   **C++**
   ```cpp
   #include "brightness_msgs/msg/brightness.hpp"
   ```

## Package Contents

### Messages
`Brightness`: Contains data about average the brightness of an image.
- ` light_level`: Average light level of all the pixels in an image.
- ` status`: A human-readable description of the average brightness level in an image.