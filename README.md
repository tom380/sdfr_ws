# Software Development for Robotics workspace

This ROS2 workspace contains ros packages made for `Assignment 6` of the Software Development for Robotics course.

## Setup
1. Create a ROS 2 workspace and src folder
   ```bash
   mkdir -p sdfr_ws/src/
   ```
2. Move the packages to the `sdfr_ws/src/` folder
3. Build the package from the workspace folder
   ```bash
   cd sdfr_ws/
   colcon build
   ```
4. Open a new terminal and source the setup
   ```bash
   cd sdfr_ws/
   source install/setup.bash
   ```

## Sub-Assignments

### 1 Getting familiar with ROS2
This assignment is not part of this workspace, as it only contained standard ROS2 tutorials.

### 2 Brightness Detection
For this assignment two nodes were made, `brightness_detection` and `brightness_detection_better`, more information and the execution of these nodes can be found in the `ros2_introduction` package [README.md](src/ros2_introduction/README.md).