# Package relbot_vision

This ROS2 package contains nodes made for `Assignment 6.4` of the Software Development for Robotics course. Which was made to provide several launch functionalties for RELbot.

## Usage

1. Add the package to your ROS 2 workspace's src folder (e.g. `sdfr_ws/src`).
2. Build the package from the workspace folder
   ```bash
   cd ~/sdfr_ws
   colcon build --packages-select relbot_launch
   ```
3. Open a new terminal and source the setup
   ```bash
   cd ~/sdfr_ws
   source install/setup.bash
   ```
4. Launch the launch files from this package as described in the launch section

## Dependencies

- `image_tools`
- `relbot_vision`
- `relbot_control`

## Package Contents

### Launch

`velocity_response_test.launch.py`: Finds a ball in the given image, with the `BLOB` method, and shows the desired RELbot velocity. Also shows a debug image of the detected ball and desired position.

- **Arguments**
   - `image_rosbag`: Path to rosbag containing an image topic. When path is specified, the launch file will play the provided file on a loop.

- **Launched nodes**
   - `ball_detector` from package `relbot_vision`  
      **Set parameters**:  
         - `method`: `BLOB`  
         - `hue`: 40.0  
         - `debug`: true   

   - `velocity_controller` from package `relbot_control`  
      **Set parameters**:  
         - `debug`: true

   - `showimage` from package `image_tools`  
      **Remapping**:  
         - `/image` --> `/velocity_controller/debug_image`  
&nbsp;

`relbot_simulation_test.launch.py`: Runs the necessary vision and control packages to control a simulation of the RELbot. It shows an target image which follows and tries to align with the ball.

- **Arguments**
   - `image_rosbag`: Path to rosbag containing an image topic. When path is specified, the launch file will play the provided file on a loop.
   - `linear_gain`: Gain of size error to linear velocity  
   Defaults to `0.005`
   - `angular_gain`: Gain of position error to angular velocity  
   Defaults to `0.005`
   - `hue`: Hue of the ball  
   Defaults to `120.0`
   - `method`: Detection method to detect the ball  
   Defaults to `BLOB`


- **Launched nodes**
   - `relbot_simulator` from package `relbot_simulator`
   - `ball_detector` from package `relbot_vision`  
      **Set parameters**:  
         - image_topic: `/output/moving_camera`  
         - method: `BLOB`  
         - hue: `<method>`  
         - debug: `true`   

   - `velocity_controller` from package `relbot_control`  
      **Set parameters**:  
         - target_size: `90.0`  
         - target_position: `90.0`  
         - linear_gain: `<linear_gain>`  
         - angular_gain: `<angular_gain>`  
         - debug: `true`

   - `differential_controller` from package `relbot_control`  

   - `showimage` from package `image_tools`  
      **Remapping**:  
         - `/image` --> `/velocity_controller/debug_image`