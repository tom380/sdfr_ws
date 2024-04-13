//=============================================
// Filename    : brightness_detection_better.cpp
// Authors     : Tom Fransen & Lute Golbach
// Group       : 9
// License     : N.A.
// Description : Implementation of the BrightnessDetectionBetter node
//=============================================

#include "../include/relbot_control/velocity_controller.hpp"
// #include "relbot_control/velocity_controller.hpp"
#include <stdexcept>
#include <algorithm>

using std::placeholders::_1;

VelocityController::VelocityController() : Node("velocity_controller") {
    // Set subscription image topic
    this->declare_parameter<std::string>("detection_topic", "/ball_detector/detection");
    std::string detection_topic_name;
    this->get_parameter("detection_topic", detection_topic_name);

    // Subscribe to topics
    detection_sub = this->create_subscription<relbot_vision::msg::BallDetection>(detection_topic_name, 10, std::bind(&VelocityController::detection_callback, this, _1));

    // Create topics to publish to
    velocity_pub = this->create_publisher<geometry_msgs::msg::TwistStamped>("velocity_controller/velocity", 10);
}

void VelocityController::detection_callback(relbot_vision::msg::BallDetection::ConstSharedPtr detection) {
    float target_size = 200; // pixels
    float target_position = 320; // pixels
    float size_error = target_size - std::max(detection->bounding_box.width, detection->bounding_box.height);
    float position_error = target_position - detection->bounding_box.centre_x;

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Size error: %f\tPosition error:%f", size_error, position_error);

    float linear_gain = 1; // meters per pixels per seconds
    float angular_gain = 1; // radians per pixels per seconds

    geometry_msgs::msg::TwistStamped velocity_msg;
    velocity_msg.twist.linear.x = linear_gain * size_error;
    velocity_msg.twist.angular.z = angular_gain * position_error;

    velocity_pub->publish(velocity_msg);
}


// The main function starts the node and "spins" it, i.e. handles all ROS2-related events such as receiving messages on topics
// You rarely need to add anything else to this function for ROS2 nodes
int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    try {rclcpp::spin(std::make_shared<VelocityController>());}
    catch (std::invalid_argument &e) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), e.what());
    }
   
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Shutting down...");
    rclcpp::shutdown();
    return 0;
}