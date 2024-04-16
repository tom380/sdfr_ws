//=============================================
// Filename    : velocity_controller.cpp
// Authors     : Tom Fransen & Lute Golbach
// Group       : 9
// License     : N.A.
// Description : Implementation of the VelocityController node
//=============================================

#include "../include/relbot_control/velocity_controller.hpp"
// #include "relbot_control/velocity_controller.hpp"
#include <stdexcept>
#include <algorithm>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

using std::placeholders::_1;

VelocityController::VelocityController() : Node("velocity_controller") {
    // Set subscription detection topic
    this->declare_parameter<std::string>("detection_topic", "/ball_detector/detection");
    std::string detection_topic_name;
    this->get_parameter("detection_topic", detection_topic_name);

    // Set subscription image topic
    this->declare_parameter<std::string>("image_topic", "/ball_detector/debug_image");
    std::string image_topic_name;
    this->get_parameter("image_topic", image_topic_name);

    // Set subscription image topic
    this->declare_parameter<bool>("debug", false);
    this->get_parameter("debug", debug);

    // Set target size
    this->declare_parameter<float>("target_size", 200);
    this->get_parameter("target_size", target_size);

    // Set target position
    this->declare_parameter<float>("target_position", 320);
    this->get_parameter("target_position", target_position);

    // Set linear gain
    this->declare_parameter<float>("linear_gain", 0.01); // meters per pixels per seconds
    this->get_parameter("linear_gain", linear_gain);

    // Set angular gain
    this->declare_parameter<float>("angular_gain", 1); // radians per pixels per seconds
    this->get_parameter("angular_gain", angular_gain);


    // Subscribe to topics
    detection_sub = this->create_subscription<relbot_vision::msg::BallDetection>(detection_topic_name, 10, std::bind(&VelocityController::detection_callback, this, _1));
    if (debug) image_sub = this->create_subscription<sensor_msgs::msg::Image>(image_topic_name, 10, std::bind(&VelocityController::image_callback, this, _1));

    // Create topics to publish to
    velocity_pub = this->create_publisher<geometry_msgs::msg::TwistStamped>("/velocity_controller/velocity", 10);
    if (debug) debugImage_pub = this->create_publisher<sensor_msgs::msg::Image>("/velocity_controller/debug_image", 10);
}

void VelocityController::detection_callback(relbot_vision::msg::BallDetection::ConstSharedPtr detection) {
    geometry_msgs::msg::TwistStamped velocity_msg;
    if (!detection->found) {
        velocity_pub->publish(velocity_msg);
        return;
    }
    
    float size_error = target_size - std::max(detection->bounding_box.width, detection->bounding_box.height);
    float position_error = target_position - detection->bounding_box.centre_x;

    RCLCPP_INFO(rclcpp::get_logger("velocity_controller"), "Size error: %f\tPosition error:%f", size_error, position_error);

    velocity_msg.twist.linear.x = linear_gain * size_error;
    velocity_msg.twist.angular.z = angular_gain * position_error;

    velocity_pub->publish(velocity_msg);
}

void VelocityController::image_callback(sensor_msgs::msg::Image::ConstSharedPtr image) {
    // Convert ROS message to openCV image
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(image, image->encoding);
    } catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    // Calculate bounding box coordinates
    cv::Point left_up(target_position - target_size/2, image->height/2 - target_size/2);
    cv::Point right_down(target_position + target_size/2, image->height/2 + target_size/2);   

    // Draw target bounding box
    cv::rectangle(cv_ptr->image, left_up, right_down, cv::Scalar(0, 0, 255), 3);

    cv::cvtColor(cv_ptr->image, cv_ptr->image, cv::COLOR_BGR2RGB);
    sensor_msgs::msg::Image::SharedPtr msg = cv_ptr->toImageMsg();
    debugImage_pub->publish(*msg);
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