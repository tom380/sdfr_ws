//=============================================
// Filename    : brightness_detection_better.cpp
// Authors     : Tom Fransen & Lute Golbach
// Group       : 9
// License     : N.A.
// Description : Implementation of the BrightnessDetectionBetter node
//=============================================

#include "relbot_vision/ball_detector.hpp"
#include <cv_bridge/cv_bridge.h>
#include <stdexcept>
#include <algorithm>
#include "image_functions_sdfr/image_functions.hpp"

using std::placeholders::_1;

BallDetector::BallDetector() : Node("ball_detector") {
    // Set subscription image topic
    this->declare_parameter<std::string>("image_topic", "image");
    std::string image_topic_name;
    this->get_parameter("image_topic", image_topic_name);
    
    // Set detection method
    this->declare_parameter<std::string>("method", "BLOB");
    std::string method_name;
    this->get_parameter("method", method_name);
    method = stringToMethod(method_name);

    // Set debug on or off
    this->declare_parameter<bool>("debug", false);
    this->get_parameter("debug", debug);

    // Set hue for blob detection
    this->declare_parameter<double>("hue", 120);
    this->get_parameter("hue", hue);
    // Throw error if hue is not between openCV hue range
    if (hue < 0 || hue > 360) throw std::invalid_argument("Hue value out of colour range, should be between 0 and 360");


    // Subscribe to topics
    image_sub = this->create_subscription<sensor_msgs::msg::Image>(image_topic_name, 10, std::bind(&BallDetector::image_callback, this, _1));

    // Create topics to publish to
    detection_pub = this->create_publisher<relbot_vision::msg::BallDetection>("ball_detector/detection", 10);
    if (debug) debugImage_pub = this->create_publisher<sensor_msgs::msg::Image>("ball_detector/debug_image", 10);


    // Create blob detector if method is BLOB
    if (method == Method::BLOB) setBlobDetector();
    
}

void BallDetector::image_callback(sensor_msgs::msg::Image::ConstSharedPtr img) {
    relbot_vision::msg::BallDetection ball_detection_msg;
    
    // Detect ball with selected method
    switch (method) {
    case Method::HOUGH_CIRCLES:
        ball_detection_msg = detect_houghCircles(img);
        break;
    case Method::BLOB:
        ball_detection_msg = detect_blob(img);
        break;
    case Method::NOCV:
        ball_detection_msg = detect_nocv(img);
        break;
    }

    // Indicate if ball was found
    if (ball_detection_msg.found) {
        RCLCPP_INFO(this->get_logger(), "Found ball at x=%i, y=%i", (int)ball_detection_msg.bounding_box.centre_x, (int)ball_detection_msg.bounding_box.centre_y);
    }
    else RCLCPP_INFO(this->get_logger(), "No ball has been found");

    // Publish messages
    detection_pub->publish(ball_detection_msg);
    if (debug) {
        sensor_msgs::msg::Image::SharedPtr debug_img = generateDebug(img, ball_detection_msg.bounding_box);
        debugImage_pub->publish(*debug_img);
    }
}


// The main function starts the node and "spins" it, i.e. handles all ROS2-related events such as receiving messages on topics
// You rarely need to add anything else to this function for ROS2 nodes
int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    try {rclcpp::spin(std::make_shared<BallDetector>());}
    catch (std::invalid_argument &e) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), e.what());
    }
   
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Shutting down...");
    rclcpp::shutdown();
    return 0;
}