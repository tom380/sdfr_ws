//=============================================
// Filename    : brightness_detection_better.cpp
// Authors     : Tom Fransen & Lute Golbach
// Group       : 9
// License     : N.A.
// Description : Implementation of the BrightnessDetectionBetter node
//=============================================

#include "relbot_vision/ball_detector.hpp"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

using std::placeholders::_1;

BallDetector::BallDetector() : Node("ball_detector") {
    this->declare_parameter<std::string>("image_topic", "image");
    std::string image_topic_name;
    this->get_parameter("image_topic", image_topic_name);
    
    // Subscribe to the image topic
    image_sub = this->create_subscription<sensor_msgs::msg::Image>(image_topic_name, 10, std::bind(&BallDetector::image_callback, this, _1));
    // Create topic to publish ball detection messages to
    ballDetection_pub = this->create_publisher<relbot_vision::msg::BallDetection>("ball_detection", 10);
    debugImage_pub = this->create_publisher<sensor_msgs::msg::Image>("ball_detection_debug", 10);
}

void BallDetector::image_callback(sensor_msgs::msg::Image::ConstSharedPtr img) {
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::RGB8);
    } catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    cv::Mat gray_image;
    cv::cvtColor(cv_ptr->image, gray_image, cv::COLOR_BGR2GRAY);
    cv::GaussianBlur(gray_image, gray_image, cv::Size(9, 9), 2, 2);

    std::vector<cv::Vec3f> circles;
    cv::HoughCircles(gray_image, circles, cv::HOUGH_GRADIENT, 1, gray_image.rows / 8, 85, 65, 10, 0); // Adjust parameters

    RCLCPP_INFO(this->get_logger(), "Amount of found circles: %li", circles.size());

    relbot_vision::msg::BallDetection ball_detection_msg;
    ball_detection_msg.found = false;
    if (!circles.empty()) {
        auto circle = circles[0]; // Using the first detected circle
        relbot_vision::msg::BoundingBox bouding_box_msg;
        bouding_box_msg.centre_x = circle[0];
        bouding_box_msg.centre_y = circle[1];
        bouding_box_msg.width = bouding_box_msg.height = circle[2];

        ball_detection_msg.found = true;
        ball_detection_msg.bounding_box = bouding_box_msg;

        // Calculate bounding box coordinates
        int x = circle[0] - circle[2];
        int y = circle[1] - circle[2];
        int width = circle[2] * 2;
        int height = circle[2] * 2;

        // Draw the bounding box
        cv::rectangle(cv_ptr->image, cv::Point(x, y), cv::Point(x + width, y + height), cv::Scalar(0, 255, 0), 3);
    }

    ballDetection_pub->publish(ball_detection_msg);

    sensor_msgs::msg::Image::SharedPtr msg = cv_ptr->toImageMsg();
    debugImage_pub->publish(*msg);

}



// The main function starts the node and "spins" it, i.e. handles all ROS2-related events such as receiving messages on topics
// You rarely need to add anything else to this function for ROS2 nodes
int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BallDetector>());
    rclcpp::shutdown();
    return 0;
}