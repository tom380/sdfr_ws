#include "relbot_vision/ball_detector.hpp"
#include <cv_bridge/cv_bridge.h>

relbot_vision::msg::BallDetection BallDetector::detect_houghCircles(const sensor_msgs::msg::Image::ConstSharedPtr& img) {
    // Convert ROS message to openCV image
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(img, img->encoding);
    } catch (cv_bridge::Exception& e) {
        std::string error_msg = e.what();
        throw new cv_bridge::Exception("cv_bridge exception: " + error_msg);
    }

    // Convert to opencv bgr standard if picture is in rgb
    if (cv_ptr->encoding == "rgb8") {
        cv::cvtColor(cv_ptr->image, cv_ptr->image, cv::COLOR_RGB2BGR);
    }

    // Convert to gray image for HoughCircles function
    cv::Mat gray_image;
    cv::cvtColor(cv_ptr->image, gray_image, cv::COLOR_BGR2GRAY);
    // Blur to remove circularity inacuracies
    cv::GaussianBlur(gray_image, gray_image, cv::Size(9, 9), 2, 2);
    
    // Vector to store detected circles
    std::vector<cv::Vec3f> circles;
    
    // Find circles
    cv::HoughCircles(
        gray_image, circles, cv::HOUGH_GRADIENT, 
        1, gray_image.rows / 8, 
        85, 65, 10, 0);


    // Initialize ball_detection message
    relbot_vision::msg::BallDetection ball_detection_msg;
    ball_detection_msg.found = false;

    // Set bounding box and found if circles have been found
    if (!circles.empty()) {
        // Assuming first circle is the ball
        cv::Vec3f circle = circles[0];
        float x = circle[0];
        float y = circle[1];
        float radius = circle[2];

        // Create bounding_box message
        relbot_vision::msg::BoundingBox bouding_box_msg;
        bouding_box_msg.centre_x = x;
        bouding_box_msg.centre_y = y;
        bouding_box_msg.width = bouding_box_msg.height = radius * 2.0f;

        // Set ball_detection message
        ball_detection_msg.found = true;
        ball_detection_msg.bounding_box = bouding_box_msg;
    }

    return ball_detection_msg;
}