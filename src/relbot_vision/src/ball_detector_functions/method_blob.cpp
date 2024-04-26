//=============================================
// Filename    : method_blob.cpp
// Authors     : Tom Fransen & Lute Golbach
// Group       : 9
// License     : N.A.
// Description : Detection method for the BallDetector node,
//               uses opencv's blob detection function.
//=============================================

#include "relbot_vision/ball_detector.hpp"
#include <cv_bridge/cv_bridge.h>
#include <string>

relbot_vision::msg::BallDetection BallDetector::detect_blob(const sensor_msgs::msg::Image::ConstSharedPtr& img) {
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
    
    // Convert image to the HSV color space for easy colour selection
    cv::Mat hsv_image;
    cv::cvtColor(cv_ptr->image, hsv_image, cv::COLOR_BGR2HSV);

    // Create mask based on colour
    cv::Mat mask;
    cv::inRange(hsv_image, cv::Scalar((hue - 20) / 2.0f, 128, 50), cv::Scalar((hue + 20) / 2.0f, 255, 255), mask);

    // Clean-up mask
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE,cv::Size(5,5));
    cv::erode(mask, mask, kernel, cv::Point(-1, -1), 3);
    cv::dilate(mask, mask, kernel, cv::Point(-1, -1), 3);

    // Detect blobs
    std::vector<cv::KeyPoint> keypoints;
    detector->detect(mask, keypoints);

    // Initialize ball_detection message
    relbot_vision::msg::BallDetection ball_detection_msg;
    ball_detection_msg.found = false;

    // Set bounding box and found if keypoints have been found
    if (!keypoints.empty()) {
        // Assuming first keypoint is the ball
        const cv::KeyPoint& keypoint = keypoints[0];

        // Create bounding_box message
        relbot_vision::msg::BoundingBox bouding_box_msg;
        bouding_box_msg.centre_x = keypoint.pt.x;
        bouding_box_msg.centre_y = keypoint.pt.y;
        bouding_box_msg.width = bouding_box_msg.height = keypoint.size;

        // Set ball_detection message
        ball_detection_msg.found = true;
        ball_detection_msg.bounding_box = bouding_box_msg;
    }

    return ball_detection_msg;
}