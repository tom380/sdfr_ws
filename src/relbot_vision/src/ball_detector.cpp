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
#include <stdexcept>

using std::placeholders::_1;

BallDetector::BallDetector() : Node("ball_detector") {
    // Set parameters
    this->declare_parameter<std::string>("image_topic", "image");
    std::string image_topic_name;
    this->get_parameter("image_topic", image_topic_name);
    
    this->declare_parameter<std::string>("method", "HOUGH_CIRCLES");
    std::string method_name;
    this->get_parameter("method", method_name);
    method = stringToEnum(method_name);

    this->declare_parameter<bool>("debug", false);
    this->get_parameter("debug", debug);

    // Subscribe to the image topic
    image_sub = this->create_subscription<sensor_msgs::msg::Image>(image_topic_name, 10, std::bind(&BallDetector::image_callback, this, _1));
    // Create topic to publish ball detection messages to
    detection_pub = this->create_publisher<relbot_vision::msg::BallDetection>("ball_detector/detection", 10);

    if (debug) debugImage_pub = this->create_publisher<sensor_msgs::msg::Image>("ball_detector/debug_image", 10);
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
    std::vector<cv::Vec3f> circles;

    switch (method) {
    case Method::HOUGH_CIRCLES:
        cv::cvtColor(cv_ptr->image, gray_image, cv::COLOR_BGR2GRAY);
        cv::GaussianBlur(gray_image, gray_image, cv::Size(9, 9), 2, 2);

        cv::HoughCircles(
            gray_image, circles, cv::HOUGH_GRADIENT, 
            1, gray_image.rows / 8, 
            85, 65, 10, 0);
        break;
    
    case Method::BLOB:
        // Configure blob detector parameters
        cv::SimpleBlobDetector::Params params;
        // Filter by Area.
        params.filterByArea = true;
        params.minArea = 100;
        params.maxArea = img->height*img->width;
        // Filter by Circularity
        params.filterByCircularity = true;
        params.minCircularity = 0.7; // Adjust this value to match your target's circularity
        // Filter by Color
        params.filterByColor = true;
        params.blobColor = 255; // Assuming the blobs are white on a darker background
        // Filter by Convexity
        params.filterByConvexity = false;
        // Filter by Inertia
        params.filterByInertia = false;
        params.minInertiaRatio = 0.5; // Adjust based on your target's inertia ratio

        // Create a blob detector with the above parameters
        cv::Ptr<cv::SimpleBlobDetector> detector = cv::SimpleBlobDetector::create(params);

        // Convert image to the HSV color space
        cv::Mat hsv_image;
        cv::cvtColor(cv_ptr->image, hsv_image, cv::COLOR_BGR2HSV);

        // Threshold the HSV image to get only the pre-known color
        cv::Mat mask;
        // HSV values H:0-180 S:0-255 V:0-255
        cv::inRange(hsv_image, cv::Scalar(50, 64, 10), cv::Scalar(70, 255, 255), mask);

        // You might want to apply some morphological operations to clean up the mask
        // cv::erode(mask, mask, cv::Mat(), cv::Point(-1, -1), 2);
        // cv::dilate(mask, mask, cv::Mat(), cv::Point(-1, -1), 2);

        // Detect blobs
        std::vector<cv::KeyPoint> keypoints;
        detector->detect(mask, keypoints);

        // Visualize detected blobs
        cv::Mat im_with_keypoints;
        cv::drawKeypoints(cv_ptr->image, keypoints, im_with_keypoints, cv::Scalar(0,255,0), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS );
        cv_ptr->image = im_with_keypoints;

        for (const auto& keypoint : keypoints) {
            float x = keypoint.pt.x;
            float y = keypoint.pt.y;
            float radius = keypoint.size / 2.0f; // Diameter to radius
            circles.push_back(cv::Vec3f(x, y, radius));
        }

        break;
    }

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

    detection_pub->publish(ball_detection_msg);
    if (debug) {
        sensor_msgs::msg::Image::SharedPtr msg = cv_ptr->toImageMsg();
        debugImage_pub->publish(*msg);
    }
}

Method BallDetector::stringToEnum(const std::string& mode) const {
    if (mode == "HOUGH_CIRCLES") return Method::HOUGH_CIRCLES;
    else if (mode == "BLOB") return Method::BLOB;
    else throw std::invalid_argument("Unknown detection mode");
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