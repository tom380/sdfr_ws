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
    this->declare_parameter<double>("hue", 60);
    this->get_parameter("hue", hue);
    // Throw error if hue is not between openCV hue range
    if (hue < 0 || hue > 180) throw std::invalid_argument("Hue value out of colour range, should be between 0 and 180");


    // Subscribe to topics
    image_sub = this->create_subscription<sensor_msgs::msg::Image>(image_topic_name, 10, std::bind(&BallDetector::image_callback, this, _1));

    // Create topics to publish to
    detection_pub = this->create_publisher<relbot_vision::msg::BallDetection>("ball_detector/detection", 10);
    if (debug) debugImage_pub = this->create_publisher<sensor_msgs::msg::Image>("ball_detector/debug_image", 10);


    // Create blob detector if method is BLOB
    if (method == Method::BLOB) {
        // Blob detector parameters
        cv::SimpleBlobDetector::Params params;
        params.filterByArea = true;
        params.minArea = 100;
        params.maxArea = std::numeric_limits<float>::infinity();
        params.filterByCircularity = true;
        params.minCircularity = 0.6;
        params.filterByColor = false;
        params.filterByConvexity = false;
        params.filterByInertia = false;

        // Create a blob detector with the above parameters
        detector = cv::SimpleBlobDetector::create(params);
    }
}

void BallDetector::image_callback(sensor_msgs::msg::Image::ConstSharedPtr img) {
    // Convert ROS message to openCV image
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    // Vector to store detected circles
    std::vector<cv::Vec3f> circles;

    // Detect circles with selected method
    switch (method) {
    case Method::HOUGH_CIRCLES:{
        // Convert to gray image for HoughCircles function
        cv::Mat gray_image;
        cv::cvtColor(cv_ptr->image, gray_image, cv::COLOR_BGR2GRAY);
        // Blur to remove circularity inacuracies
        cv::GaussianBlur(gray_image, gray_image, cv::Size(9, 9), 2, 2);

        // Find circles
        cv::HoughCircles(
            gray_image, circles, cv::HOUGH_GRADIENT, 
            1, gray_image.rows / 8, 
            85, 65, 10, 0);
        break;
    }
    case Method::BLOB:
        // Convert image to the HSV color space for easy colour selection
        cv::Mat hsv_image;
        cv::cvtColor(cv_ptr->image, hsv_image, cv::COLOR_BGR2HSV);

        // Create mask based on colour
        cv::Mat mask;
        cv::inRange(hsv_image, cv::Scalar(hue - 10, 128, 50), cv::Scalar(hue + 10, 255, 255), mask);

        // Clean-up mask
        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE,cv::Size(5,5));
        cv::erode(mask, mask, kernel, cv::Point(-1, -1), 3);
        cv::dilate(mask, mask, kernel, cv::Point(-1, -1), 3);

        // Detect blobs
        std::vector<cv::KeyPoint> keypoints;
        detector->detect(mask, keypoints);

        // Visualize mask
        cv_ptr->image = colourMask(cv_ptr->image, mask);

        // Save keypoints as circles
        for (const cv::KeyPoint& keypoint : keypoints) {
            const float x = keypoint.pt.x;
            const float y = keypoint.pt.y;
            const float radius = keypoint.size / 2.0f;
            circles.push_back(cv::Vec3f(x, y, radius));
        }

        break;
    }

    
    // Initialize ball_detection message
    relbot_vision::msg::BallDetection ball_detection_msg;
    ball_detection_msg.found = false;

    // Set bounding_box and found if circles have been found
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

        // Draw the bounding box if debug is on
        if (debug) {
            // Calculate bounding box coordinates
            cv::Point left_up(x - radius, y - radius);
            cv::Point right_down(x + radius, y + radius);            
            
            // Draw bounding box
            cv::rectangle(cv_ptr->image, left_up, right_down, cv::Scalar(0, 255, 0), 3);
        }
    }
    
    // Indicate if ball was found
    if (ball_detection_msg.found) {
        RCLCPP_INFO(this->get_logger(), "Found ball at x=%i, y=%i", (int)ball_detection_msg.bounding_box.centre_x, (int)ball_detection_msg.bounding_box.centre_y);
    }
    else RCLCPP_INFO(this->get_logger(), "No ball has been found");

    // Publish messages
    detection_pub->publish(ball_detection_msg);
    if (debug) {
        cv::cvtColor(cv_ptr->image, cv_ptr->image, cv::COLOR_BGR2RGB);
        sensor_msgs::msg::Image::SharedPtr msg = cv_ptr->toImageMsg();
        debugImage_pub->publish(*msg);
    }
}

cv::Mat BallDetector::colourMask(cv::Mat original_image, cv::Mat mask) {
    // Convert the original image to grayscale
    cv::Mat gray_image;
    cv::cvtColor(original_image, gray_image, cv::COLOR_BGR2GRAY);
    // Convert back to BGR for bitwise operations
    cv::cvtColor(gray_image, gray_image, cv::COLOR_GRAY2BGR);

    // Initialize the result image
    cv::Mat result = cv::Mat::zeros(original_image.size(), original_image.type());

    // Apply the mask to the original image to get the colored regions
    cv::Mat colored_parts;
    cv::bitwise_and(original_image, original_image, colored_parts, mask);

    // Invert the mask to combine with the grayscale image
    cv::Mat inverted_mask;
    cv::bitwise_not(mask, inverted_mask);

    // Apply the inverted mask to the grayscale image to get the grayscale regions
    cv::Mat gray_parts;
    cv::bitwise_and(gray_image, gray_image, gray_parts, inverted_mask);

    // Combine the two parts
    cv::add(colored_parts, gray_parts, result);

    return result;
}

BallDetector::Method BallDetector::stringToMethod(const std::string& mode) const {
    // Return the matching method
    if (mode == "HOUGH_CIRCLES") return Method::HOUGH_CIRCLES;
    else if (mode == "BLOB") return Method::BLOB;
    // If no matching method has been found throw invalid_argument error, to indicate wrong method
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