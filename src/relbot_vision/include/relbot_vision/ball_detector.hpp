//=============================================
// Filename    : ball_detector.hpp
// Authors     : Tom Fransen & Lute Golbach
// Group       : 9
// License     : N.A.
// Description : Declaration of the BallDetector node
//=============================================

#ifndef BALL_DETECTOR_HPP
#define BALL_DETECTOR_HPP

#include <functional>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include <opencv2/opencv.hpp>

#include "sensor_msgs/msg/image.hpp"
#include "relbot_vision/msg/ball_detection.hpp"

// Placeholder for std::bind.
using std::placeholders::_1;

class BallDetector : public rclcpp::Node {
public:
    BallDetector();

private:
    // Detection methods
    enum class Method {HOUGH_CIRCLES, BLOB, NOCV};

    /// Callback functions.
    /**
     * @brief Callback to process any incoming Image messages.
     * 
     * @param img The image that was received.
    */
    void image_callback(sensor_msgs::msg::Image::ConstSharedPtr img);

    /// Utility functions
    // Convert string to repective method
    Method stringToMethod(const std::string& mode) const;
    // Set parameters for the blob detector
    void setBlobDetector();
    // Generate debug image
    sensor_msgs::msg::Image::SharedPtr generateDebug(const sensor_msgs::msg::Image::ConstSharedPtr& img, const relbot_vision::msg::BoundingBox& boundingBox);

    /// Detection method functions
    relbot_vision::msg::BallDetection detect_nocv(const sensor_msgs::msg::Image::ConstSharedPtr& img);
    relbot_vision::msg::BallDetection detect_houghCircles(const sensor_msgs::msg::Image::ConstSharedPtr& img);
    relbot_vision::msg::BallDetection detect_blob(const sensor_msgs::msg::Image::ConstSharedPtr& img);

    /// Private variables.
    bool debug;
    Method method;
    cv::Ptr<cv::SimpleBlobDetector> detector;
    double hue;

    /// Subscriber variables.
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub;

    /// Publisher variables.
    rclcpp::Publisher<relbot_vision::msg::BallDetection>::SharedPtr detection_pub;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debugImage_pub;
};

#endif /* BALL_DETECTOR_HPP */