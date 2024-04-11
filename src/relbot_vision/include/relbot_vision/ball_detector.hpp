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

#include "sensor_msgs/msg/image.hpp"
#include "relbot_vision/msg/ball_detection.hpp"

// Placeholder for std::bind.
using std::placeholders::_1;

enum class Method {HOUGH_CIRCLES, BLOB};

class BallDetector : public rclcpp::Node {
public:
    BallDetector();

private:
    /// Callback functions.
    /**
     * @brief Callback to process any incoming Image messages.
     * 
     * @param img The image that was received.
    */
    void image_callback(sensor_msgs::msg::Image::ConstSharedPtr img);
    Method stringToEnum(const std::string& mode) const;

    /// Private variables.
    bool debug;
    Method method;

    /// Subscriber variables.
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub;

    /// Publisher variables.
    rclcpp::Publisher<relbot_vision::msg::BallDetection>::SharedPtr detection_pub;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debugImage_pub;
};

#endif /* BALL_DETECTOR_HPP */