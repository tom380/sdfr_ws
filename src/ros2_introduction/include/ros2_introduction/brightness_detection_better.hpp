//=============================================
// Filename    : brightness_detection_better.hpp
// Authors     : Tom Fransen & Lute Golbach
// Group       : 9
// License     : N.A.
// Description : Declaration of the BrightnessDetectionBetter node
//=============================================

#ifndef BRIGHTNESS_DETECTION_BETTER_HPP
#define BRIGHTNESS_DETECTION_BETTER_HPP

#include <functional>
#include <memory>
#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/image.hpp"
#include "brightness_msgs/msg/brightness.hpp"

// Placeholder for std::bind.
using std::placeholders::_1;

class BrightnessDetectionBetter : public rclcpp::Node {
public:
    BrightnessDetectionBetter();

private:
    /// Callback functions.
    /**
     * @brief Callback to process any incoming Image messages.
     * 
     * @param img The image that was received.
    */
    void image_callback(sensor_msgs::msg::Image::ConstSharedPtr img);

    /// Private variables.
    uint8_t treshold = 128;

    /// Subscriber variables.
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub;

    /// Publisher variables.
    rclcpp::Publisher<brightness_msgs::msg::Brightness>::SharedPtr brightness_pub;
};

#endif /* BRIGHTNESS_DETECTION_BETTER_HPP */