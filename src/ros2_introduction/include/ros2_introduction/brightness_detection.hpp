//=============================================
// Filename    : brightness_detection.hpp
// Authors     : Tom Fransen & Lute Golbach
// Group       : 9
// License     : N.A.
// Description : Declaration of the BrightnessDetection node
//=============================================

#ifndef BRIGHTNESS_DETECTION_HPP
#define BRIGHTNESS_DETECTION_HPP

#include <functional>
#include <memory>
#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/image.hpp"
#include "example_interfaces/msg/u_int8.hpp"
#include "example_interfaces/msg/string.hpp"

// Placeholder for std::bind.
using std::placeholders::_1;

class BrightnessDetection : public rclcpp::Node {
public:
    BrightnessDetection();

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
    rclcpp::Publisher<example_interfaces::msg::UInt8>::SharedPtr lightLevel_pub;
    rclcpp::Publisher<example_interfaces::msg::String>::SharedPtr brightnessStatus_pub;
};

#endif /* BRIGHTNESS_DETECTION_HPP */