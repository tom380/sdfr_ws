//=============================================
// Filename    : velocity_controller.hpp
// Authors     : Tom Fransen & Lute Golbach
// Group       : 9
// License     : N.A.
// Description : Declaration of the VelocityController node
//=============================================

#ifndef VELOCITY_CONTROLLER_HPP
#define VELOCITY_CONTROLLER_HPP

#include <functional>
#include <memory>
#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "sensor_msgs/msg/image.hpp"
//#include "../../../install/relbot_vision/include/relbot_vision/relbot_vision/msg/ball_detection.hpp"
#include "relbot_vision/msg/ball_detection.hpp"

// Placeholder for std::bind.
using std::placeholders::_1;

class VelocityController : public rclcpp::Node {
public:
    VelocityController();

private:
    /// Callback functions.
    void detection_callback(relbot_vision::msg::BallDetection::ConstSharedPtr detection);
    void image_callback(sensor_msgs::msg::Image::ConstSharedPtr image);

    /// Private variables.
    bool debug;
    float target_size = 200; // pixels
    float target_position = 320; // pixels

    /// Subscriber variables.
    rclcpp::Subscription<relbot_vision::msg::BallDetection>::SharedPtr detection_sub;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub;

    /// Publisher variables.
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr velocity_pub;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debugImage_pub;
};

#endif /* VELOCITY_CONTROLLER_HPP */