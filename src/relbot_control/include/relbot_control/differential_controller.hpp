//=============================================
// Filename    : differential_controller.hpp
// Authors     : Tom Fransen & Lute Golbach
// Group       : 9
// License     : N.A.
// Description : Declaration of the DifferentialController node
//=============================================

#ifndef DIFFERENTIAL_CONTROLLER_HPP
#define DIFFERENTIAL_CONTROLLER_HPP

#include <functional>
#include <memory>
#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "example_interfaces/msg/float64.hpp"

// Placeholder for std::bind.
using std::placeholders::_1;

class DifferentialController : public rclcpp::Node {
public:
    DifferentialController();

private:
    /// Callback functions.
    void velocity_callback(geometry_msgs::msg::TwistStamped::ConstSharedPtr velocity);

    /// Private variables.
    double width;
    double wheel_radius;

    /// Subscriber variables.
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr velocity_sub;

    /// Publisher variables.
    rclcpp::Publisher<example_interfaces::msg::Float64>::SharedPtr leftMotor_pub;
    rclcpp::Publisher<example_interfaces::msg::Float64>::SharedPtr rightMotor_pub;
};

#endif /* DIFFERENTIAL_CONTROLLER_HPP */