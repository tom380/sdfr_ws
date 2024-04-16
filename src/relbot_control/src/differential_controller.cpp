//=============================================
// Filename    : differential_controller.cpp
// Authors     : Tom Fransen & Lute Golbach
// Group       : 9
// License     : N.A.
// Description : Implementation of the DifferentialController node
//=============================================

#include "relbot_control/differential_controller.hpp"
#include <stdexcept>
#include <algorithm>

using std::placeholders::_1;

DifferentialController::DifferentialController() : Node("differential_controller") {
    // Set inra-wheel distance
    this->declare_parameter<double>("intra_wheel_width", 0.209);
    this->get_parameter("intra_wheel_width", width);

    // Set wheel radius
    this->declare_parameter<double>("wheel_radius", 0.0505);
    this->get_parameter("wheel_radius", wheel_radius);


    // Subscribe to topics
    velocity_sub = this->create_subscription<geometry_msgs::msg::TwistStamped>("/velocity_controller/velocity", 10, std::bind(&DifferentialController::velocity_callback, this, _1));
    
    // Create topics to publish to
    leftMotor_pub = this->create_publisher<example_interfaces::msg::Float64>("/input/left_motor/setpoint_vel", 10);
    rightMotor_pub = this->create_publisher<example_interfaces::msg::Float64>("/input/right_motor/setpoint_vel", 10);
}

void DifferentialController::velocity_callback(geometry_msgs::msg::TwistStamped::ConstSharedPtr velocity){
    double V = velocity->twist.linear.x;
    double omega = velocity->twist.angular.z;

    // if (V < 0.001) V = 0;
    // if (omega < 0.01) omega = 0;

    example_interfaces::msg::Float64 leftMotor_msg;
    leftMotor_msg.data = (V - omega * width / 2.0) / wheel_radius;
    example_interfaces::msg::Float64 rightMotor_msg;
    rightMotor_msg.data = (V + omega * width / 2.0) / wheel_radius;

    leftMotor_pub->publish(leftMotor_msg);
    rightMotor_pub->publish(rightMotor_msg);

    RCLCPP_INFO_EXPRESSION(this->get_logger(),leftMotor_msg.data == 0 && rightMotor_msg.data == 0, "Target reached, motors turned off");
}


int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    try {rclcpp::spin(std::make_shared<DifferentialController>());}
    catch (std::invalid_argument &e) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), e.what());
    }
   
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Shutting down...");
    rclcpp::shutdown();
    return 0;
}