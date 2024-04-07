//=============================================
// Filename    : brightness_detection.cpp
// Authors     : Tom Fransen & Lute Golbach
// Group       : 9
// License     : N.A.
// Description : Implementation of the BrightnessDetection node
//=============================================

#include "ros2_introduction/brightness_detection.hpp"
#include "image_functions_sdfr/image_functions.hpp"

using std::placeholders::_1;

BrightnessDetection::BrightnessDetection() : Node("brightness_detection") {
    // Subscribe to the image topic
    image_sub = this->create_subscription<sensor_msgs::msg::Image>("image", 10, std::bind(&BrightnessDetection::image_callback, this, _1));
    // Create topic to publish light level to
    lightLevel_pub = this->create_publisher<example_interfaces::msg::UInt8>("light_level", 10);
    // Create topic to publish brightness status to
    brightnessStatus_pub = this->create_publisher<example_interfaces::msg::String>("brightness_level", 10);
}

void BrightnessDetection::image_callback(sensor_msgs::msg::Image::ConstSharedPtr img) {
    // Get width and height of image
    const int width = image_functions::getImageWidth(img);
    const int height = image_functions::getImageHeight(img);

    // Create variable to sum each individual brightness on each iteration
    float totalBrightness = 0;

    // Iterate over all pixels
    for (int x = 0; x < width; x++) {
        for (int y = 0; y < height; y++) {
            // Add pixel brighness to total brightness
            totalBrightness += image_functions::getPixelBrightness(img, x, y);
        }
    }
    // Calculate average brightness
    const uint8_t avgBrightness = totalBrightness / (width * height);

    // Create light level message
    example_interfaces::msg::UInt8 lightLevel_msg;
    // Set light level
    lightLevel_msg.data = avgBrightness;
    // Publish light level message
    lightLevel_pub->publish(lightLevel_msg);

    // Create brightness status message
    example_interfaces::msg::String brightnessStatus_msg;
    // Set brightness status
    std::string message = "Average brighness is ";
    if (avgBrightness > treshold) message += "above";
    else if (avgBrightness == treshold) message += "equal to";
    else message += "below";
    message += " treshold value";
    brightnessStatus_msg.data = message;
    // Publish brightness status message
    brightnessStatus_pub->publish(brightnessStatus_msg);
}



// The main function starts the node and "spins" it, i.e. handles all ROS2-related events such as receiving messages on topics
// You rarely need to add anything else to this function for ROS2 nodes
int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BrightnessDetection>());
    rclcpp::shutdown();
    return 0;
}
