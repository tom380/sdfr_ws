//=============================================
// Filename    : brightness_detection_better.cpp
// Authors     : Tom Fransen & Lute Golbach
// Group       : 9
// License     : N.A.
// Description : Implementation of the BrightnessDetectionBetter node
//=============================================

#include "ros2_introduction/brightness_detection_better.hpp"
#include "image_functions_sdfr/image_functions.hpp"

using std::placeholders::_1;

BrightnessDetectionBetter::BrightnessDetectionBetter() : Node("brightness_detection") {
    // Subscribe to the image topic
    image_sub = this->create_subscription<sensor_msgs::msg::Image>("image", 10, std::bind(&BrightnessDetectionBetter::image_callback, this, _1));
    // Create topic to publish brightness messages to
    brightness_pub = this->create_publisher<brightness_msgs::msg::Brightness>("brightness", 10);
}

void BrightnessDetectionBetter::image_callback(sensor_msgs::msg::Image::ConstSharedPtr img) {
    // Get width and height of image
    const int width = image_functions::getImageWidth(img);
    const int height = image_functions::getImageHeight(img);

    // Create variable to sum each individual brightness on each iteration
    float totalBrightness = 0;

    // Iterate over all pixels
    for (int x = 0; x < width; x++) {
        for (int y = 0; y < height; y++) {
            // Add pixel brightness to total brightness
            totalBrightness += image_functions::getPixelBrightness(img, x, y);
        }
    }
    // Calculate average brightness
    const uint8_t avgBrightness = totalBrightness / (width * height);

    // Create brightness message
    brightness_msgs::msg::Brightness brightness_msg;
    // Set light level
    brightness_msg.light_level = avgBrightness;
    // Set status
    std::string status = "Average brighness is ";
    if (avgBrightness > treshold) status += "above";
    else if (avgBrightness == treshold) status += "equal to";
    else status += "below";
    status += " treshold value";
    brightness_msg.status = status;
    // Publish message
    brightness_pub->publish(brightness_msg);
}



// The main function starts the node and "spins" it, i.e. handles all ROS2-related events such as receiving messages on topics
// You rarely need to add anything else to this function for ROS2 nodes
int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BrightnessDetectionBetter>());
    rclcpp::shutdown();
    return 0;
}
