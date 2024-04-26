//=============================================
// Filename    : method_nocv.cpp
// Authors     : Tom Fransen & Lute Golbach
// Group       : 9
// License     : N.A.
// Description : Ball detection method for the BallDetector node,
//               does not make use of cvbridge/opencv
//=============================================

#include "relbot_vision/ball_detector.hpp"
#include "image_functions_sdfr/image_functions.hpp"

// Convert bgr to hsv
std::tuple<float, float, float> toHSV(const std::tuple<int, int, int>& bgr);

relbot_vision::msg::BallDetection BallDetector::detect_nocv(const sensor_msgs::msg::Image::ConstSharedPtr& img) {
    // Sampling steps for pixels, to reduce computations
    const unsigned int step = 3;
    // Amount of found pixels within treshold
    int foundPixelCount = 0;

    // Initialize bounding box coordinates to extreme values
    int minX = (int)img->width, minY = (int)img->height, maxX = 0, maxY = 0;

    for (int x = 0; x < (int)img->width; x += step) {
        for (int y = 0; y < (int)img->height; y += step) {
            // Convert pixel values to hsv
            const auto [h, s, v] = toHSV(image_functions::getPixelChannels(img, x, y));

            // Check if pixel values lies within tresholds
            if (h >= hue - 10 && h <= hue + 10 && s >= 60 && v >= 20) {
                // Add to pixel counter
                foundPixelCount++;

                // Set new min and max values
                minX = std::min(minX, x);
                minY = std::min(minY, y);
                maxX = std::max(maxX, x);
                maxY = std::max(maxY, y);
            }
        }
    }

    // Create ball detection message
    relbot_vision::msg::BallDetection ball_detection_msg;
    ball_detection_msg.found = false;

    // Remove noise by only detecting blobs with an area bigger than 100 pixels^2
    if (foundPixelCount * step * step >= 100) {
        // Ball is found
        ball_detection_msg.found = true;

        // Calculate bounding box
        ball_detection_msg.bounding_box.centre_x = (maxX + minX) / 2.0;
        ball_detection_msg.bounding_box.centre_y = (maxY + minY) / 2.0;
        ball_detection_msg.bounding_box.width = (maxX - minX);
        ball_detection_msg.bounding_box.height = (maxY - minY);
    }

    // Return ball detection
    return ball_detection_msg;
}

std::tuple<float, float, float> toHSV(const std::tuple<int, int, int>& bgr) {
    // Set bgr range from 0 to 1
    float b = std::get<0>(bgr) / 255.0; 
    float g = std::get<1>(bgr) / 255.0; 
    float r = std::get<2>(bgr) / 255.0; 
  
    // Calculate max and min colour
    float cmax = std::max({r, g, b});
    float cmin = std::min({r, g, b});
    // Calcualte difference between max an min colour
    float diff = cmax - cmin;
    // Create hsv variables
    float h = 0, s = 0, v = cmax * 100;
  
    // Calculate hue
    if (cmax == r) 
        h = fmod(60 * ((g - b) / diff) + 360, 360); 
    else if (cmax == g) 
        h = fmod(60 * ((b - r) / diff) + 120, 360); 
    else if (cmax == b) 
        h = fmod(60 * ((r - g) / diff) + 240, 360); 
  
    // If max value is not zero (fully desaturated), then the saturation can be calculated
    if (cmax != 0)
        s = (diff / cmax) * 100;

    // Return hsv, h is divided by 2 to match cv standard
    return {h / 2.0, s, v};
}