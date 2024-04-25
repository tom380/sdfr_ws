#include "relbot_vision/ball_detector.hpp"
#include "image_functions_sdfr/image_functions.hpp"

std::tuple<float, float, float> toHSV(const std::tuple<int, int, int>& bgr);

relbot_vision::msg::BallDetection BallDetector::detect_nocv(const sensor_msgs::msg::Image::ConstSharedPtr& img) {
    int step = 1; // Sampling step for pixels to reduce computations
    int greenPixelCount = 0;

    // Initialize bounding box coordinates to extreme values
    int minX = (int)img->width, minY = (int)img->height, maxX = 0, maxY = 0;

    for (int x = 0; x < (int)img->width; x += step) {
        for (int y = 0; y < (int)img->height; y += step) {
            const auto [h, s, v] = toHSV(image_functions::getPixelChannels(img, x, y));

            // Adjusted HSV range for a more specific shade of green and brightness/contrast conditions
            if (h >= hue - 10 && h <= hue + 10 && s >= 75 && v >= 20) {
                greenPixelCount++;
                minX = std::min(minX, x);
                minY = std::min(minY, y);
                maxX = std::max(maxX, x);
                maxY = std::max(maxY, y);
            }
        }
    }

    relbot_vision::msg::BallDetection ball_detection_msg;
    ball_detection_msg.found = false;

    if (greenPixelCount * step * step >= 100) {
        ball_detection_msg.found = true;

        ball_detection_msg.bounding_box.centre_x = (maxX + minX) / 2.0;
        ball_detection_msg.bounding_box.centre_y = (maxY + minY) / 2.0;
        ball_detection_msg.bounding_box.width = (maxX - minX);
        ball_detection_msg.bounding_box.height = (maxY - minY);
    }

    return ball_detection_msg;
}

std::tuple<float, float, float> toHSV(const std::tuple<int, int, int>& bgr) {
    // R, G, B values are divided by 255 
    // to change the range from 0..255 to 0..1 
    float b = std::get<0>(bgr) / 255.0; 
    float g = std::get<1>(bgr) / 255.0; 
    float r = std::get<2>(bgr) / 255.0; 
  
    // h, s, v = hue, saturation, value 
    float cmax = std::max({r, g, b}); // maximum of r, g, b 
    float cmin = std::min({r, g, b}); // minimum of r, g, b 
    float diff = cmax - cmin; // diff of cmax and cmin. 
    float h = -1, s = -1; 
  
    // if cmax and cmax are equal then h = 0 
    if (cmax == cmin) 
        h = 0; 
  
    // if cmax equal r then compute h 
    else if (cmax == r) 
        h = fmod(60 * ((g - b) / diff) + 360, 360); 
  
    // if cmax equal g then compute h 
    else if (cmax == g) 
        h = fmod(60 * ((b - r) / diff) + 120, 360); 
  
    // if cmax equal b then compute h 
    else if (cmax == b) 
        h = fmod(60 * ((r - g) / diff) + 240, 360); 
  
    // if cmax equal zero 
    if (cmax == 0) 
        s = 0; 
    else
        s = (diff / cmax) * 100; 
  
    // compute v 
    float v = cmax * 100;

    return {h / 2.0, s, v};
}