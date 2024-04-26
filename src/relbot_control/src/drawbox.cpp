//=============================================
// Filename    : drawbox.cpp
// Authors     : Tom Fransen & Lute Golbach
// Group       : 9
// License     : N.A.
// Description : VelocityController node function to draw a box
//               onto a input image to generate a debug image.
//=============================================

#include "relbot_control/velocity_controller.hpp"
#include "image_functions_sdfr/image_functions.hpp"

void VelocityController::drawbox(sensor_msgs::msg::Image::SharedPtr img, relbot_vision::msg::BoundingBox target_box) {
    // Get image width and height
    const unsigned int& width = img->width;
    const unsigned int& height = img->height;

    // Calculate bounding box corners
    const int x_left = target_box.centre_x - (target_box.width/2);
    const int x_right = target_box.centre_x + (target_box.width/2);
    const int y_up = target_box.centre_y - (target_box.height/2);
    const int y_down = target_box.centre_y + (target_box.height/2);

    // Horizontal lines
    for (int x = (x_left < 0 ? 0 : x_left); x < (x_right > (int)width ? (int)width : x_right); x++) {
        // Draw upper line
        if (y_up >= 0)
            image_functions::setPixelColor(img, x,y_up,255,0,0);
        // Draw lower line
        if (y_down < (int)height)
            image_functions::setPixelColor(img, x,y_down,255,0,0);
    }

    // Draw vertical lines
    for (int y = (y_up < 0 ? 0 : y_up); y < (y_down > (int)height ? (int)height : y_down); y++) {
        // Draw left line
        if (x_left >= 0)
            image_functions::setPixelColor(img, x_left,y,255,0,0);
        // Draw right line
        if (x_right < (int)width)
            image_functions::setPixelColor(img, x_right,y,255,0,0);
    }

}