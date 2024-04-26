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
    // Calculate bounding box corners
    const int x_left = target_box.centre_x - (target_box.width/2);
    const int x_right = target_box.centre_x + (target_box.width/2);
    const int y_up = target_box.centre_y - (target_box.height/2);
    const int y_down = target_box.centre_y + (target_box.height/2);

    // Horizontal lines
    for (int x = x_left; x < x_right; x++) {
        // Draw upper line
        image_functions::setPixelColor(img, x,y_up,255,0,0);
        // Draw lower line
        image_functions::setPixelColor(img, x,y_down,255,0,0);
    }

    // Draw vertical lines
    for (int y = y_up; y < y_down; y++) {
        // Draw left line
        image_functions::setPixelColor(img, x_left,y,255,0,0);
        // Draw right line
        image_functions::setPixelColor(img, x_right,y,255,0,0);
    }

}