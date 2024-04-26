//=============================================
// Filename    : utils.cpp
// Authors     : Tom Fransen & Lute Golbach
// Group       : 9
// License     : N.A.
// Description : Utitlity functions for the BallDetector node
//=============================================

#include "relbot_vision/ball_detector.hpp"
#include "image_functions_sdfr/image_functions.hpp"

BallDetector::Method BallDetector::stringToMethod(const std::string& mode) const {
    // Return the matching method
    if (mode == "HOUGH_CIRCLES") return Method::HOUGH_CIRCLES;
    else if (mode == "BLOB") return Method::BLOB;
    else if (mode == "NOCV") return Method::NOCV;
    // If no matching method has been found throw invalid_argument error, to indicate wrong method
    else throw std::invalid_argument("Unknown detection mode");
}

void BallDetector::setBlobDetector() {
    // Blob detector parameters
    cv::SimpleBlobDetector::Params params;
    params.filterByArea = true;
    params.minArea = 100;
    params.maxArea = std::numeric_limits<float>::infinity();
    params.filterByCircularity = true;
    params.minCircularity = 0.6;
    params.filterByColor = false;
    params.filterByConvexity = false;
    params.filterByInertia = false;

    // Create a blob detector with the above parameters
    detector = cv::SimpleBlobDetector::create(params);
}

sensor_msgs::msg::Image::SharedPtr BallDetector::generateDebug(const sensor_msgs::msg::Image::ConstSharedPtr& img, const relbot_vision::msg::BoundingBox& bounding_box) {
    // Copy image to a modifyable object
    sensor_msgs::msg::Image::SharedPtr debug_img = std::make_shared<sensor_msgs::msg::Image>();
    image_functions::copyImageProperties(debug_img, img);
    debug_img->data = img->data;

    // Calculate the corners of the bounding box
    const int minX = bounding_box.centre_x - bounding_box.width / 2.0;
    const int maxX = bounding_box.centre_x + bounding_box.width / 2.0;
    const int minY = bounding_box.centre_y - bounding_box.height / 2.0;
    const int maxY = bounding_box.centre_y + bounding_box.height / 2.0;

    // Draw horizontal lines
    for (int x = minX; x < maxX; x++) {
        // Draw with a thickness of 5 pixels
        for (int i = -2; i <= 2; i++) {
            // Draw bottom line
            if (minY + i >= 0 && minY + i < (int)debug_img->width)
                image_functions::setPixelColor(debug_img, x, minY + i, 0, 0, 255);
            // Draw top line
            if (maxY + i >= 0 && maxY + i < (int)debug_img->width)                            
                image_functions::setPixelColor(debug_img, x, maxY + i, 0, 0, 255);
        }
    }
    // Draw vertical lines
    for (int y = minY; y < maxY; y++) {
        // Draw with a thickness of 5 pixels
        for (int i = -2; i <= 2; i++) {
            // Draw left line
            if (minX + i >= 0 && minX + i < (int)debug_img->width)
                image_functions::setPixelColor(debug_img, minX + i, y, 0, 0, 255);
            // Draw right line
            if (maxX + i >= 0 && maxX + i < (int)debug_img->width)
                image_functions::setPixelColor(debug_img, maxX + i, y, 0, 0, 255);
        }
    }

    // Return image
    return debug_img;
}