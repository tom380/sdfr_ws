#include "relbot_vision/ball_detector.hpp"

cv::Mat BallDetector::colourMask(cv::Mat original_image, cv::Mat mask) {
    // Convert the original image to grayscale
    cv::Mat gray_image;
    cv::cvtColor(original_image, gray_image, cv::COLOR_BGR2GRAY);
    // Convert back to BGR for bitwise operations
    cv::cvtColor(gray_image, gray_image, cv::COLOR_GRAY2BGR);

    // Initialize the result image
    cv::Mat result = cv::Mat::zeros(original_image.size(), original_image.type());

    // Apply the mask to the original image to get the colored regions
    cv::Mat colored_parts;
    cv::bitwise_and(original_image, original_image, colored_parts, mask);

    // Invert the mask to combine with the grayscale image
    cv::Mat inverted_mask;
    cv::bitwise_not(mask, inverted_mask);

    // Apply the inverted mask to the grayscale image to get the grayscale regions
    cv::Mat gray_parts;
    cv::bitwise_and(gray_image, gray_image, gray_parts, inverted_mask);

    // Combine the two parts
    cv::add(colored_parts, gray_parts, result);

    return result;
}

BallDetector::Method BallDetector::stringToMethod(const std::string& mode) const {
    // Return the matching method
    if (mode == "HOUGH_CIRCLES") return Method::HOUGH_CIRCLES;
    else if (mode == "BLOB") return Method::BLOB;
    else if (mode == "NOCV") return Method::NOCV;
    // If no matching method has been found throw invalid_argument error, to indicate wrong method
    else throw std::invalid_argument("Unknown detection mode");
}