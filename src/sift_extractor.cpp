/**
 * @file sift_extractor.cpp
 * @brief Implements extract_sift declared in sift_extractor.hpp.
 *
 * @author Rudraksha R. Bandodkar
 * @copyright Copyright (c) 2026 Rudraksha R. Bandodkar. MIT License, see
 *            LICENSE file in the project root for full terms.
 */

#include "sift_extractor.hpp"
#include <iostream>

void extract_sift(Frame& frame, cv::Ptr<cv::SIFT>& sift){
    // read image (gray-scale) using image path from frame
    cv::Mat img = cv::imread(frame.get_image_path(), cv::IMREAD_GRAYSCALE);

    if (img.empty()){
        std::cerr << "Error: could not load image: "<<frame.get_image_path() << std::endl;
        return;
    }

    // storing keypoints and descriptors
    std::vector<cv::KeyPoint> keypoints;
    cv::Mat descriptors;

    // extracting SIFT keypoints and descriptors
    sift->detectAndCompute(img, cv::noArray(), keypoints, descriptors);

    // setting keypoints and descriptors to that frame
    frame.set_keypoints_and_descriptors(keypoints, descriptors);
}