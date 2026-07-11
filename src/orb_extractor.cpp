/**
 * @file orb_extractor.cpp
 * @brief Implements extract_orb declared in orb_extractor.hpp.
 *
 * @author Rudraksha R. Bandodkar
 * @copyright Copyright (c) 2026 Rudraksha R. Bandodkar. MIT License, see
 *            LICENSE file in the project root for full terms.
 */

#include "orb_extractor.hpp"
#include <iostream>

void extract_orb(Frame& frame, cv::Ptr<cv::ORB>& orb)
{
    // loading the image from frame's image path (get grayscale image)
    cv::Mat img = cv::imread(frame.get_image_path(), cv::IMREAD_GRAYSCALE);

    // check image loaded successfully
    if (img.empty()){
        std::cerr << "Error: could not load image: "<< frame.get_image_path() << std::endl;
        return;
    }

    // run detectAndCompute — fills keypoints and descriptors
    std::vector<cv::KeyPoint> keypoints;
    cv::Mat descriptors;
    
    orb->detectAndCompute(img, cv::noArray(), keypoints, descriptors);

    // store results back into the frame
    frame.set_keypoints_and_descriptors(keypoints, descriptors);
}