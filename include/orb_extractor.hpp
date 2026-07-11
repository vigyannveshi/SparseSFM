/**
 * @file orb_extractor.hpp
 * @brief ORB feature detection and description. extract_orb runs
 *        detectAndCompute on a single frame and stores the results
 *        via set_keypoints_and_descriptors. The ORB detector is
 *        created once by the caller and reused across all frames.
 *
 * @author Rudraksha R. Bandodkar
 * @copyright Copyright (c) 2026 Rudraksha R. Bandodkar. MIT License, see
 *            LICENSE file in the project root for full terms.
 */

#ifndef ORB_EXTRACTOR_HPP
#define ORB_EXTRACTOR_HPP

#include <opencv2/opencv.hpp>
#include "frame.hpp"

void extract_orb(Frame& frame, cv::Ptr<cv::ORB>& orb);

#endif