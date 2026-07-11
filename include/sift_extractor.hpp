/**
 * @file sift_extractor.hpp
 * @brief SIFT feature detection and description. extract_sift runs
 *        detectAndCompute on a single frame and stores the results
 *        via set_keypoints_and_descriptors. The SIFT detector is
 *        created once by the caller and reused across all frames.
 *
 * @author Rudraksha R. Bandodkar
 * @copyright Copyright (c) 2026 Rudraksha R. Bandodkar. MIT License, see
 *            LICENSE file in the project root for full terms.
 */

#ifndef SIFT_EXTRACTOR_HPP
#define SIFT_EXTRACTOR_HPP

#include <opencv2/opencv.hpp>
#include "frame.hpp"

void extract_sift(Frame& frame, cv::Ptr<cv::SIFT>& sift);

#endif
