/**
 * @file sift_matcher.hpp
 * @brief SIFT descriptor matching between consecutive frame pairs.
 *        match_frames_sift runs BFMatcher with the correct norm for
 *        SIFT's floating point descriptors and applies Lowe's ratio
 *        test to filter weak matches.
 *
 * @author Rudraksha R. Bandodkar
 * @copyright Copyright (c) 2026 Rudraksha R. Bandodkar. MIT License, see
 *            LICENSE file in the project root for full terms.
 */

#ifndef SIFT_MATCHER_HPP
#define SIFT_MATCHER_HPP

#include <opencv2/opencv.hpp>
#include "frame.hpp"

std::vector<cv::DMatch> match_frames_sift(const Frame& frame1, const Frame& frame2);

#endif