/**
 * @file orb_matcher.hpp
 * @brief ORB descriptor matching between consecutive frame pairs.
 *        match_frames runs BFMatcher with Hamming distance and applies
 *        Lowe's ratio test to filter weak matches.
 *
 * @author Rudraksha R. Bandodkar
 * @copyright Copyright (c) 2026 Rudraksha R. Bandodkar. MIT License, see
 *            LICENSE file in the project root for full terms.
 */

#ifndef ORB_MATCHER_HPP
#define ORB_MATCHER_HPP

#include <opencv2/opencv.hpp>
#include "frame.hpp"

std::vector<cv::DMatch> match_frames_orb(const Frame& frame1, const Frame& frame2);

#endif