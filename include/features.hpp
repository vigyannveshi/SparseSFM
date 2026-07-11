/**
 * @file features.hpp
 * @brief Unified feature extraction and matching interface supporting ORB and SIFT.
 * @author Rudraksha R. Bandodkar
 * @copyright MIT License, see LICENSE
 */

#ifndef FEATURE_EXTRACTOR_MATCHER_HPP
#define FEATURE_EXTRACTOR_MATCHER_HPP

#include <opencv2/opencv.hpp>

// frame datastructure
#include "frame.hpp"

enum class FeatureType{ORB, SIFT};

// extract features
void extract_features(Frame& frame, const FeatureType type);

// match frames
std::vector<cv::DMatch> match_frames(const Frame& frame1, const Frame& frame2, const FeatureType type);

#endif