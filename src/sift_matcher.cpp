/**
 * @file sift_matcher.cpp
 * @brief Implements match_frames_sift declared in sift_matcher.hpp.
 *
 * @author Rudraksha R. Bandodkar
 * @copyright Copyright (c) 2026 Rudraksha R. Bandodkar. MIT License, see
 *            LICENSE file in the project root for full terms.
 */

#include "sift_matcher.hpp"
#include <iostream>
#include "constants.hpp"

// verbose for debugging
constexpr bool VERBOSE = false;

std::vector<cv::DMatch> match_frames_sift(const Frame& frame1, const Frame& frame2)
{
    // ensure that there are sufficient number of descriptors for BruteForce matching
    if (frame1.get_descriptors().rows < 2 || frame2.get_descriptors().rows < 2)
    {
        if (VERBOSE){
            std::cerr << "Warning: too few descriptors to run knnmatch, skipping." << std::endl;
        }
        return {};
    }

    // create BFMatcher with L2 norm distance
    cv::BFMatcher matcher(cv::NORM_L2);

    // matching
    std::vector<std::vector<cv::DMatch>> knn_matches;
    matcher.knnMatch(frame1.get_descriptors(), frame2.get_descriptors(), knn_matches, 2);

    // apply ratio test, keep matches if the best distance is significantly smaller,
    // than the second distance
    std::vector<cv::DMatch> good_matches;

    for (const auto& pair: knn_matches){
        // handling pairs having less than 2 matches
        if (pair.size()<2) continue;
        if (pair[0].distance < SIFT_RATIO_THRESHOLD * pair[1].distance)
        {
            good_matches.push_back(pair[0]);
        }
    }

    // print good matches count
    if (VERBOSE){
        std::cout << "Matches: " << good_matches.size() << " / " << knn_matches.size() << " passed the ratio test"<<std::endl;
    }

    return good_matches;
}