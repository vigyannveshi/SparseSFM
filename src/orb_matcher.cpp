/**
 * @file orb_matcher.cpp
 * @brief Implements match_frames declared in orb_matcher.hpp.
 *
 * @author Rudraksha R. Bandodkar
 * @copyright Copyright (c) 2026 Rudraksha R. Bandodkar. MIT License, see
 *            LICENSE file in the project root for full terms.
 */

#include "orb_matcher.hpp"
#include <iostream>
#include "constants.hpp"

// to enable verbose while debugging
constexpr bool VERBOSE = false;

std::vector<cv::DMatch> match_frames_orb(const Frame& frame1, const Frame& frame2){
    // ensuring we have atleast two descriptors per-frame
    if (frame1.get_descriptors().rows < 2 || frame2.get_descriptors().rows < 2)
    {
        if (VERBOSE){
            std::cerr <<"Warning: to few descriptors to run knnMatch, skipping." << std::endl;
        }
        return {};
    }

    // create BFMatcher with Hamming distance
    cv::BFMatcher matcher(cv::NORM_HAMMING);

    // run knnMatch with k=2 to get two best matches per keypoint
    std::vector<std::vector<cv::DMatch>> knn_matches;
    matcher.knnMatch(frame1.get_descriptors(), frame2.get_descriptors(), knn_matches, 2);

    // apply ratio test — keep match only if best distance is significantly
    // smaller than second best
    std::vector<cv::DMatch> good_matches;
    for (const auto& pair : knn_matches)
    {   
        // handling pair having less than two matches
        if (pair.size() < 2) continue;
        if (pair[0].distance < ORB_RATIO_THRESHOLD * pair[1].distance){
            good_matches.push_back(pair[0]);
        }
    }

    if (VERBOSE){
        std::cout << "Matches: " << good_matches.size()
        << " / " << knn_matches.size() << " passed ratio test" << std::endl;
    }

    return good_matches;
}


/*
Notes:

-x-x-x-x-x-x-x-x-x-x-x-x-x-x-x-x-x-x-

cv::DMatch
----------
It's a simple struct defined by OpenCV representing a single match between one descriptor in the query set and one descriptor in the train set.

Three fields you care about:
queryIdx   → index into frame1's keypoints/descriptors
trainIdx   → index into frame2's keypoints/descriptors
distance   → Hamming distance between the two descriptors


Concrete example — say keypoint 42 in frame1 best matches keypoint 137 in frame2 with Hamming distance 30:
pair[0].queryIdx  = 42
pair[0].trainIdx  = 137
pair[0].distance  = 30

-x-x-x-x-x-x-x-x-x-x-x-x-x-x-x-x-x-x-

*/