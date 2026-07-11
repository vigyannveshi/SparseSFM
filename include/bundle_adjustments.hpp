/**
 * @file bundle_adjustment.hpp
 * @brief Bundle adjustment via Levenberg-Marquardt with Schur complement
 * @author Rudraksha R. Bandodkar
 * @copyright MIT License, see LICENSE
 */

#ifndef BUNDLE_ADJUSTMENTS_HPP
#define BUNDLE_ADJUSTMENTS_HPP

#include <vector>
#include <map>

#include <opencv2/opencv.hpp>

#include "frame.hpp"
#include "landmark.hpp"
#include "pose.hpp"
#include "constants.hpp"

void bundle_adjust(
    std::vector<Frame>&         frames,
    std::vector<Landmark>&      landmarks,
    const cv::Mat&              K,
    const int                   window_start = 0,
    int                         window_end = -1, // -1 for global
    const int                   max_iterations = BA_GLOBAL_ITERATIONS,
    std::vector<int>            fixed_frames = {}
);

#endif