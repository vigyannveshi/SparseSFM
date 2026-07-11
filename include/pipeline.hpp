/**
 * @file pipeline.hpp
 * @brief Fine-grained pipeline steps for sparse SfM
 * @author Rudraksha R. Bandodkar
 * @copyright MIT License, see LICENSE
 */

#ifndef PIPELINE_HPP
#define PIPELINE_HPP

#include <map>
#include <vector>

#include "camera.hpp"
#include "features.hpp"
#include "frame.hpp"
#include "initializer.hpp"
#include "landmark.hpp"
#include "pose.hpp"
#include "triangulator.hpp"
#include "constants.hpp"

bool initialize_pipeline(
    const cv::Mat&                              K,
    const FeatureType&                          feature_type,
    std::vector<Frame>&                         frames,
    const int                                   reference_frame_idx,
    int&                                        selected_frame_idx,
    std::vector<Landmark>&                       landmarks,
    std::map<std::pair<int,int>, int>&          frame_keypoint2landmark_map
);

bool register_frame(
    std::vector<Frame>&                         frames,
    const int                                   i,
    std::vector<Landmark>&                      landmarks,
    std::map<std::pair<int,int>, int>&          frame_keypoint2landmark_map,
    const cv::Mat&                              K,
    const FeatureType&                          feature_type
);


void cull_landmarks(
    std::vector<Landmark>&      landmarks,
    const std::vector<Frame>&   frames,
    const cv::Mat&              K,
    double                      threshold_px = CULLING_THRESHOLD
);

void rebuild_fkp2lm(
    const std::vector<Landmark>&        landmarks,
    std::map<std::pair<int,int>, int>&  frame_keypoint2landmark_map
);

void normalize_scale(
    std::vector<Frame>&     frames,
    std::vector<Landmark>&  landmarks,
    const int               ref_frame_idx,
    const int               selected_frame_idx
);

#endif