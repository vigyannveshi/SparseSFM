/**
 * @file initializer.hpp
 * @brief Declares initialization routines for the sparse SfM pipeline.
 *        Selects the best initial frame pair via H-vs-F scoring and
 *        recovers the initial relative pose via Essential matrix decomposition.
 *
 * @author Rudraksha R. Bandodkar
 * @copyright Copyright (c) 2026 Rudraksha R. Bandodkar. MIT License, see
 *            LICENSE file in the project root for full terms.
 */


#ifndef INITIALIZER_HPP
#define INITIALIZER_HPP

#include <opencv2/opencv.hpp>
#include <vector>
#include "frame.hpp"
#include "features.hpp"

// InitialPair
struct InitialPair{
    int frame_idx;
    std::vector<cv::DMatch> matches;
};

// Inlier Data
struct InlierData{
    std::vector<cv::Point2f> pts1;
    std::vector<cv::Point2f> pts2;
    std::vector<cv::DMatch> inlier_matches;
};

// function to get matched points
void get_matched_points(const Frame& frame1,
                        const Frame& frame2,
                        const std::vector<cv::DMatch>& matches,
                        std::vector<cv::Point2f>& pts1,
                        std::vector<cv::Point2f>& pts2
                    );

// function to compute compute rh
float compute_rh(const std::vector<cv::Point2f>& pts1,
                 const std::vector<cv::Point2f>& pts2);

// function to select initial pair
InitialPair select_initial_pair(std::vector<Frame>& frames, FeatureType type, int ref_frame_idx = 0);

// function to get the recovered initial pose
bool recover_initial_pose(const std::vector<cv::Point2f>& pts1,
                          const std::vector<cv::Point2f>& pts2,
                          const cv::Mat& K,
                          cv::Mat& R,
                          cv::Mat& t,
                          cv::Mat& inlier_mask
                        );

                        
// function to get inlier points
InlierData filter_inliers(
    const std::vector<cv::Point2f>& pts1, 
    const std::vector<cv::Point2f>& pts2,
    const std::vector<cv::DMatch>& matches,
    const cv::Mat& inlier_mask
);

#endif