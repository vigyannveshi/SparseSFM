/**
 * @file initializer.cpp
 * @brief Implements initialization routines declared in initializer.hpp.
 *
 * @author Rudraksha R. Bandodkar
 * @copyright Copyright (c) 2026 Rudraksha R. Bandodkar. MIT License, see
 *            LICENSE file in the project root for full terms.
 */

#include "initializer.hpp"
#include <iostream>
#include "constants.hpp"

// function to get matched points
void get_matched_points(const Frame& frame1,
                        const Frame& frame2,
                        const std::vector<cv::DMatch>& matches,
                        std::vector<cv::Point2f>& pts1,
                        std::vector<cv::Point2f>& pts2){
    for (const auto& m : matches){
        pts1.push_back(frame1.get_keypoints()[m.queryIdx].pt);
        pts2.push_back(frame2.get_keypoints()[m.trainIdx].pt);
    }
}


// function to compute compute rh (Ratio Homography)
float compute_rh(const std::vector<cv::Point2f>& pts1,
                 const std::vector<cv::Point2f>& pts2){
    // inlier mask                
    cv::Mat mask_h, mask_f;

    // compute the homography matrix
    cv::findHomography(pts1, pts2, cv::RANSAC, 3.0, mask_h);
    cv::findFundamentalMat(pts1, pts2, cv::FM_RANSAC, 3.0, 0.99, mask_f);

    float SH = cv::countNonZero(mask_h);
    float SF = cv::countNonZero(mask_f);
    
    // handling non-zero denominator 
    return SH + SF == 0? 1:SH / (SH + SF);
}


// function to select initial pair
InitialPair select_initial_pair(std::vector<Frame>& frames, FeatureType type, int ref_frame_idx){

    // extract features using given type
    extract_features(frames[ref_frame_idx],type);

    // create temporary variables for the loop
    std::vector<cv::DMatch> matches;
    std::vector<cv::Point2f> pts1;
    std::vector<cv::Point2f> pts2;

    // variable to store ratio homography
    float rh{};

    for (int i = ref_frame_idx + 1;  i <= ref_frame_idx + INIT_SCAN_WINDOW; i++){

        // extract features for frame i
        extract_features(frames[i],type);

        // get the matched features from frame i and frame 0
        matches = match_frames(frames[ref_frame_idx],frames[i],type);

        // clear points
        pts1.clear();
        pts2.clear();

        // get the matched points between frame i and frame 0
        get_matched_points(frames[ref_frame_idx], frames[i], matches, pts1, pts2);

        // compute ratio homography for frame i and frame 0
        rh = compute_rh(pts1, pts2);
        
        // return first pair below threshold
        if (rh < INIT_RH_THRESHOLD){
            return InitialPair {frames[i].get_id(), matches};
        }
    }   
    // no pair found, throw error
    throw std::runtime_error("No frame found with ratio homography < " + std::to_string(INIT_RH_THRESHOLD) + " w.r.t frame " + std::to_string(frames[ref_frame_idx].get_id()) );
}


// function to get the recovered initial pose
bool recover_initial_pose(const std::vector<cv::Point2f>& pts1,
                          const std::vector<cv::Point2f>& pts2,
                          const cv::Mat& K,
                          cv::Mat& R,
                          cv::Mat& t,
                          cv::Mat& inlier_mask
                        ){

    // get the essential matrix using frame where rh < INIT_RH_THRESHOLD
    cv::Mat E;
    E = cv::findEssentialMat(pts1, pts2, K, cv::RANSAC, 0.99, 3.0, cv::noArray());

    // recoverPose (get R,t) using E, pts1, pts2, and return true if inliers > INIT_E_INLIER_COUNT
    return cv::recoverPose(E, pts1, pts2, K, R, t,inlier_mask) > INIT_E_INLIER_COUNT;
}

// filtering inliers selected from computation of Essential matrix
InlierData filter_inliers(
    const std::vector<cv::Point2f>& pts1, 
    const std::vector<cv::Point2f>& pts2,
    const std::vector<cv::DMatch>& matches,
    const cv::Mat& inlier_mask
){
    std::vector<cv::Point2f> pts3;
    std::vector<cv::Point2f> pts4;
    std::vector<cv::DMatch> inlier_matches;
    for (int i = 0; i < inlier_mask.rows; i++){
        if (inlier_mask.at<uchar>(i)){
            pts3.push_back(pts1[i]);
            pts4.push_back(pts2[i]);
            inlier_matches.push_back(matches[i]);
        }
    }
    return InlierData{pts3, pts4, inlier_matches};
}