/**
 * @file features.cpp
 * @brief Implementation of unified feature extraction and matching for ORB and SIFT.
 * @author Rudraksha R. Bandodkar
 * @copyright MIT License, see LICENSE
 */

 
#include "features.hpp"

// constants
#include "constants.hpp"

// feature extractors & matchers
#include "orb_extractor.hpp"
#include "orb_matcher.hpp"
#include "sift_extractor.hpp"
#include "sift_matcher.hpp"


// extract features
void extract_features(Frame& frame, const FeatureType type){
    switch (type)
    {
        case FeatureType::ORB:{
            cv::Ptr<cv::ORB> orb = cv::ORB::create(ORB_NUM_FEATURES);
            extract_orb(frame, orb);
            return;
        }
        
        case FeatureType::SIFT: {
            cv::Ptr<cv::SIFT> sift = cv::SIFT::create(SIFT_NUM_FEATURES,SIFT_N_OCTAVES, 
                SIFT_CONTRAST_THRESHOLD, SIFT_EDGE_THRESHOLD, SIFT_SIGMA);
                extract_sift(frame, sift);
                return; 
            }
        default:
            throw std::runtime_error("Unknown FeatureType");
    }
}

// match frames
std::vector<cv::DMatch> match_frames(const Frame& frame1, const Frame& frame2, const FeatureType type){
    switch (type)
    {
        case FeatureType::ORB:
            return match_frames_orb(frame1, frame2);
            
        case FeatureType::SIFT:
            return match_frames_sift(frame1, frame2);

        default:
            throw std::runtime_error("Unknown FeatureType");
    }
}