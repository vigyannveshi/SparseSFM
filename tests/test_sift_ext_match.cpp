/**
 * @file test_sift_ext_match.cpp
 * @brief Test executable verifying SIFT extraction and matching across frame pairs.
 *
 * @author Rudraksha R. Bandodkar
 * @copyright Copyright (c) 2026 Rudraksha R. Bandodkar. MIT License, see
 *            LICENSE file in the project root for full terms.
 */

#include "sift_extractor.hpp"
#include "sift_matcher.hpp"

#include <iostream>
#include "frame.hpp"
#include "frame_io.hpp"
#include "constants.hpp"

// CONSTANTS
const std::string FRAMES_DIR = "../data/frames_3"; 

int main(){
    // loading frames
    std::vector<Frame> frames = load_frames(FRAMES_DIR);

    // ensure frames are not empty
    if (frames.empty()){
        std::cerr<<"Error: frames directory: "<< FRAMES_DIR <<" is empty.";
        return 1;
    }
    
    // create sift-detector
    cv::Ptr<cv::SIFT> sift = cv::SIFT::create(SIFT_NUM_FEATURES);

    // extract SIFT features for each frame
    for (auto& frame : frames){
        extract_sift(frame, sift);
    }

    // match frames on consecutive pairs & visualize
    for (int i = 0; i<frames.size()-1; i++)
    {   
        // create images to visualize
        cv::Mat img1 = cv::imread(frames[i].get_image_path());
        cv::Mat img2 = cv::imread(frames[i+1].get_image_path());

        // draw matches between image-pairs
        cv::Mat output;
        cv::drawMatches(
            img1, frames[i].get_keypoints(),
            img2, frames[i+1].get_keypoints(),
            match_frames_sift(frames.at(i),frames.at(i+1)), // getting matches per-pair
            output);

        cv::Mat resized;
        cv::resize(output, resized, cv::Size(), VIZ_SCALE, VIZ_SCALE);

        cv::imshow("Matches", resized);

        // tracking key-presses to change frames visualized
        int key = cv::waitKey(0);
        if (key == 27) break; // 27 is ESC
    }

    return 0;
}