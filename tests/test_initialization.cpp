/**
 * @file test_initialization.cpp
 * @brief Test executable verifying initial pair selection and pose recovery.
 *
 * @author Rudraksha R. Bandodkar
 * @copyright Copyright (c) 2026 Rudraksha R. Bandodkar. MIT License, see
 *            LICENSE file in the project root for full terms.
 */

#include "initializer.hpp"

#include <iostream>
#include "calib_io.hpp"
#include "camera.hpp"
#include "frame.hpp"
#include "frame_io.hpp"
#include "features.hpp"
#include "constants.hpp"


// calib.txt
const std::string CALIB_PATH = "../data/calib.txt";
// frames_3
const std::string FRAMES_PATH = "../data/frames_3";


int main(){
    // read calib.txt and create camera object
    // Camera cam = read_calib(path) works because read_calib returns a Camera by value.
    Camera cam = read_calib(CALIB_PATH);

    // load frames
    std::vector<Frame> frames = load_frames(FRAMES_PATH);

    // select initial pair
    InitialPair ip = select_initial_pair(frames, FeatureType::SIFT);

    std::cout << "Image pair frame-id: " << ip.frame_idx << std::endl;

    // get matched points from selected pair
    std::vector<cv::Point2f> pts1;
    std::vector<cv::Point2f> pts2;
    get_matched_points(frames[0], frames[ip.frame_idx], ip.matches, pts1, pts2);

    // recover pose to get R, t
    cv::Mat R,t, inlier_mask;
    if (recover_initial_pose(pts1, pts2,cam.get_K(), R, t, inlier_mask)){
        std::cout << "Recovered Pose:"
                  << "\nR:\n"<< R
                  << "\nt:\t"<< t <<std::endl; 
    }
    else{
        int inlier_count = cv::countNonZero(inlier_mask);
        std::cerr << "Inlier count / Inlier count threshold: "
                  << inlier_count << " / " << INIT_E_INLIER_COUNT
                  <<".\nInsufficient inlier count to recover R, t"<<std::endl;
    }
    return 0;
}