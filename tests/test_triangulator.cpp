/**
 * @file test_triangulator.cpp
 * @brief Test executable verifying triangulation of inlier point correspondences
 *        into 3D landmarks using the recovered initial pose.
 * @author Rudraksha R. Bandodkar
 * @copyright MIT License, see LICENSE
 */

#include "triangulator.hpp"

#include <iostream>
#include "calib_io.hpp"
#include "camera.hpp"
#include "frame.hpp"
#include "frame_io.hpp"
#include "features.hpp"
#include "initializer.hpp"
#include "landmark.hpp"
#include "visualizer.hpp"
#include "pose.hpp"

// constants
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
    
    // frame 0 params
    int frame_idx_1 = 0; 
    cv::Mat R1 = cv::Mat::eye(3,3, CV_64F);
    cv::Mat t1 = (cv::Mat_<double>(3,1) << 0.0,0.0,0.0); 

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
        // filter inlier data based on points which satisfy geometric constraints w.r.t Essential Matrix computed
        InlierData inlier_data = filter_inliers(pts1, pts2, ip.matches, inlier_mask);
        
        // triangulate points and get Landmarks vector
        std::vector<Landmark> landmarks;
        triangulate_all(
            frame_idx_1,
            ip.frame_idx,
            inlier_data,
            R1,t1,
            R,t,
            cam.get_K(),
            landmarks
        );

        std::cout << "Triangulated landmarks: " << landmarks.size() << std::endl;
        std::cout << "First point: " << landmarks[0].get_position() << std::endl;   

        // create camera poses vector
        std::vector<Pose> poses{
            {R1,t1},
            {R,t}
        };

        // visualize landmarks
        visualize_landmarks(landmarks, frames,poses);
    }   
    else{
        int inlier_count = cv::countNonZero(inlier_mask);
        std::cerr << "Inlier count / Inlier count threshold: "
                  << inlier_count << " / " << INIT_E_INLIER_COUNT
                  <<".\nInsufficient inlier count to recover R, t"<<std::endl;
    }
    return 0;
}