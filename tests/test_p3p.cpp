/**
 * @file test_p3p.cpp
 * @brief Test for P3P pose estimation and RANSAC driver
 * @author Rudraksha R. Bandodkar
 * @copyright MIT License, see LICENSE
 */

#include "p3p_solver.hpp"

#include <iostream>
#include <map>
#include "calib_io.hpp"
#include "camera.hpp"
#include "frame.hpp"
#include "frame_io.hpp"
#include "features.hpp"
#include "initializer.hpp"
#include "landmark.hpp"
#include "visualizer.hpp"
#include "pose.hpp"
#include "triangulator.hpp"

// constants
#include "constants.hpp"

// calib.txt
const std::string CALIB_PATH = "../data/calib.txt";
// frames_3
const std::string FRAMES_PATH = "../data/frames_3";

// FEATURE_TYPE
FeatureType feature_type = FeatureType::SIFT;


// function to initialize
bool initialize(std::vector<Frame>& frames, 
                const cv::Mat& K,
                const int& ref_frame_idx,
                int& selected_frame_idx, 
                InlierData& inlier_data){

    // return false if number of frames <2
    if (frames.size()<2){
        return false;
    }
    
    // setting reference frame pose
    cv::Mat R0 = cv::Mat::eye(3,3, CV_64F);
    cv::Mat t0 = (cv::Mat_<double>(3,1) << 0.0,0.0,0.0); 
    frames[ref_frame_idx].set_pose(R0,t0);

    // select initial pair
    InitialPair ip = select_initial_pair(frames, feature_type, ref_frame_idx);
    std::cout << "Image pair frame-id: " << ip.frame_idx << std::endl;
    selected_frame_idx = ip.frame_idx;
    
    // get matched points from selected pair
    std::vector<cv::Point2f> pts1;
    std::vector<cv::Point2f> pts2;
    get_matched_points(frames[ref_frame_idx], frames[ip.frame_idx], ip.matches, pts1, pts2);

    // recover initialize pose
    cv::Mat R,t, inlier_mask;
    if (recover_initial_pose(pts1, pts2, K, R, t, inlier_mask)){
        frames[selected_frame_idx].set_pose(R,t);
        // filter points to return points & patches in the inlier mask
        inlier_data = filter_inliers(pts1, pts2, ip.matches, inlier_mask);
        return true;
    }
    else{
        int inlier_count = cv::countNonZero(inlier_mask);
        std::cerr << "Inlier count / Inlier count threshold: "
                  << inlier_count << " / " << INIT_E_INLIER_COUNT
                  <<".\nInsufficient inlier count to recover R, t"<<std::endl;
        return false;
    }
}


int main(){
    // read calib.txt and create camera object
    // Camera cam = read_calib(path) works because read_calib returns a Camera by value.
    Camera cam = read_calib(CALIB_PATH);
    
    // load frames
    std::vector<Frame> frames = load_frames(FRAMES_PATH);
    
    // initialize
    bool is_initialized = false;
    int ref_frame_idx = 0;
    int selected_frame_idx;
    InlierData inlier_data;

    while (!is_initialized && ref_frame_idx < MAX_INITIALIZATION_TRIALS){
        std::cout<< "Attempting intialization, trial: " + std::to_string(ref_frame_idx+1) + " / " + 
                    std::to_string(MAX_INITIALIZATION_TRIALS)<<". \n";
        // to catch runtime-error of initialization pair not found, shift reference frame
        try{
            if(initialize(frames, cam.get_K(), ref_frame_idx, selected_frame_idx, inlier_data)){
                is_initialized = true;
            }
            else{
                // insufficient inliers, shift reference frame
                std::cout << "Insufficient inliers, shifting reference frame"<<std::endl;
                ref_frame_idx ++;
            }
        }
        catch (const std::runtime_error& e){
            std::cerr <<e.what() <<std::endl;
            ref_frame_idx ++;
        }
    }

    if (!is_initialized){
        throw std::runtime_error("Initialization failed after "+
                                 std::to_string(MAX_INITIALIZATION_TRIALS) + 
                                 " trials"                      
        );
    }

    // vector to store landmarks
    std::vector<Landmark> landmarks;

    // hash map: (frame_idx, keypoint_idx) ---> lm_idx 
    std::map<std::pair<int,int>, int> fr_kp2lm;

    // triangulate using poses obtained after initialization
    triangulate_all(ref_frame_idx, selected_frame_idx, inlier_data, 
                    frames[ref_frame_idx].get_R(), frames[ref_frame_idx].get_t(),
                    frames[selected_frame_idx].get_R(), frames[selected_frame_idx].get_t(),
                    cam.get_K(), landmarks
                );

    // building fr_kp2lm
    for (int lm_idx = 0; lm_idx < static_cast<int>(landmarks.size()); lm_idx++){
        for (const Observation& obs : landmarks[lm_idx].get_observations()){
            fr_kp2lm[{obs.frame_idx,obs.keypoint_idx}] = lm_idx;
        }
    }

    std::cout<<"Initial landmarks: " << landmarks.size()<<std::endl; 
    std::cout<<"fr_kp2lm entries: " << fr_kp2lm.size()<<std::endl; 
    std::cout<<"\n------------Initialization & First Triangulation Complete------------\n"<<std::endl;

    std::cout<<"\n------------Starting Incremental PNP------------\n"<<std::endl;

    // corresponding points
    std::vector<cv::Point2f> pts2d;
    std::vector<cv::Point3d> pts3d;

    // incremental pnp (ransac_p3p)    
    for (int i = ref_frame_idx + 1 ; i < static_cast<int>(frames.size()); i++){
        // clear corresponding points
        pts2d.clear();
        pts3d.clear();

        // if previous frame doesn't have pose
        if (!frames[i-1].get_has_pose()) continue;

        // no-need to update for selected_frame_idx
        if (i == selected_frame_idx) continue;
        
        /* STEP 1. Extract features & match them*/

        // extract features
        extract_features(frames[i], feature_type);

        // match frame[i-1]  against frame[i] 

        std::vector<cv::DMatch> matches = match_frames(frames[i-1], frames[i], feature_type);
        
        /* STEP 2. Get 2D-3D correspondances*/
        for(cv::DMatch& match : matches){
            // frame[i] -> query image, frame[i-1] -> train image
            std::pair<int, int> key = {frames[i-1].get_id(), match.queryIdx};
            if (fr_kp2lm.count(key)){
                pts3d.push_back(landmarks[fr_kp2lm[key]].get_position());
                pts2d.push_back(frames[i].get_keypoints()[match.trainIdx].pt);
            }
        }
        std::cout << "Found " << pts2d.size() <<" 2D-3D correspondences."<< std::endl; 

        /* STEP 3. Run RANSAC-PNP to get pose*/
        Pose pose;
        std::vector<bool> pnp_inlier_mask;

        /* Uncomment and replace below code to test opencv_pnp */
        // if (!opencv_pnp(pts2d, pts3d, cam.get_K(), pose, pnp_inlier_mask)){
        if (!ransac_pnp(pts2d, pts3d, cam.get_K(), pose, pnp_inlier_mask)){
            std::cerr << "Pose estimation failed: Insufficient inliers: "
                      << std::count(pnp_inlier_mask.begin(), pnp_inlier_mask.end(), true) << " < " << P3P_MIN_INLIERS <<std::endl;
            continue;
        }
        frames[i].set_pose(pose.R, pose.t);

        /* STEP 4. Collect only those matches which don't have a landmark*/
        InlierData pnp_inlier_data;
        for (const cv::DMatch& match: matches){
            std::pair<int,int> key = {frames[i-1].get_id(), match.queryIdx};
            if (!fr_kp2lm.count(key)){
                pnp_inlier_data.pts1.push_back(frames[i-1].get_keypoints()[match.queryIdx].pt);
                pnp_inlier_data.pts2.push_back(frames[i].get_keypoints()[match.trainIdx].pt);
                pnp_inlier_data.inlier_matches.push_back(match);
            }
        }

        // note the landmark count before triangulation, since we will be adding only the new landmarks to the fr_kp2lm map.
        int lm_count_before = landmarks.size();

        /* STEP 5. Triangulate*/
        triangulate_all(frames[i-1].get_id(), frames[i].get_id(), pnp_inlier_data,
                        frames[i-1].get_R(), frames[i-1].get_t(),
                        frames[i].get_R(), frames[i].get_t(),
                        cam.get_K(), landmarks
        );

        /* STEP 6. Update fr_kp2lm with new landmarks added*/
        for (int lm_idx = lm_count_before; lm_idx < static_cast<int>(landmarks.size()); lm_idx++)
        {
            for (const Observation& obs: landmarks[lm_idx].get_observations()){
                fr_kp2lm[{obs.frame_idx, obs.keypoint_idx}] = lm_idx;
            }
        }

        std::cout << "Frame " << i << ": "
                  << landmarks.size() - lm_count_before 
                  << " new landmarks added. Total: "
                  << landmarks.size() <<std::endl;
        
    }

    // collect poses for visualization
    std::vector<Pose> poses;

    for (const Frame& f:frames){
        if (f.get_has_pose()){
            poses.push_back({f.get_R(), f.get_t()});
        }
    }

    // visualize 
    visualize_landmarks(landmarks, frames, poses);
    
    return 0;
}


// testing opencv-pnp (additional)
bool opencv_pnp(
    const std::vector<cv::Point2f>& pts2d,
    const std::vector<cv::Point3d>& pts3d,
    const cv::Mat& K,
    Pose& pose_out,
    std::vector<bool>& inlier_mask
){
    if ((int)pts2d.size() < 4) return false;

    // convert Point3d to Point3f for OpenCV
    std::vector<cv::Point3f> pts3f;
    for (const auto& p : pts3d)
        pts3f.push_back(cv::Point3f(p.x, p.y, p.z));

    cv::Mat rvec, tvec, inliers;
    bool success = cv::solvePnPRansac(
        pts3f, pts2d, K, cv::noArray(),
        rvec, tvec, false,
        100, 8.0, 0.99, inliers,
        cv::SOLVEPNP_P3P
    );

    if (!success) return false;

    // convert rvec to R
    cv::Mat R;
    cv::Rodrigues(rvec, R);
    pose_out = {R, tvec};

    // build inlier mask
    inlier_mask.assign(pts2d.size(), false);
    for (int i = 0; i < inliers.rows; i++)
        inlier_mask[inliers.at<int>(i)] = true;

    return true;
}