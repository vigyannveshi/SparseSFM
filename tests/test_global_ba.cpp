/**
 * @file test_global_ba.cpp
 * @brief Global Bundle-Adjustment run after first registration
 * @author Rudraksha R. Bandodkar
 * @copyright MIT License, see LICENSE
 */

#include "calib_io.hpp"
#include "features.hpp"
#include "frame_io.hpp"
#include "frame.hpp"
#include "landmark.hpp"
#include "pose.hpp"
#include "pipeline.hpp"
#include "bundle_adjustments.hpp"
#include "visualizer.hpp"
#include "constants.hpp"

#include <iostream>
#include <map>

// constants
const std::string CALIB_PATH = "../data/calib.txt";
const std::string FRAMES_PATH = "../data/frames_3";
constexpr FeatureType feature_type = FeatureType::SIFT;

int main(){
    // read_calib, load_frames
    Camera cam = read_calib(CALIB_PATH);
    std::vector<Frame> frames = load_frames(FRAMES_PATH);

    // create vector for landmarks, and map for frame, keypoint -> landmark
    std::vector<Landmark> landmarks;
    std::map<std::pair<int,int>, int> frame_keypoint2landmark_map;

    // initialize
    bool is_initialized = false;
    int ref_frame_idx = 0;
    int selected_frame_idx;

    while (!is_initialized && ref_frame_idx < MAX_INITIALIZATION_TRIALS){
        std::cout<< "Attempting intialization, trial: " + std::to_string(ref_frame_idx+1) + " / " + 
                    std::to_string(MAX_INITIALIZATION_TRIALS)<<". \n";
        // to catch runtime-error of initialization pair not found, shift reference frame
        try{
            if(initialize_pipeline(cam.get_K(), feature_type,
                                   frames, ref_frame_idx, selected_frame_idx,
                                   landmarks, frame_keypoint2landmark_map)){
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

    // if not initialized: throw
    if (!is_initialized){
        throw std::runtime_error("Initialization failed after "+
                                 std::to_string(MAX_INITIALIZATION_TRIALS) + 
                                 " trials"                      
        );
    }

    std::cout<<"\n------------ SFM INITIALIZED------------\n"<<std::endl;

    
    // for i from ref_frame_idx+1 to frames.size()-1:
    //   register_frame(...)
    std::cout<<"\n------------Starting Incremental PNP------------\n"<<std::endl;
    
    for (int i = ref_frame_idx+1; i < static_cast<int>(frames.size()); i++){
        register_frame(
            frames, i, 
            landmarks, frame_keypoint2landmark_map,
            cam.get_K(), feature_type
        );
    }
    
    
    // collect poses from frames that have_pose for visualization
    std::vector<Pose> poses;
    
    for (const Frame& f:frames){
        if (f.get_has_pose()){
            poses.push_back({f.get_R(), f.get_t()});
        }
    }
    
    // visualize_landmarks
    std::cout<<"\n------------Visualization before BA------------\n"<<std::endl;
    visualize_landmarks(landmarks, frames, poses, "Before BA");

    pangolin::DestroyWindow("Before BA");


    // remove landmarks with just single observation
    landmarks.erase(
    std::remove_if(landmarks.begin(), landmarks.end(),
        [](const Landmark& lm){ return lm.get_observations().size() < 2; }),
    landmarks.end()
    );
    
    std::cout<<"Landmarks count, after eliminating landmarks with single observation: "<<landmarks.size()<<std::endl;

    double total_err = 0.0;
    int n_obs = 0;
    for (const Landmark& lm : landmarks){
        for (const Observation& obs : lm.get_observations()){
            if (!frames[obs.frame_idx].get_has_pose()) continue;
            total_err += reprojection_error(
                {frames[obs.frame_idx].get_R(), frames[obs.frame_idx].get_t()},
                lm.get_position(),
                frames[obs.frame_idx].get_keypoints()[obs.keypoint_idx].pt,
                cam.get_K()
            );
            n_obs++;
        }
    }
    
    std::cout << "Mean reprojection error before BA: " << total_err/n_obs << " px" << std::endl;
    
    std::cout << "\n------------ Bundle Adjustment ------------\n";
    bundle_adjust(frames, landmarks, cam.get_K(), 0, -1, BA_GLOBAL_ITERATIONS, {ref_frame_idx, selected_frame_idx});

    // cull landmarks
    cull_landmarks(landmarks, frames, cam.get_K());

    
    poses.clear();
    for (const Frame& f:frames){
        if (f.get_has_pose()){
            poses.push_back({f.get_R(), f.get_t()});
        }
    }

    // visualize_landmarks
    std::cout<<"\n------------Visualization after BA------------\n"<<std::endl;
    visualize_landmarks(landmarks, frames, poses, "After BA");

    return 0;
}
