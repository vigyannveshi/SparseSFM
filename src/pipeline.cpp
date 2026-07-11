/**
 * @file pipeline.cpp
 * @brief Fine-grained pipeline steps for sparse SfM
 * @author Rudraksha R. Bandodkar
 * @copyright MIT License, see LICENSE
 */

#include "pipeline.hpp"

#include "pose.hpp"
#include "p3p_solver.hpp"
#include "constants.hpp"
#include <iostream>
#include <set>

constexpr bool VERBOSE = true; 

bool initialize_pipeline(
    const cv::Mat&                              K,
    const FeatureType&                          feature_type,
    std::vector<Frame>&                         frames,
    const int                                   reference_frame_idx,
    int&                                        selected_frame_idx,
    std::vector<Landmark>&                      landmarks,
    std::map<std::pair<int,int>, int>&          frame_keypoint2landmark_map
){
    // reference alias
    std::map<std::pair<int,int>, int>& fkp2lm = frame_keypoint2landmark_map;

    // guard: need at least 2 frames
    if (frames.size()<2){
        return false;
    }
    
    // set reference frame pose to identity
    cv::Mat R0 = cv::Mat::eye(3,3, CV_64F);
    cv::Mat t0 = (cv::Mat_<double>(3,1) << 0.0,0.0,0.0); 
    frames[reference_frame_idx].set_pose(R0,t0);

    // select_initial_pair -> InitialPair ip
    InitialPair ip = select_initial_pair(frames, feature_type, reference_frame_idx);
    selected_frame_idx = ip.frame_idx;

    // get_matched_points -> pts1, pts2
    std::vector<cv::Point2f> pts1;
    std::vector<cv::Point2f> pts2;
    get_matched_points(frames[reference_frame_idx], frames[ip.frame_idx], ip.matches, pts1, pts2);

    // recover_initial_pose -> R, t, inlier_mask
    // on failure: return false
    cv::Mat R,t, inlier_mask;

    if (!recover_initial_pose(pts1, pts2, K, R, t, inlier_mask)){
        if (VERBOSE){
            int inlier_count = cv::countNonZero(inlier_mask);
            std::cerr << "Inlier count / Inlier count threshold: "
                      << inlier_count << " / " << INIT_E_INLIER_COUNT
                      <<".\nInsufficient inlier count to recover R, t"<<std::endl;
        }
        return false;
    }

    // set pose on frames[selected_frame_idx]
    frames[selected_frame_idx].set_pose(R,t);

    // filter_inliers -> inlier_data
    InlierData inlier_data = filter_inliers(pts1, pts2, ip.matches, inlier_mask);


    // triangulate_all -> appends into landmarks
    triangulate_all(reference_frame_idx, selected_frame_idx, inlier_data, 
                    frames[reference_frame_idx].get_R(), frames[reference_frame_idx].get_t(),
                    frames[selected_frame_idx].get_R(), frames[selected_frame_idx].get_t(),
                    K, landmarks
                );

    // build fkp2lm from all landmarks
    for (int lm_idx = 0; lm_idx < static_cast<int>(landmarks.size()); lm_idx++){
        for (const Observation& obs : landmarks[lm_idx].get_observations()){
            fkp2lm[{obs.frame_idx,obs.keypoint_idx}] = lm_idx;
        }
    }
    return true;
}


bool register_frame(
    std::vector<Frame>&                         frames,
    const int                                   i,
    std::vector<Landmark>&                      landmarks,
    std::map<std::pair<int,int>, int>&          frame_keypoint2landmark_map,
    const cv::Mat&                              K,
    const FeatureType&                          feature_type
){
    // reference alias
    std::map<std::pair<int,int>, int>& fkp2lm = frame_keypoint2landmark_map;

    // guard: skip if frames[i] already has pose
    if (frames[i].get_has_pose()) return false; 

    // guard: check if any frame in window [i-1, i-MATCH_WINDOW_SIZE] has pose
    //        if none found: return false
    bool has_pose = false;
    for (int j = i-1; j >= std::max(0, i - MATCH_WINDOW_SIZE); j--){
        if (frames[j].get_has_pose()){
            has_pose = true;
            break;
        }
    }
    if (!has_pose) return false;

    // STEP 1: extract_features(frames[i])
    extract_features(frames[i], feature_type);

    // STEP 2: match against window of previous frames
    //   pts2d, pts3d = empty
    std::vector<cv::Point2f> pts2d;
    std::vector<cv::Point3d> pts3d;

    //   used_keypoints_i = empty set
    std::set<int> used_keypoints_i;

    //   for j from i-1 down to max(0, i-MATCH_WINDOW_SIZE):
    for(int j = i-1; j >= std::max(0, i - MATCH_WINDOW_SIZE); j--){
        // if frames[j] has no pose: continue
        if (!frames[j].get_has_pose()) continue;

        // matches_j = match_frames(frames[i], frames[j])
        std::vector<cv::DMatch> matches_j = match_frames(frames[j],frames[i], feature_type);

        // for each match in matches_j:
        for (const cv::DMatch& match: matches_j){
            // frame[j] -> query, frame[i] -> train_idx
            std::pair<int,int> key(frames[j].get_id(), match.queryIdx);

            if (fkp2lm.count(key) && !used_keypoints_i.count(match.trainIdx)){
                pts3d.push_back(landmarks[fkp2lm[key]].get_position());
                pts2d.push_back(frames[i].get_keypoints()[match.trainIdx].pt);
                used_keypoints_i.insert(match.trainIdx);
            }
        }
    }

    if (VERBOSE){
    std::cout << "\nFound " << pts2d.size() << " 2D-3D correspondences." << std::endl;
    }
        
    // STEP 3: ransac_pnp -> pose, on failure return false
    Pose pose;
    std::vector<bool> pnp_inlier_mask;

    if (!ransac_pnp(pts2d, pts3d, K, pose, pnp_inlier_mask)){
        if (VERBOSE){

            std::cerr << "Pose estimation failed: Insufficient inliers: "
            << std::count(pnp_inlier_mask.begin(), pnp_inlier_mask.end(), true) << " < " << P3P_MIN_INLIERS <<std::endl;
        }
        return false;
    }

    // set pose on frames[i]
    frames[i].set_pose(pose.R, pose.t);

    // STEP 4: find triangulation partner, with pose and collect new matches for triangulation 
    int tri_frame_idx = -1;
    for (int j = i-1; j >= std::max(0, i-MATCH_WINDOW_SIZE); j--){
        if (frames[j].get_has_pose()){
            tri_frame_idx = j;
            break;
        }
    }

    // note the landmark count before triangulation, since we will be adding only the new landmarks to the fr_kp2lm map.
    int lm_count_before = landmarks.size();

    // if tri_frame_idx == -1: skip triangulation, go to STEP 6
    if (tri_frame_idx != -1){

        InlierData pnp_inlier_data;
        std::vector<cv::DMatch> matches = match_frames(frames[tri_frame_idx], frames[i], feature_type);

        for (const cv::DMatch& match: matches){
            std::pair<int, int> key (frames[tri_frame_idx].get_id(), match.queryIdx);
            if (!fkp2lm.count(key)){
                pnp_inlier_data.pts1.push_back(frames[tri_frame_idx].get_keypoints()[match.queryIdx].pt);
                pnp_inlier_data.pts2.push_back(frames[i].get_keypoints()[match.trainIdx].pt);
                pnp_inlier_data.inlier_matches.push_back(match);
            }
        }
            
        // STEP 5: triangulate_all -> appends into landmarks
        triangulate_all(frames[tri_frame_idx].get_id(), frames[i].get_id(), pnp_inlier_data,
                        frames[tri_frame_idx].get_R(), frames[tri_frame_idx].get_t(),
                        frames[i].get_R(), frames[i].get_t(),
                        K, landmarks
        );
    }

    // STEP 6: update fkp2lm with new landmarks
    for (int lm_idx = lm_count_before; lm_idx < static_cast<int>(landmarks.size()); lm_idx++)
    {
        for (const Observation& obs: landmarks[lm_idx].get_observations()){
            fkp2lm[{obs.frame_idx, obs.keypoint_idx}] = lm_idx;
        }
    }

    if (VERBOSE){
        std::cout << "Frame " << i << ": "
            << landmarks.size() - lm_count_before 
            << " new landmarks added. Total: "
            << landmarks.size() <<std::endl;
    }
    return true;
}


void cull_landmarks(
    std::vector<Landmark>&      landmarks,
    const std::vector<Frame>&   frames,
    const cv::Mat&              K,
    double                      threshold_px
){
    std::vector<Landmark> inlier_landmarks;

    for(const Landmark& lm: landmarks){
        bool is_inlier = true;

        for (const Observation& obs: lm.get_observations()){
            if (!frames[obs.frame_idx].get_has_pose()) continue;

            // calculate reprojection error

            // if the point is an outlier for even a single frame, then discard it
            cv::Point2f p_obs = frames[obs.frame_idx].get_keypoints()[obs.keypoint_idx].pt;

            if (reprojection_error({frames[obs.frame_idx].get_R(),frames[obs.frame_idx].get_t()},
                                lm.get_position(), p_obs, K                    
            ) > threshold_px){
                is_inlier = false;
                break;
            }
        }
        if (is_inlier) inlier_landmarks.push_back(lm);
    }

    if (VERBOSE){
        std::cout << "Landmarks after culling: "<< inlier_landmarks.size()
                  <<" / "<<landmarks.size()<<std::endl;
    }
    landmarks = inlier_landmarks;
}


void rebuild_fkp2lm(
    const std::vector<Landmark>&        landmarks,
    std::map<std::pair<int,int>, int>&  frame_keypoint2landmark_map
){
    frame_keypoint2landmark_map.clear();
    for (int lm_idx = 0; lm_idx < static_cast<int>(landmarks.size()); lm_idx++){
        for (const Observation& obs : landmarks[lm_idx].get_observations()){
            frame_keypoint2landmark_map[{obs.frame_idx, obs.keypoint_idx}] = lm_idx;
        }
    }
}


void normalize_scale(
    std::vector<Frame>&     frames,
    std::vector<Landmark>&  landmarks,
    const int               ref_frame_idx,
    const int               selected_frame_idx
){
    // compute camera centers of fixed frames
    cv::Mat R0 = frames[ref_frame_idx].get_R();
    cv::Mat t0 = frames[ref_frame_idx].get_t();
    cv::Mat R_sel = frames[selected_frame_idx].get_R();
    cv::Mat t_sel = frames[selected_frame_idx].get_t();

    cv::Mat c0 = -R0.t() * t0;  // camera center of ref frame (to orient camera to world/camera-frame 0)
    cv::Mat c5 = -R_sel.t() * t_sel;  // camera center of selected frame (to orient camera to world/camera-frame 0)

    // compute current baseline
    double baseline = cv::norm(c0 - c5);
    if (baseline < 1e-6) return;  // degenerate

    double scale = 1.0 / baseline;

    // scale all camera translations
    for (Frame& f : frames){
        if (!f.get_has_pose()) continue;
        f.set_pose(f.get_R(), f.get_t() * scale);
    }

    // scale all landmark positions
    for (Landmark& lm : landmarks){
        cv::Point3d p = lm.get_position();
        lm.set_position({p.x * scale, p.y * scale, p.z * scale});
    }
}