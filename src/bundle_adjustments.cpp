/**
 * @file bundle_adjustment.cpp
 * @brief Bundle adjustment via Levenberg-Marquardt with Schur complement
 * @author Rudraksha R. Bandodkar
 * @copyright MIT License, see LICENSE
 */

#include "bundle_adjustments.hpp"
#include <iostream>
#include <algorithm>

constexpr bool VISUALIZE_MAT = true;
constexpr int VIS_MAT_ITER = 5;
constexpr bool VERBOSE = true;

// -----------------------------------------------------------------------
// Internal helpers
// -----------------------------------------------------------------------


// Project world point P using pose (R,t) and intrinsics K
// returns 2D pixel coordinates
static cv::Point2f project(
    const cv::Mat& R,
    const cv::Mat& t,
    const cv::Mat& K,
    const cv::Point3d& P
){
    // P_c = R*P + t (apply_pose function from pose)
    // transform to cv::Mat
    cv::Mat Pk = point3DtoMat(ref_to_camera_frame(P, {R,t}));

    // Projection as homogeneous transformation
    cv::Mat Pc = K*Pk;
    return cv::Point2f(static_cast<float>(Pc.at<double>(0)/Pc.at<double>(2)), 
                       static_cast<float>(Pc.at<double>(1)/Pc.at<double>(2))
                      );    
}


// Compute 2x3 Jacobian of projection w.r.t Landmark L
static cv::Mat compute_jacobian_landmark(
        const cv::Mat& R,
        const cv::Mat& t,
        const cv::Mat& K,
        const cv::Point3d& L
){
    // Pk = R*P + t -> extract Xk, Yk, Zk (K-> camera frame (before projection))
    cv::Point3d Lk = ref_to_camera_frame(L, {R,t});
    double Xk = Lk.x;
    double Yk = Lk.y;
    double Zk = Lk.z;

    // extract f_x, f_y, s from K
    double fx = K.at<double>(0,0);
    double fy = K.at<double>(1,1);
    double s  = K.at<double>(0,1);

    // build 2x3 matrix:
    // | f_x/Zk   s/Zk   -(f_x*Xk + s*Yk)/Zk² |
    // |   0    f_y/Zk      -f_y*Y/Zk²     |
    // multiply on right by R
    cv::Mat J_l = (cv::Mat_<double>(2,3) <<
        fx/Zk, s/Zk, -(fx*Xk + s*Yk)/(Zk*Zk),
        0.0,  fy/Zk, -(fy*Yk)/(Zk*Zk)
    )*R;

    // return 2x3 cv::Mat CV_64F
    return J_l;
}


// Compute 2x6 Jacobian of projection w.r.t pose perturbation in SE(3)
static cv::Mat compute_jacobian_pose(
    const cv::Mat& R,
    const cv::Mat& t,
    const cv::Mat& K,
    const cv::Point3d& L // landmark point
){
    // Pk = R*P + t -> extract Xk, Yk, Zk (K-> camera frame (before projection))
    cv::Point3d Lk = ref_to_camera_frame(L, {R,t});
    double Xk = Lk.x;
    double Yk = Lk.y;
    double Zk = Lk.z;

    // extract f_x, f_y, s from K
    double fx = K.at<double>(0,0);
    double fy = K.at<double>(1,1);
    double s  = K.at<double>(0,1);

    // build d_pi_dPk (2x3):
    // | f_x/Zk   s/Zk   -(f_x*Xk + s*Yk)/Zk² |
    // |   0    f_y/Zk      -f_y*Y/Zk²     |
    cv::Mat dpi_dPk = (cv::Mat_<double>(2,3) <<
        fx/Zk, s/Zk, -(fx*Xk + s*Yk)/(Zk*Zk),
        0.0,  fy/Zk, -(fy*Yk)/(Zk*Zk)
    );

    // build skew-symmetric matrix of R*L (3x3):
    // rP = R * L_mat  (L as 3x1 cv::Mat)
    // rP_skew = |  0    -rL_z   rL_y |
    //           | rL_z    0    -rL_x |
    //           |-rL_y   rL_x    0   |
    cv::Mat Lm = point3DtoMat(L); // Lm as 3x1
    cv::Mat rL = R * Lm;
    
    cv::Mat rL_skew = (cv::Mat_<double>(3,3) <<
     0,                 -rL.at<double>(2),  rL.at<double>(1),               
     rL.at<double>(2),   0,                -rL.at<double>(0),               
    -rL.at<double>(1),   rL.at<double>(0),  0               
    );


    // build dPk_dxi (3x6):
    cv::Mat I3 = cv::Mat::eye(3,3, CV_64F);
    cv::Mat dPk_dxi(3, 6, CV_64F);
    // | I   -rP_skew |
    // (first 3 cols = I, last 3 cols = -rL_skew)
    I3.copyTo(dPk_dxi.colRange(0,3));
    cv::Mat neg_rL_skew = -rL_skew;
    (neg_rL_skew).copyTo(dPk_dxi.colRange(3,6));

    // J_xi = d_pi_dPk * dPk_dxi  (2x6)
    return dpi_dPk*dPk_dxi;
}


// visualize a matrix as a gray-scale (sparsity pattern)
static void visualize_matrix(
    const cv::Mat& M,
    const std::string& name
){
    // log scale normalization
    cv::Mat log_M;
    cv::log(cv::abs(M)+1, log_M);

    // normalize M to [0, 255]
    // convert to CV_8U
    cv::Mat normalized;
    cv::normalize(log_M, normalized, 0, 255, cv::NORM_MINMAX, CV_8U);

    // rescaling size to fit the window
    double scale = static_cast<double>(MATRIX_MAX_DISPLAY / std::max(M.rows, M.cols));
    int display_w = static_cast<int>(M.cols*scale);
    int display_h = static_cast<int>(M.rows*scale);

    cv::Mat display;
    cv::resize(normalized, display, cv::Size(display_w, display_h), 0, 0, cv::INTER_LINEAR);
    
    // cv::imshow(name, normalized)
    cv::imshow(name, display);
    cv::waitKey(0);
}


// Compute all residuals and accumulate B, C, E, b_xi, b_L
static void compute_residuals_and_jacobians(
    const std::vector<Frame>&               frames,
    const std::vector<Landmark>&            landmarks,
    const cv::Mat&                          K,
    const std::vector<int>&                 pose_idx, // frame-id -> pose block index (-1 if no pose)
    // H, b
    cv::Mat&                                B,     // 6m x 6m, block diagonal
    cv::Mat&                                C,     // 3n x 3n, block diagonal
    cv::Mat&                                E,     // 6m x 3n
    cv::Mat&                                b_xi,  // 6m x 1
    cv::Mat&                                b_L,   // 3n x 1
    double&                                 total_reprojection_error,
    const int                               window_start, // first frame index to include
    const int                               window_end    // last frame index to include
){ 

    // 1. zero out all matrices
    //    B, C, E, b_xi, b_L all setTo(0)
    //    total_reprojection_error = 0.0
    B.setTo(0.0);
    C.setTo(0.0);
    E.setTo(0.0);
    b_xi.setTo(0.0);
    b_L.setTo(0.0);
    total_reprojection_error = 0.0;

    // 2. compute n_landmarks from pose_idx and landmarks
    int n_landmarks = landmarks.size();

    // 3. for each lm_idx from 0 to landmarks.size()-1:
    for(int lm_idx = 0; lm_idx < n_landmarks; lm_idx++){
        // get L = landmark.get_position()
        cv::Point3d L = landmarks[lm_idx].get_position();


        // for each obs in landmark.get_observations():
        for (const Observation& obs : landmarks[lm_idx].get_observations()){
            // 3a. skip if obs.frame_idx < window_start || obs.frame_idx > window_end
            if (obs.frame_idx < window_start || obs.frame_idx > window_end) continue;
            

            // 3b. get frame = frames[obs.frame_idx]
            Frame frame = frames[obs.frame_idx];
            //     skip if !frame.get_has_pose()
            // if (!frame.get_has_pose()) continue; // no need since , pi == -1 handles it
            

            /*At pose i, camera observes landmark j*/
            // 3c. pi = pose_idx[frame.get_id()]
            int pi = pose_idx[frame.get_id()]; // index of pose_i in B
            //     skip if pi == -1
            if (pi == -1) continue;
            //     lj = lm_idx
            int lj = lm_idx;


            // 3d. get R, t from frame
            cv::Mat R = frame.get_R();
            cv::Mat t = frame.get_t();

            //     get observed keypoint: p_obs = frame.get_keypoints()[obs.keypoint_idx].pt 
            //     2D pixel corresponding to landmark
            cv::Point2f p_obs = frame.get_keypoints()[obs.keypoint_idx].pt;


            // 3e. project L -> p_proj (cv::Point2f)
            cv::Point2f p_proj = project(R, t, K, L);
            //     build residual r (2x1 CV_64F):
            //     r = | p_proj.x - p_obs.x |
            //         | p_proj.y - p_obs.y |
            cv::Mat residual(2,1, CV_64F);
            residual.at<double>(0) = p_proj.x - p_obs.x;  
            residual.at<double>(1) = p_proj.y - p_obs.y;  
            //     total_reprojection_error += r.dot(r)
            total_reprojection_error += residual.dot(residual);


            // 3f. J_xi = compute_jacobian_pose(R, t, K, L)      // 2x6
            //     J_l  = compute_jacobian_landmark(R, t, K, L)  // 2x3
            cv::Mat J_xi = compute_jacobian_pose(R, t, K, L); 
            cv::Mat J_l  = compute_jacobian_landmark(R, t, K, L);
            

            /*
            cv::Rect(x, y, width, height) — this is OpenCV syntax for extracting a submatrix (ROI) from a matrix. x is the column offset, y is the row offset.
            */
            // 3g. get ROI references into B, C, E, b_xi, b_L, for pose i and landmark j
            // 3h. accumulate:
            cv::Mat C_block  = C(cv::Rect(lj*3, lj*3, 3, 3));
            cv::Mat bL_block  = b_L (cv::Rect(0, lj*3, 1, 3));
            //
            C_block  += J_l.t()  * J_l;
            bL_block  -= J_l.t()  * residual;
            
            // don't accumulate pose blocks for fixed-frames 
            // (frames-without pose are already excluded)
            if (pi>=0){
                cv::Mat B_block  = B(cv::Rect(pi*6, pi*6, 6, 6));
                cv::Mat E_block  = E(cv::Rect(lj*3, pi*6, 3, 6));
                cv::Mat bxi_block = b_xi(cv::Rect(0, pi*6, 1, 6));
                B_block  += J_xi.t() * J_xi;
                E_block  += J_xi.t() * J_l;
                bxi_block -= J_xi.t() * residual;
            }
        }
    }
}


// Apply Levenberg-Marquadt's damping (Marquadt's correction: damp by diagonal)
// other variant: B_block += lambda * I, C_block += lambda *I
static void apply_damping(
    cv::Mat& B,
    cv::Mat& C,
    const int n_poses,
    const int n_landmarks,
    const double lambda       
){
    /* 
    Note: cv::Mat::diag on a matrix returns a column vector, not a diagonal matrix
    */
    for(int pi = 0; pi<n_poses; pi++){
        for (int k = 0; k < 6; k++){
            B.at<double>(pi*6 + k, pi*6 + k) *= (1.0 + lambda);
        }
    }
    
    for(int lm_idx = 0; lm_idx<n_landmarks; lm_idx++){
        for (int k = 0; k < 3; k++){
            C.at<double>(lm_idx*3 + k, lm_idx*3 + k) *= (1.0 + lambda);
        }
    }
}


// Solve Reduced Camera System (RCS)[after Block Gauss Elimination] via Schlur Complement
// return delta_xi, delta_L
static bool solve_schur(
    const cv::Mat& B,
    const cv::Mat& C,
    const cv::Mat& E,
    const cv::Mat& b_xi,
    const cv::Mat& b_L,
    const int n_poses,
    const int n_landmarks,
    cv::Mat& delta_xi,
    cv::Mat& delta_L
){
    // build C_inv: invert each 3x3 block of C independently
    cv::Mat C_inv = cv::Mat::zeros(C.size(), C.type());
    for(int lm_idx = 0; lm_idx < n_landmarks; lm_idx++){
        cv::Mat C_block = C(cv::Rect(lm_idx*3, lm_idx*3, 3, 3));
        cv::Mat C_inv_block = C_inv(cv::Rect(lm_idx*3, lm_idx*3, 3, 3));
        cv::invert(C_block, C_inv_block, cv::DECOMP_LU);
    }
    
    // build RCS = B - E * C_inv * E.t()
    cv::Mat RCS = B - E * C_inv * E.t();

    // build rhs = b_xi - E * C_inv * b_L
    cv::Mat rhs = b_xi - E * C_inv * b_L; 

    // solve RCS * delta_xi = rhs  via cv::solve
    if (!cv::solve(RCS, rhs, delta_xi, cv::DECOMP_CHOLESKY)) return false;

    // back-substitute: delta_L = C_inv * (b_L - E.t() * delta_xi)
    delta_L = C_inv * (b_L - E.t() * delta_xi);

    // return true if solve succeeded
    return true;
}


// Apply delta_xi to each pose via SE(3) exponential map
// Apply delta_L to each landmark position

static void update_parameters(
    std::vector<Frame>& frames,
    std::vector<Landmark>& landmarks,
    const std::vector<int>& pose_idx,
    const cv::Mat& delta_xi,
    const cv::Mat& delta_L
){
    // for each frame with pose:
    for (int i = 0; i < static_cast<int>(frames.size()); i++){
        // extract delta_rho (3x1) and delta_phi (3x1) from delta_xi block
        int pi = pose_idx[frames[i].get_id()]; // get block-id
        if (pi < 0) continue;

        /* Rect(x,y, width, height)*/
        cv::Mat delta_rho = delta_xi(cv::Rect(0, pi*6, 1, 3)); // translation
        cv::Mat delta_phi = delta_xi(cv::Rect(0, pi*6+3, 1, 3)); // rotation (R,P,Y)
        
        /*
            R_new = exp(delta_phi×) * R  ≈ (I + delta_phi×) * R  for small delta
            
            The first-order approximation exp(δφ×) ≈ I + δφ× is only accurate when ||δφ|| is very small. It also produces a matrix that is not exactly a rotation matrix — (I + δφ×) has det ≠ 1 and RᵀR ≠ I for any non-zero δφ. Accumulated over many iterations this drifts off SO(3).
            // cv::Mat delta_phi_x = (cv::Mat_<double>(3,3) << 
            // 0,                        -delta_phi.at<double>(2),  delta_phi.at<double>(1),
            // delta_phi.at<double>(2),   0,                       -delta_phi.at<double>(0),
            // -delta_phi.at<double>(1),  delta_phi.at<double>(0),  0
            // );
        */

        // Using cv::Rodrigues instead 
        // R = I + sin(θ)/θ * δφ×  +  (1 - cos(θ))/θ² * (δφ×)²
        // This is always a valid rotation matrix regardless of step size
        cv::Mat delta_R;
        cv::Rodrigues(delta_phi, delta_R);
        cv::Mat R = delta_R*frames[i].get_R();

        // t_new = t + delta_rho
        cv::Mat t = frames[i].get_t() + delta_rho;

        // frame.set_pose(R_new, t_new)
        frames[i].set_pose(R, t);
    }

    // for each landmark:
    for (int lm_idx = 0; lm_idx < static_cast<int>(landmarks.size()); lm_idx++){
        // update landmark position by delta_L
        landmarks[lm_idx].update_position({delta_L.at<double>(lm_idx*3),
                                           delta_L.at<double>(lm_idx*3 + 1),
                                           delta_L.at<double>(lm_idx*3 + 2),
                                         });
    }
}


// -----------------------------------------------------------------------
// Public function
// -----------------------------------------------------------------------

// bundle-adjustment
void bundle_adjust(
    std::vector<Frame>&         frames,
    std::vector<Landmark>&      landmarks,
    const cv::Mat&              K,
    const int                   window_start,
    int                         window_end, // -1 for global
    const int                   max_iterations, // BA_GLOBAL_ITERATIONS
    const std::vector<int>      fixed_frames
){
    if (window_end == -1) window_end = static_cast<int>(frames.size());

    // build pose_idx map: frame id -> pose block index (also count n_poses)
    std::vector<int> pose_idx(frames.size(), -1);
    int n_poses = 0;
    for (int i = window_start; i < window_end; i++){
        if (frames[i].get_has_pose()){
            // skip fixed frames (DONE TO ENSURE FIXED GAUGE FREEEDOM)
            if (std::find(fixed_frames.begin(), fixed_frames.end(),
                          frames[i].get_id())!= fixed_frames.end()){
                            pose_idx[frames[i].get_id()] = -2;
                            continue;
                          }
            pose_idx[frames[i].get_id()] = n_poses++;
        }    
    }

    // count n_landmarks
    int n_landmarks = static_cast<int>(landmarks.size()); 

    // allocate B (6m x 6m), C (3n x 3n), E (6m x 3n), b_xi (6m x 1), b_L (3n x 1)
    cv::Mat B = cv::Mat::zeros(6*n_poses, 6*n_poses, CV_64F);
    cv::Mat C = cv::Mat::zeros(3*n_landmarks, 3*n_landmarks, CV_64F);
    cv::Mat E = cv::Mat::zeros(6*n_poses, 3*n_landmarks, CV_64F);
    cv::Mat b_xi = cv::Mat::zeros(6*n_poses, 1, CV_64F);
    cv::Mat b_L = cv::Mat::zeros(3*n_landmarks, 1, CV_64F);


    // LM parameters: lambda, total_error
    double lambda = BA_LM_LAMBDA;
    double total_error = 0.0;

    // iteration loop:
    int iter;
    for(iter = 0; iter < max_iterations; iter++){

        // 1. compute_residuals_and_jacobians -> B, C, E, b_xi, b_L, total_cost
        compute_residuals_and_jacobians(
            frames, landmarks, K,
            pose_idx,
            B, C, E, b_xi, b_L,
            total_error,
            window_start, window_end
        );

        if (VERBOSE){
            std::cout << "Iter " << iter + 1 << " cost: " << total_error << " lambda: " << lambda << std::endl;
        }

        // 2. if iter == 0: visualize_matrix for B, C, E
        if(iter == VIS_MAT_ITER && VISUALIZE_MAT){
            visualize_matrix(B, "B");
            visualize_matrix(C, "C");
            visualize_matrix(E, "E");
        }

        // 3. apply_damping(B, C, n_poses, n_landmarks, lambda)
        apply_damping(B, C, n_poses, n_landmarks, lambda);

        // 4. solve_schur -> delta_xi, delta_L
        //    if solve fails: break
        cv::Mat delta_xi;
        cv::Mat delta_L;
        if (!solve_schur(B, C, E, b_xi, b_L,
                         n_poses, n_landmarks,
                         delta_xi, delta_L
                        )){
            if (VERBOSE){
                std::cerr << "Solve Schur failed ..." <<std::endl;
            }      
            break;
        }
        
        // 5. create copy, to try updates
        std::vector<Frame> frames_copy = frames;
        std::vector<Landmark> landmarks_copy = landmarks;

        // 6. trial update on copies
        update_parameters(
            frames_copy, landmarks_copy, pose_idx, delta_xi, delta_L
        );

        // 7. compute new error
        double updated_error = 0.0;
        compute_residuals_and_jacobians(
            frames_copy, landmarks_copy, K, pose_idx,
            B, C, E, b_xi, b_L, updated_error,
            window_start, window_end
        );

        // 8. accept or reject update
        if (updated_error < total_error){
            frames = frames_copy;
            landmarks = landmarks_copy;

            double relative_improvement = (total_error - updated_error)/total_error;
            if (relative_improvement > RELATIVE_ERROR_IMPROVEMENT){
                lambda /= BA_LM_LAMBDA_SCALE_FACTOR; 
            }

            // else: accept step but keep lambda -  improvement too small to justify decrease

            // 9. check for convergence
            if (std::abs(total_error - updated_error) < BA_ERROR_TOL){
                total_error = updated_error;
                break;
            }
        }
        else{
            lambda *= BA_LM_LAMBDA_SCALE_FACTOR;
        }
        
    }

    //10. print final cost and iterations
    if (VERBOSE){
        std::cout << "\nTotal iterations: " << iter << ".\nTotal error: " << total_error <<".\n";
    }

}
