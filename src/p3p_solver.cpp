/**
 * @file p3p_solver.cpp
 * @brief Implementation of P3P pose estimation and RANSAC driver
 * @author Rudraksha R. Bandodkar
 * @copyright MIT License, see LICENSE
 */

#include "p3p_solver.hpp"

#include <iostream>
#include <cmath>
#include <numeric>
#include <algorithm>
#include <random>

// constants
#include "constants.hpp"


// qaurtic solver (static function)
static std::vector<double> solve_quartic(
    double a0, double a1, double a2, double a3, double a4 
){
    if (std::abs(a0) < TENDS_TO_ZERO_THRESH){
        std::cerr << "degenerate a0: "<<a0<<std::endl;
        return {};  // degenerate, skip
    } 

    // companion-matrix
    cv::Mat C = cv::Mat::zeros(4,4, CV_64F);

    // adding values to companion matrix
    C.at<double>(1,0) = 1.0;
    C.at<double>(2,1) = 1.0;
    C.at<double>(3,2) = 1.0;

    C.at<double>(0,3) = -a4/a0;
    C.at<double>(1,3) = -a3/a0;
    C.at<double>(2,3) = -a2/a0;
    C.at<double>(3,3) = -a1/a0;

    cv::Mat eig_values;
    cv::Mat eig_vectors;
    
    try {
        cv::eigenNonSymmetric(C, eig_values, eig_vectors);
    } catch (const cv::Exception& e) {
        std::cerr<<e.what()<<std::endl;
        return {};
    }

    std::vector<double> x;

    for(int i = 0; i < 4; i++){
        if(std::abs(eig_values.at<double>(i,1)) < IMAGINARY_THRESHOLD)
        {
            x.push_back(eig_values.at<double>(i,0));
        }
    }
    return x;
}

// Procrutes solver (static function)
static void computeProcrutes(const std::vector<cv::Mat>& P, const cv::Mat& x0,
                             const std::vector<cv::Mat>& Q, const cv::Mat& y0,
                             Pose &p
){
    cv::Mat H = cv::Mat::zeros(3,3, CV_64F);
    for (int i = 0; i < 3; i++){
        H += (P[i]-x0)*(Q[i]-y0).t();
    }

    // compute SVD
    cv::Mat U,Sig,Vt;
    cv::SVD::compute(H,Sig,U,Vt);

    // compute R (rotation w.r.t Camera frame 0 (world frame))
    cv::Mat R = Vt.t() * U.t();

    // reflection fix
    if (cv::determinant(R) < 0 ){
        Vt.row(2) *= -1;
        R = Vt.t() * U.t();
    }
    cv::Mat t = y0 - R*x0;
    p = {R,t};
}


// p3p solver 
std::vector<Pose> solve_p3p(
    const std::vector<cv::Point2f>& pts2d,
    const std::vector<cv::Point3d>& pts3d,
    const cv::Mat& K
){
    cv::Mat K_inv = K.inv();

    //Step 1. Unproject the 3 pixel points through K to get unit rays f1, f2, f3
    std::vector<cv::Mat> F;
    for (cv::Point2f pt: pts2d){
        cv::Mat p = (cv::Mat_<double>(3,1) << static_cast<double>(pt.x), 
                                              static_cast<double>(pt.y), 1.0);
        cv::Mat f = K_inv*p;
        f = f/cv::norm(f);
        F.push_back(f);
    }

    // Step 2. Cosines of apex angles
    const double cos12 = F[0].dot(F[1]);
    const double cos13 = F[0].dot(F[2]);
    const double cos23 = F[1].dot(F[2]);


    // Step 3. World-side distances (w.r.t Camera-Frame 0)
    double d12 = cv::norm(pts3d[0] - pts3d[1]);
    double d13 = cv::norm(pts3d[0] - pts3d[2]);
    double d23 = cv::norm(pts3d[1] - pts3d[2]);

    // Step 4. Gao's parameters
    double p = 2*cos23;   // angle BPC -> F[1].dot(F[2])
    double q = 2*cos13;   // angle APC -> F[0].dot(F[2])
    double r = 2*cos12;   // angle APB -> F[0].dot(F[1])

    double a = (d23*d23) / (d12*d12);
    double b = (d13*d13) / (d12*d12);

    // Step 5. Quartic coefficients from Gao et al. Appendix A
    double a0 = -2*b + b*b + a*a + 1 - b*r*r*a + 2*b*a - 2*a;

    double a1 = -2*b*q*a - 2*a*a*q + b*r*r*q*a - 2*q + 2*b*q
                + 4*a*q + p*b*r + b*r*p*a - b*b*r*p;

    double a2 = q*q + b*b*r*r - b*p*p - q*p*b*r + b*b*p*p
                - b*r*r*a + 2 - 2*b*b - a*b*r*p*q
                + 2*a*a - 4*a - 2*q*q*a + q*q*a*a;

    double a3 = -b*b*r*p + b*r*p*a - 2*a*a*q + q*p*p*b
                + 2*b*q*a + 4*a*q + p*b*r - 2*b*q - 2*q;

    double a4 = 1 - 2*a + 2*b + b*b - b*p*p + a*a - 2*b*a;
    
    // Step 6. Solve quartic
    std::vector<double> roots = solve_quartic(a0, a1, a2, a3, a4);

    // Step 7. Create a vector of 3D points, and also compute its centroid    
    std::vector<cv::Mat> P = {point3DtoMat(pts3d[0]),
                              point3DtoMat(pts3d[1]),
                              point3DtoMat(pts3d[2])};

    // compute centroid of 3D points
    cv::Mat x0 = (P[0] + P[1] + P[2])/3.0;

    /* Step 8.
     i)   For each real positive root x, recover y, s1, s2, s3 
     ii)  Create projection vectors for that solution
     iii) Get pose using Procrutes
    */  
    
    std::vector<Pose> poses;
    
    for (double x:roots){
        if(x <= 0) continue;

        // recover y from quadratic in y
        // (1-a)*y^2 - y*(p - a*x*r) + (1 - a*x^2) = 0
        double A = 1 - a;
        double B = -(p - a*x*r);
        double C = 1 - a*x*x;
        double disc = B*B - 4*A*C;

        // imaginary roots for quadratic in y
        if (disc < 0) continue;

        std::vector<double> y_candidates;

        if (std::abs(A) < TENDS_TO_ZERO_THRESH){
            // degenerate: linear in y
            if (std::abs(B) > TENDS_TO_ZERO_THRESH)
                y_candidates.push_back(-C / B);
        } else {
            y_candidates.push_back((-B + std::sqrt(disc)) / (2*A));
            y_candidates.push_back((-B - std::sqrt(disc)) / (2*A));
        }

        // get x, s1, s2, s3
        for (double y : y_candidates){
            if (y <= 0) continue;

            // recover s3, s1, s2
            double v  = x*x + y*y - x*y*r;
            if (v <= 0) continue;
            double s3 = d12 / std::sqrt(v);
            double s1 = x * s3;
            double s2 = y * s3;

            // projection vectors for particular solution
            std::vector<cv::Mat> Q = {s1*F[0], s2*F[1], s3*F[2]};

            // centroid of projection vectors
            cv::Mat y0 = (Q[0]+Q[1]+Q[2])/3.0;

            // compute Procrutes
            Pose p;
            computeProcrutes(P, x0, Q, y0, p);
            poses.push_back(p);
        }
    }
    return poses;  
}


static void randomize_indices(std::vector<int>& indices){
    std::iota(indices.begin(), indices.end(), 0);
    std::shuffle(indices.begin(), indices.end(), std::mt19937{std::random_device{}()});
}


bool ransac_pnp(
    const std::vector<cv::Point2f>& pts2d,
    const std::vector<cv::Point3d>& pts3d,
    const cv::Mat& K,
    Pose& pose_out,
    std::vector<bool>& inlier_mask
){    
    // total points
    int total_matches = pts2d.size();

    // if total matches < Minimum P3P inliers -> No need to perform RANSAC-P3P
    if (total_matches < P3P_MIN_INLIERS){
        return false;
    }
    
    // indices
    std::vector<int> indices(total_matches);
    
    // best inlier count
    int best_inlier_count = 0;

    // best pose
    Pose best_pose;

    for (int i = 0; i < RANSAC_ITERATIONS; i++) // for each iteration
    {
        // create randomize indices
        randomize_indices(indices);
        
        // sample 3 points using randomized indices
        std::vector<cv::Point3d> p3d = {pts3d[indices[0]],pts3d[indices[1]],pts3d[indices[2]]};
        std::vector<cv::Point2f> p2d = {pts2d[indices[0]],pts2d[indices[1]],pts2d[indices[2]]};

        // reject degenerate samples where points are too close in image space
        double d01 = cv::norm(p2d[0] - p2d[1]);
        double d02 = cv::norm(p2d[0] - p2d[2]);
        double d12 = cv::norm(p2d[1] - p2d[2]);
        
        if (d01 < MIN_SAMPLE_DIST_IMG_SPACE || 
            d02 < MIN_SAMPLE_DIST_IMG_SPACE || 
            d12 < MIN_SAMPLE_DIST_IMG_SPACE)
            continue;

        // solve p3p to get upto 4 poses
        std::vector<Pose> poses = solve_p3p(p2d, p3d, K);
        
        // for each pose
        for (const Pose& pose : poses){
            // reset inlier mask
            int inlier_count = 0;

            // temporary inlier mask
            std::vector<bool> temp_mask(total_matches, false);

            // score the pose against all the points
            for(int j = 0; j < total_matches; j++){
                if (reprojection_error(pose, pts3d[j], pts2d[j], K)
                    <REPROJECTION_ERROR_THRESHOLD)
                {
                    inlier_count++;
                    temp_mask[j] = true;
                }
            }
            if (inlier_count > best_inlier_count){
                best_pose = pose;
                best_inlier_count = inlier_count;
                inlier_mask = temp_mask;
            }
        }
    }

    if (best_inlier_count < P3P_MIN_INLIERS) return false;
    pose_out = best_pose;
    return true;
}