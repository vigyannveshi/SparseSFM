/**
 * @file p3p_solver.hpp
 * @brief P3P pose estimation solver with RANSAC, using Gao et al. 2003 quartic formulation
 * @author Rudraksha R. Bandodkar
 * @copyright MIT License, see LICENSE
 */

#ifndef P3P_SOLVER_HPP
#define P3P_SOLVER_HPP

#include <opencv2/opencv.hpp>
#include <vector>
#include "pose.hpp"

// p3p solver 
std::vector<Pose> solve_p3p(
    const std::vector<cv::Point2f>& pts2d,
    const std::vector<cv::Point3d>& pts3d,
    const cv::Mat& K
);

bool ransac_pnp(
    const std::vector<cv::Point2f>& pts2d,
    const std::vector<cv::Point3d>& pts3d,
    const cv::Mat& K,
    Pose& pose_out,
    std::vector<bool>& inlier_mask
);


#endif