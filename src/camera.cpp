/**
 * @file camera.cpp
 * @brief Implements the Camera class declared in camera.hpp.
 *
 * @author Rudraksha R. Bandodkar
 * @copyright Copyright (c) 2026 Rudraksha R. Bandodkar. MIT License, see
 *            LICENSE file in the project root for full terms.
 */

#include "camera.hpp"

// Constructor: builds K_ and dist_coeffs_ once from the ten calibration values
Camera::Camera(double fx, double fy, double cx, double cy, double s, 
               double k1, double k2, double p1, double p2, double k3){
    K_ = (cv::Mat_<double>(3,3) <<
                fx, s,  cx,
                0,  fy, cy,
                0,  0,  1    
        );
    dist_coeffs_ = (cv::Mat_<double>(1,5) << k1, k2, p1, p2, k3);
}

cv::Mat Camera::get_K() const{
    return K_;
}

cv::Mat Camera::get_D() const{
    return dist_coeffs_;
}
