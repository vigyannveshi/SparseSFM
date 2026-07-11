/**
 * @file camera.hpp
 * @brief Declares the Camera class, a simple data holder for a single
 *        camera's intrinsic matrix (K) and distortion coefficients, built
 *        once at construction from already-parsed calibration values and
 *        shared, read-only, across all frames in the sequence.
 *
 * @author Rudraksha R. Bandodkar
 * @copyright Copyright (c) 2026 Rudraksha R. Bandodkar. MIT License, see
 *            LICENSE file in the project root for full terms.
 */

#ifndef CAMERA_HPP
#define CAMERA_HPP

#include <opencv2/opencv.hpp>

class Camera{
    public:
        Camera(double fx, double fy, double cx, double cy, double s, 
               double k1, double k2, double p1, double p2, double k3);
            
        cv::Mat get_K() const;

        cv::Mat get_D() const;

    private:
        cv::Mat K_;
        cv::Mat dist_coeffs_;
};

#endif