/**
 * @file triangulator.hpp
 * @brief Triangulation of 3D landmark positions from two-view point correspondences.
 * @author Rudraksha R. Bandodkar
 * @copyright MIT License, see LICENSE
 */

#ifndef TRIANGULATOR_HPP
#define TRIANGULATOR_HPP

#include <opencv2/opencv.hpp>
#include <vector>
#include "landmark.hpp"
#include "initializer.hpp"


// function to triangulate single point
cv::Point3d triangulate_point(
    const cv::Point2f& p1,
    const cv::Point2f& p2,
    const cv::Mat& R1, const cv::Mat& t1,
    const cv::Mat& R2, const cv::Mat& t2,
    const cv::Mat& K    
);


// function to triangulate all points
void triangulate_all(
    const int frame_idx1,
    const int frame_idx2,
    const InlierData& inlier_data,
    const cv::Mat& R1, const cv::Mat& t1,
    const cv::Mat& R2, const cv::Mat& t2,
    const cv::Mat& K, 
    std::vector<Landmark>& landmarks
);

#endif