/**
 * @file pose.hpp
 * @brief Pose data structure and geometric utilities for camera pose manipulation.
 * @author Rudraksha R. Bandodkar
 * @copyright MIT License, see LICENSE
 */

#ifndef POSE_HPP
#define POSE_HPP

#include <opencv2/opencv.hpp>

struct Pose
{
    cv::Mat R;
    cv::Mat t;
};

// transforms point given pose: from camera frame to reference frame
cv::Point3d camera_to_ref_frame(const cv::Point3d& p, const Pose& pose);

// transform point given pose: from reference frame to camera frame
cv::Point3d ref_to_camera_frame(const cv::Point3d& P, const Pose& pose);

// convert point3DtoMat
cv::Mat point3DtoMat(const cv::Point3d &p);

// compute reprojection error
double reprojection_error(
    const Pose& pose,
    const cv::Point3d& pt3d,
    const cv::Point2f& pt2d,
    const cv::Mat& K
);

#endif

