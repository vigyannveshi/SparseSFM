/**
 * @file pose.cpp
 * @brief Implementation of pose utilities declared in pose.hpp.
 * @author Rudraksha R. Bandodkar
 * @copyright MIT License, see LICENSE
 */

#include "pose.hpp"

// transform point given pose: from camera frame to reference frame
cv::Point3d camera_to_ref_frame(const cv::Point3d& p, const Pose& pose)
{
    cv::Mat pm = (cv::Mat_<double>(3,1) << p.x, p.y, p.z);
    pm = pose.R.t()*pm - pose.R.t()*pose.t;
    return {pm.at<double>(0), pm.at<double>(1), pm.at<double>(2)};
}

// transform point given pose: from  reference frame to camera frame
cv::Point3d ref_to_camera_frame(const cv::Point3d& P, const Pose& pose){
    cv::Mat Pm = (cv::Mat_<double>(3,1) << P.x, P.y, P.z);
    Pm = pose.R*Pm + pose.t;
    return {Pm.at<double>(0), Pm.at<double>(1), Pm.at<double>(2)};
}


// converting 3D point to matrix
cv::Mat point3DtoMat(const cv::Point3d &p){
    cv::Mat P = (cv::Mat_<double>(3,1) << p.x, p.y, p.z);
    return P;
}

// reprojection error
double reprojection_error(
    const Pose& pose,
    const cv::Point3d& pt3d,
    const cv::Point2f& pt2d,
    const cv::Mat& K
){
    cv::Mat x_proj_h = K*(pose.R * point3DtoMat(pt3d) + pose.t); // homogeneous coordinates
    cv::Point2d x_proj_e(x_proj_h.at<double>(0)/x_proj_h.at<double>(2),
                        x_proj_h.at<double>(1)/x_proj_h.at<double>(2));

    double dx = x_proj_e.x - static_cast<double>(pt2d.x);
    double dy = x_proj_e.y - static_cast<double>(pt2d.y);
    return std::sqrt(dx*dx + dy*dy);
}