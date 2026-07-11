/**
 * @file triangulator.cpp
 * @brief Implementation of triangulation functions for sparse SfM pipeline.
 * @author Rudraksha R. Bandodkar
 * @copyright MIT License, see LICENSE
 */

#include "triangulator.hpp"
#include "pose.hpp"
#include "constants.hpp"

cv::Point3d triangulate_point(
    const cv::Point2f& p1,
    const cv::Point2f& p2,
    const cv::Mat& R1, const cv::Mat& t1,
    const cv::Mat& R2, const cv::Mat& t2,
    const cv::Mat& K    
){
    // unproject pixels to normalized rays
    cv::Mat K_inv = K.inv();
    
    // getting pixel coordinates -> camera coordinates
    cv::Mat x1 = K_inv * (cv::Mat_<double>(3,1) << static_cast<double>(p1.x), static_cast<double>(p1.y), 1);
    cv::Mat x2 = K_inv * (cv::Mat_<double>(3,1) << static_cast<double>(p2.x), static_cast<double>(p2.y), 1);

    // getting vector rays
    cv::Mat r = R1.t() * x1;
    cv::Mat s = R2.t() * x2;

    // getting camera principal point
    cv::Mat p = -R1.t() * t1;
    cv::Mat q = -R2.t() * t2;

    // defining A
    cv::Mat A = (cv::Mat_<double>(2,2) <<
        r.dot(r), -r.dot(s),
        s.dot(r), -s.dot(s)
    );

    // defining b
    cv::Mat b = (cv::Mat_<double>(2,1) <<
        (q-p).dot(r),
        (q-p).dot(s)
    );

    // solving Ax = b 
    cv::Mat x;
    cv::solve(A, b, x);

    // getting vector rays
    cv::Mat f = p + x.at<double>(0)*r;
    cv::Mat g = q + x.at<double>(1)*s;

    // getting the midpoint of intersection of f & g rays
    cv::Mat midpoint = (f + g) / 2.0;
    cv::Point3d X;
    X.x = midpoint.at<double>(0);
    X.y = midpoint.at<double>(1);
    X.z = midpoint.at<double>(2);
    return X;    
}

void triangulate_all(
    const int frame_idx1,
    const int frame_idx2,
    const InlierData& inlier_data,
    const cv::Mat& R1, const cv::Mat& t1,
    const cv::Mat& R2, const cv::Mat& t2,
    const cv::Mat& K, 
    std::vector<Landmark>& landmarks
){    

    for (int i = 0; i < inlier_data.pts1.size(); i++){
        // storing the triangulated point as a landmark
        Landmark lm(triangulate_point(inlier_data.pts1[i],
                                      inlier_data.pts2[i],
                                      R1, t1, 
                                      R2, t2,
                                      K)
                    );
        
        // chirality check
        /* check point is in front of both cameras*/
        cv::Point3d Pk1 = ref_to_camera_frame(lm.get_position(), {R1, t1});
        cv::Point3d Pk2 = ref_to_camera_frame(lm.get_position(), {R2, t2});
        if (Pk1.z < 0 || Pk2.z < 0) continue;

        // reprojection error check
        if (reprojection_error({R1, t1}, lm.get_position(), inlier_data.pts1[i],K) > TRIANGULATION_MAX_REPROJ_ERROR) continue;
        if (reprojection_error({R2, t2}, lm.get_position(), inlier_data.pts2[i],K) > TRIANGULATION_MAX_REPROJ_ERROR) continue;

        // the point was observed in frame_idx_1, with keypoint as:
        lm.add_observation(Observation{
            frame_idx1, inlier_data.inlier_matches[i].queryIdx
        });

        // the point was observed in frame_idx_2, with keypoint as:
        lm.add_observation(Observation{
            frame_idx2, inlier_data.inlier_matches[i].trainIdx
        });

        landmarks.push_back(lm);
    }
}
