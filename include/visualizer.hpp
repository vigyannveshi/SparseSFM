/**
 * @file visualizer.hpp
 * @brief Pangolin-based 3D viewer for sparse SfM pipeline.
 * @author Rudraksha R. Bandodkar
 * @copyright MIT License, see LICENSE
 */

/**  NOTES:
 * A camera frustum is the region of space visible to camera bounded by the image plane and the four rays going out from the camera center through the four corners of the image
 * Visually it looks like a pyramid with the tip at the camera center and the base being the image plane in the front
 * For drawing, we need five points in the camera's local frame coordinate system:
    * O  = (0,0,0) - camera center (tip)
    * p1 = (-w, -h, f) — top-left corner of image plane
    * p2 = ( w, -h, f) — top-right corner of image plane
    * p3 = ( w,  h, f) — bottom-right corner of image plane
    * p4 = (-w,  h, f) — bottom-left corner of image plane
    Where w and h are half-widths scaled by a display factor, and f is how far out the frustum extends (display depth)
 * then you draw:
    * 4 lines from O to each corner (the pyramid edges)
    * 4 lines connecting the corners (the base rectangle)
 * To place this in the scene you transform each point by the camera pose R and t rotate and translate it into the reference frame's coordinate system.
 
 * sign conventions
(0,0)---------------> x (width)
  |
  |
  |
  |
  |
  V
  y (height)
*/

#ifndef VISUALIZER_HPP
#define VISUALIZER_HPP

#include <pangolin/pangolin.h>
#include <opencv2/opencv.hpp>
#include <vector>
#include "landmark.hpp"
#include "frame.hpp"
#include "pose.hpp"

// constants
#include "constants.hpp"

// defining struct for camera frustum
struct CameraFrustum {
    cv::Point3d O;  // camera center
    cv::Point3d p1; // top-left image corner
    cv::Point3d p2; // top-right image corner
    cv::Point3d p3; // bottom-right image corner
    cv::Point3d p4; // bottom-left image corner
};

// defining an inline camera_frustum, to avoid re-creation for every definition
inline CameraFrustum camera_frustum{
    {0,0,0},
    {-PANGOLIN_CAMERA_FRUSTUM_W, -PANGOLIN_CAMERA_FRUSTUM_H, PANGOLIN_CAMERA_FRUSTUM_F},
    {PANGOLIN_CAMERA_FRUSTUM_W, -PANGOLIN_CAMERA_FRUSTUM_H, PANGOLIN_CAMERA_FRUSTUM_F},
    {PANGOLIN_CAMERA_FRUSTUM_W, PANGOLIN_CAMERA_FRUSTUM_H, PANGOLIN_CAMERA_FRUSTUM_F},
    {-PANGOLIN_CAMERA_FRUSTUM_W, PANGOLIN_CAMERA_FRUSTUM_H, PANGOLIN_CAMERA_FRUSTUM_F},
};

// function to visualize point-clouds using landmarks
void visualize_landmarks(const std::vector<Landmark>& landmarks, 
                         const std::vector<Frame>& frames,
                         const std::vector<Pose>& poses = {},
                         const std::string& title = "Sparse SFM Viewer"
                        );

// function to get landmark colors
std::vector<cv::Vec3b> get_landmark_colors(
    const std::vector<Landmark>& landmarks,
    const std::vector<Frame>& frames
);

// function to camera-frustum at given pose
void draw_camera_frustum(const Pose& pose);

// function to trace poses (draws line between two given poses)
void trace_path(const std::pair<Pose,Pose>& pose_pair, const Color& color,const float& linewidth);

#endif

