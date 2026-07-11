/**
 * @file constants.hpp
 * @brief Project-wide tunable constants. All pipeline parameters live
 *        here. Modify this file to tune the pipeline behaviour.
 *
 * @author Rudraksha R. Bandodkar
 * @copyright Copyright (c) 2026 Rudraksha R. Bandodkar. MIT License, see
 *            LICENSE file in the project root for full terms.
 */

#ifndef CONSTANTS_HPP
#define CONSTANTS_HPP

// struct for RGB-color
struct Color{
    float R;
    float G;
    float B;
    float alpha;
};

// ----- frame_io -----
constexpr int ZERO_PADDING = 6;
constexpr int FRAMES_SAMPLING_INTERVAL = 10;

// ----- orb_extractor -----
constexpr int ORB_NUM_FEATURES = 2000;
constexpr float ORB_SCALE_FACTOR = 1.2f;
constexpr int ORB_N_LEVELS = 8;

// ----- orb matcher -----
constexpr float ORB_RATIO_THRESHOLD = 0.75f;

// ----- sift_extractor -----
constexpr int SIFT_NUM_FEATURES = 2000;
constexpr double SIFT_CONTRAST_THRESHOLD = 0.04;
constexpr int SIFT_EDGE_THRESHOLD = 10;
constexpr int SIFT_N_OCTAVES = 3;
constexpr double SIFT_SIGMA = 1.6;

// ----- sift matcher -----
constexpr float SIFT_RATIO_THRESHOLD = 0.75f;

// ----- initializer -----
constexpr float INIT_RH_THRESHOLD = 0.45f;
constexpr int INIT_SCAN_WINDOW = 15;
constexpr int INIT_E_INLIER_COUNT = 20;

// Triangulation
constexpr double TRIANGULATION_MAX_REPROJ_ERROR = 4.0;

// ----- p3p-solver -----
constexpr double TENDS_TO_ZERO_THRESH = 1e-10;
constexpr double IMAGINARY_THRESHOLD = 1e-6;
constexpr int RANSAC_ITERATIONS = 100;
constexpr double MIN_SAMPLE_DIST_IMG_SPACE = 15.0;
constexpr double REPROJECTION_ERROR_THRESHOLD = 3.0; // orig: 3.0
constexpr int P3P_MIN_INLIERS = 6;

// ----- bundle-adjustments -----
constexpr int BA_GLOBAL_ITERATIONS = 20; // orig: 50
constexpr int BA_LOCAL_ITERATIONS = 5; // orig: 10
constexpr double BA_LM_LAMBDA = 1e-4;
constexpr double BA_LM_LAMBDA_SCALE_FACTOR = 10;
constexpr double BA_ERROR_TOL = 1e-6;
constexpr double RELATIVE_ERROR_IMPROVEMENT = 1e-3; // orig: 1e-2

// ----- pipeline & drivers -----
constexpr int MAX_INITIALIZATION_TRIALS = 5; 
constexpr float CULLING_THRESHOLD = 2.0; // 2px
constexpr int N_REGISTRATION_PASSES = 5; // orig: 3
constexpr int MATCH_WINDOW_SIZE = 3; // orig: 5

// ----- visualizer ---

constexpr Color PANGOLIN_BACKGROUND_COLOR {1.0f, 1.0f, 1.0f, 1.0f}; // white
constexpr int PANGOLIN_WINDOW_WIDTH = 1024;
constexpr int PANGOLIN_WINDOW_HEIGHT = 768;

// viewer's camera calibration matrix
constexpr double PANGOLIN_VIEWER_FX = 500.0;
constexpr double PANGOLIN_VIEWER_FY = 500.0;
constexpr double PANGOLIN_VIEWER_CX = static_cast<double>(PANGOLIN_WINDOW_WIDTH)/2.0;
constexpr double PANGOLIN_VIEWER_CY = static_cast<double>(PANGOLIN_WINDOW_HEIGHT)/2.0;
constexpr double PANGOLIN_VIEWER_NEAR_CLIP = 0.1;  // near clipping plane
constexpr double PANGOLIN_VIEWER_FAR_CLIP  = 1000.0;  // far clipping plane

// location of viewer's camera-center/eye
constexpr double PANGOLIN_VIEWER_EX = 0.0; 
constexpr double PANGOLIN_VIEWER_EY = 0.0;
constexpr double PANGOLIN_VIEWER_EZ = -10.0;

// location at which viewer is looking at
constexpr double PANGOLIN_VIEWER_LX = 0.0; 
constexpr double PANGOLIN_VIEWER_LY = 0.0;
constexpr double PANGOLIN_VIEWER_LZ = 0.0;

// point cloud
constexpr float PANGOLIN_POINT_CLOUD_POINT_SIZE = 3.0f;

// image buffer size
constexpr int PANGOLIN_IMG_BUFFER_SIZE = 20;

// camera 
constexpr float PANGOLIN_CAMERA_FRUSTUM_W = 0.1f;
constexpr float PANGOLIN_CAMERA_FRUSTUM_H = 0.075f;
constexpr float PANGOLIN_CAMERA_FRUSTUM_F = 0.15f;
constexpr float PANGOLIN_CAMERA_FRUSTUM_LINE_WIDTH = 2.0f;
constexpr Color PANGOLIN_CAMERA_FRUSTUM_COLOR {0.f, 0.f, 1.0f, 1.0f}; // blue
constexpr float PANGOLIN_CAMERA_PATH_LINE_WIDTH = 1.0f;
constexpr Color PANGOLIN_CAMERA_PATH_COLOR {0.f, 1.f, 0.0f, 1.0f}; // green


// ---- other constants ----
constexpr float VIZ_SCALE = 0.4f; // downscale by, while visualizing features extracted
constexpr double MATRIX_MAX_DISPLAY = 800.0; // scaling matrix for visualization
#endif