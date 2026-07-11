/**
 * @file calibrate.cpp
 * @brief Camera calibration from a checkerboard video. Detects checkerboard
 *        corners across sampled frames, runs OpenCV's calibrateCamera, and
 *        writes the intrinsics (fx, fy, cx, cy, s) and distortion
 *        coefficients (k1, k2, p1, p2, k3) to a text file.
 *
 * @usage ./calibrate <video_path> <output_path>
 *
 * @author Rudraksha R. Bandodkar
 * @copyright Copyright (c) 2026 Rudraksha R. Bandodkar. MIT License, see
 *            LICENSE file in the project root for full terms.
 */

#include <iostream>
#include <fstream>
#include <opencv2/opencv.hpp>

// GLOBAL VARIABLES
constexpr int SAMPLE_INTERVAL = 15;
constexpr float SQUARE_SIZE = 20.0f;
const cv::Size PATTERN_SIZE(9, 6); // internal corners (orientation doesn't matter, it looks for 9 interal corners on one side, and 6 internal corners on the other), so (6,9) is also okay.


int main(int argc, char** argv){
    // if we don't get 3 arguements (./calibrate, video_path, output_path) raise error
    if (argc != 3){
        std::cerr<< "Usage: "<<argv[0] << " <video_path> <output_path>" <<std::endl;
        return 1;
    }

    // getting the video and output path
    std::string video_path = argv[1];
    std::string output_path = argv[2];

    // reading video and ensure that it is found
    cv::VideoCapture cap(video_path);

    if (!cap.isOpened()){
        std::cerr << "Error: could not open video file" << video_path << std::endl;
        return 1; 
    }

    cv::Size image_size(
        static_cast<int>(cap.get(cv::CAP_PROP_FRAME_WIDTH)),
        static_cast<int>(cap.get(cv::CAP_PROP_FRAME_HEIGHT))
    );


    // getting corners
    cv::Mat frame;
    int frame_idx = 0;
    int good_detections = 0;


    // collecting corners and object points
    std::vector<std::vector<cv::Point2f>> image_points;
    std::vector<std::vector<cv::Point3f>> object_points;

    // build the 3D template once, the frame loop
    std::vector<cv::Point3f> obj_points_template;
    for (int row = 0; row < PATTERN_SIZE.height; row ++){
        for (int col = 0; col < PATTERN_SIZE.width; col++){
            obj_points_template.push_back(
                cv::Point3f(
                    col*SQUARE_SIZE,
                    row*SQUARE_SIZE,
                    0.0f
                )
            );
        }
    }

    // reading frames and getting corners for the sampled frames
    while (cap.read(frame)){
        if (frame_idx % SAMPLE_INTERVAL == 0)
        {
            cv::Mat gray;
            cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

            std::vector<cv::Point2f> corners;
            bool found = cv::findChessboardCorners(gray, PATTERN_SIZE, corners);

            if (found)
            {
                // subpixel refinement
                cv::cornerSubPix(
                        gray,corners, cv::Size(11,11), cv::Size(-1,-1),
                        cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 0.001) 
                );
                image_points.push_back(corners);
                object_points.push_back(obj_points_template);
                
                // increment good_detections
                good_detections++;

                // visualize corners tracked
                cv::drawChessboardCorners(frame, PATTERN_SIZE, corners, found);

                // resize for visualization
                cv::Mat display;
                cv::resize(frame, display, cv::Size(), 0.4,0.4);
                cv::imshow("Corners detected", display);
                cv::waitKey(1);
            }

        }
        frame_idx++;
    }

    cv::destroyAllWindows();

    // camera calibration
    cv::Mat K, dist_coeffs;
    std::vector<cv::Mat> rvecs, tvecs;
    

    double rms_error = cv::calibrateCamera(
        object_points, image_points, image_size,
        K, dist_coeffs, rvecs, tvecs,
        cv::CALIB_USE_LU
    );

    std::cout << "Good detections: " << good_detections << " out of frames sampled: "<< static_cast <int> (frame_idx/SAMPLE_INTERVAL) <<std::endl;

    std::cout << "RMS reprojection error: " << rms_error <<std::endl;

    // writing out the calibration matrices
    std::ofstream out_file(output_path);

    // getting the camera parameters
    double fx = K.at<double>(0,0);
    double fy = K.at<double>(1,1);
    double cx = K.at<double>(0,2);
    double cy = K.at<double>(1,2);
    double s = K.at<double>(0,1);

    out_file << fx << " " << fy << " " << cx << " " << cy << " " << s << std::endl;

    for (int i = 0; i < dist_coeffs.cols; i++){
        out_file<<dist_coeffs.at<double>(0,i) << " "; 
    }

    out_file.close();
    return 0;
}