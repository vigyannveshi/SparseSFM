/**
 * @file undistort_visualization.cpp
 * @brief Reads camera intrinsics and distortion coefficients from a
 *        calibration text file, applies cv::undistort to each frame of a
 *        video, and displays the original and undistorted frames side by
 *        side for visual verification.
 *
 * @usage ./undistort_visualization <video_path> <calib_path>
 *
 * @author Rudraksha R. Bandodkar
 * @copyright Copyright (c) 2026 Rudraksha R. Bandodkar. MIT License, see
 *            LICENSE file in the project root for full terms.
 */

#include <iostream>
#include <fstream>
#include <opencv2/opencv.hpp>

// GLOBAL VARIABLES
constexpr float SCALE_WIDTH = 0.4;
constexpr float SCALE_HEIGHT = 0.4;


int main(int argc,char** argv){

    std::string output_path = "";
    bool save_output = false;
    cv::VideoWriter writer;

    // check for atleast 3 arguements (if 4 arguements are given, consider it as output_path)
    if (argc == 4){
        output_path = argv[3];
        save_output = true;
    }
    else if (argc != 3)
    {
        std::cerr << "Usage: "<< argv[0]<< " <video_path> <calib_path>"<<std::endl;
        return 1; 
    }
    

    std::string video_path = argv[1];
    std::string calib_path = argv[2];

    // reading the calibration file
    std::ifstream in_file(calib_path);

    double fx, fy, cx, cy, s, k1, k2, p1, p2, k3;

    /*
    operator `>>` on an input stream, when reading into a numeric type like double, automatically skips all leading whitespace before reading the next token, and whitespace includes spaces, tabs, and newlines, all treated identically
    */
    in_file >> fx >> fy >> cx >> cy >> s >> k1 >> k2 >> p1 >> p2 >> k3;

    // reconstructing K and dist_coeffs as cv::Mat
    cv::Mat K = (cv::Mat_<double>(3,3) <<
     fx, s,  cx, 
     0,  fy, cy,
      0,  0,  1
    );

    cv::Mat dist_coeffs = (cv::Mat_<double>(1,5) << k1, k2, p1, p2, k3);

    // reading video
    cv::VideoCapture cap(video_path);

    if (!cap.isOpened())
    {
        std::cerr << "Error: could not open video file" << video_path <<std::endl;
        return 1;
    }
    
    // setting video-writer
    if (save_output){
        // fps
        double fps = cap.get(cv::CAP_PROP_FPS);
        
        // frame size
        cv::Size output_size(
            static_cast<int>(cap.get(cv::CAP_PROP_FRAME_WIDTH)*2*SCALE_WIDTH),
            static_cast<int>(cap.get(cv::CAP_PROP_FRAME_HEIGHT)*SCALE_HEIGHT)
        );
        int fourcc = cv::VideoWriter::fourcc('m', 'p', '4', 'v');
        if (!writer.open(output_path, fourcc, fps, output_size)){
            std::cerr << "Error: could not open output video writer for path: " << output_path <<" (fourcc: MJPG, fps: " << fps << ", size: "<< output_size << ")" << std::endl;
            return 1;
        }
    }

    // undistorting
    cv::Mat frame, undistorted, disp_org, disp_undist, combined;

    while (cap.read(frame))
    {
        cv::undistort(frame, undistorted, K, dist_coeffs);
        cv::resize(frame, disp_org, cv::Size(), 0.4, 0.4);
        cv::resize(undistorted, disp_undist, cv::Size(), 0.4, 0.4);

        cv::hconcat(disp_org, disp_undist, combined);
        // show image
        cv::imshow("Original vs Undistorted", combined);

        // write to output video
        if (save_output){
            writer.write(combined);
        }

        if (cv::waitKey(1) == 'q'){
            break;
        }
    }
    cv::destroyAllWindows();
    if (save_output){
        writer.release();
    }

    return 0;
}