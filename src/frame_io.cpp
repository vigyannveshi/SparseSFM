/**
 * @file frame_io.cpp
 * @brief Implements extract_frames and load_frames declared in frame_io.hpp.
 *
 * @author Rudraksha R. Bandodkar
 * @copyright Copyright (c) 2026 Rudraksha R. Bandodkar. MIT License, see
 *            LICENSE file in the project root for full terms.
 */

#include "frame_io.hpp"

#include <opencv2/opencv.hpp>
#include <filesystem>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <algorithm>

// constants
#include "constants.hpp"

namespace fs = std::filesystem;

// extracting frames
bool extract_frames(const std::string& video_path,
                    const std::string& output_dir){
    // check if output directory exists and is non-empty - if so skip extraction
    if (fs::exists(output_dir) && !fs::is_empty(output_dir)){
        std::cout << "Frames already extracted in: " << output_dir <<" , skipping.\n";
        return true;
    }

    // create output_dir if it isn't existent
    if (!fs::exists(output_dir)){
        fs::create_directories(output_dir);
    }

    // creating video cap
    cv::VideoCapture cap(video_path);
    if (!cap.isOpened()){
        std::cerr << "Error: could not open video: "<< video_path << std::endl;
        return false;
    }
    int total_frames = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_COUNT));
    int total_sampled = (total_frames + FRAMES_SAMPLING_INTERVAL - 1) / FRAMES_SAMPLING_INTERVAL;

    // writing frames
    cv::Mat frame;
    int frame_idx = 0;
    int saved_count = 0;

    while (cap.read(frame)){
        if (frame_idx % FRAMES_SAMPLING_INTERVAL == 0){
            // build zero-padded filename,e.g: frame_000042.png
            std::ostringstream oss;
            oss << "/frame_" << std::setw(ZERO_PADDING) << std::setfill('0') << saved_count <<".png";
            
            std::string filename = output_dir + oss.str();
            // writing frame to disk, check success
            bool success = cv::imwrite(filename, frame);

            if (!success){
                std::cerr << "Error: could not write frame: "<< filename << std::endl;
            }
            
            saved_count ++;
            std::cout << "\r(" << saved_count << "/" << total_sampled << ") frames extracted" << std::flush;
        }
        frame_idx ++;
    }
 
    std::cout << "\nExtracted " << saved_count << " frames to: " << output_dir << std::endl;
    return true;
}

std::vector<Frame> load_frames(const std::string& frames_dir)
{
    // check if frames_dir exists
    if (!fs::exists(frames_dir))
    {
        std::cerr << "Error: "<<frames_dir<<" doesn't exist"<<std::endl;
        return {};
    }

    // collect all .png from the frames_dir
    std::vector<fs::path> paths;
    for (const auto& entry :  fs::directory_iterator(frames_dir))
    {
        if (entry.path().extension() == ".png")
        {
            paths.push_back(entry);
        }
    }

    // sort paths: zero-padding makes alphabetical sort == numeric sort
    std::sort(paths.begin(), paths.end());

    // construct Frame objects in order
    std::vector<Frame> frames;
    for (const auto& p: paths){
        Frame f(p.string());
        frames.push_back(f);
    }

    std::cout << "Loaded " <<frames.size() << " frames from: " << frames_dir << std::endl;
    return frames; 
}