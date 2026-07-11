/**
 * @file test_frame_io.cpp
 * @brief Test executable verifying extract_frames and load_frames.
 *
 * @author Rudraksha R. Bandodkar
 * @copyright Copyright (c) 2026 Rudraksha R. Bandodkar. MIT License, see
 *            LICENSE file in the project root for full terms.
 */

#include "frame_io.hpp"

#include "frame.hpp"
#include <iostream>

const std::string OUTPUT_DIR = "../data/frames_3";
const std::string VID_PATH = "../data/vid_3.mp4";

void print_get(const Frame & frame)
{
    // getting values
    std::cout << "frame-id: "<<frame.get_id()<< std::endl;
    std::cout << "image-path: " << frame.get_image_path() << std::endl;
    std::cout << "Keypoints: " << frame.get_keypoints().size() << std::endl;
    std::cout << "Descriptors: " << frame.get_descriptors() << std::endl;
    std::cout << "R: " << frame.get_R() << std::endl;
    std::cout << "t: " << frame.get_t() << std::endl;
    std::cout << "has_pose:" << frame.get_has_pose() << std::endl;
}

int main(){
    // create frames
    bool extracted = extract_frames(VID_PATH, OUTPUT_DIR);
    
    std::vector<Frame> frames;
    if (extracted){
        frames = load_frames(OUTPUT_DIR);
    }

    std::cout<<"Load frames: \n"<<std::endl;
    for (int i = 0; i<=5; i++)
    {
        Frame f = frames.at(i);
        print_get(f);
        std::cout<<"\n"<<std::endl;
    }

    return 0;
}