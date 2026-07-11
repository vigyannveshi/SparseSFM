/**
 * @file test_frame.cpp
 * @brief Test executable verifying Frame construction, setters, and id logic.
 *
 * @author Rudraksha R. Bandodkar
 * @copyright Copyright (c) 2026 Rudraksha R. Bandodkar. MIT License, see
 *            LICENSE file in the project root for full terms.
 */

#include "frame.hpp"

#include <iostream>

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
    // dummy data
    std::vector<cv::KeyPoint> dummy_keypoints;
    cv::Mat dummy_descriptors;
    cv::Mat dummy_R = cv::Mat::eye(3,3,CV_64F);
    cv::Mat dummy_t = cv::Mat::zeros(3,1,CV_64F);

    // create frame object
    Frame frame("/data/images/image_1.png");

    // before setting
    print_get(frame);

    // setting values
    frame.set_keypoints_and_descriptors(dummy_keypoints,dummy_descriptors);
    frame.set_pose(dummy_R, dummy_t);

    // after setting
    print_get(frame);

    // testing id's
    std::cout<<"\n---Testing id-logic---\n"<<std::endl;
    for (int i = 0; i <= 5; i++)
    {
        Frame f("/data/images/image" + std::to_string(i+2) + ".png");
        print_get(f);
        std::cout<<std::endl;
    }

    return 0;
}