/**
 * @file frame.cpp
 * @brief Implements the Frame class declared in frame.hpp.
 *
 * @author Rudraksha R. Bandodkar
 * @copyright Copyright (c) 2026 Rudraksha R. Bandodkar. MIT License, see
 *            LICENSE file in the project root for full terms.
 */

#include "frame.hpp"

// handling static-id counter
int Frame::next_id_ = 0;

Frame::Frame(const std::string& image_path): image_path_(image_path), has_pose_(false), id_(next_id_++){}

// copy constructor
Frame::Frame(const Frame& other): 
       image_path_(other.image_path_), 
       id_(other.id_),
       has_pose_(other.has_pose_),
       keypoints_(other.keypoints_),
       descriptors_(other.descriptors_.clone()),
       R_(other.R_.clone()),
       t_(other.t_.clone()){}


// setters
void Frame::set_keypoints_and_descriptors(const std::vector<cv::KeyPoint>& keypoints,
                                    const cv::Mat& descriptors){
    keypoints_ = keypoints;
    descriptors_ = descriptors;
}

void Frame::set_pose(const cv::Mat& R, const cv::Mat& t){
    R_ = R.clone();
    t_ = t.clone();
    has_pose_ = true;
}

// getters
const std::vector<cv::KeyPoint>& Frame::get_keypoints() const{
    return keypoints_;
}

const cv::Mat& Frame::get_descriptors() const{
    return descriptors_;
}

const cv::Mat& Frame::get_R() const{
    return R_;
}

const cv::Mat& Frame::get_t() const{
    return t_;
}

bool Frame::get_has_pose() const{
    return has_pose_;
}

int Frame::get_id() const{
    return id_;
}

const std::string Frame::get_image_path() const{
    return image_path_;
}