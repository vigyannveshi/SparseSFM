/**
 * @file landmark.cpp
 * @brief Implements the Landmark class declared in landmark.hpp.
 *
 * @author Rudraksha R. Bandodkar
 * @copyright Copyright (c) 2026 Rudraksha R. Bandodkar. MIT License, see
 *            LICENSE file in the project root for full terms.
 */

#include "landmark.hpp"

Landmark::Landmark(cv::Point3d position){
    position_ = position;
}

// setters
void Landmark::add_observation(const Observation& observation){
    observations_.push_back(observation);
}

void Landmark::update_position(const cv::Point3d& del_position){
    position_.x +=del_position.x;
    position_.y +=del_position.y;
    position_.z +=del_position.z;
}

void Landmark::set_position(const cv::Point3d& position){
    position_.x = position.x;
    position_.y = position.y;
    position_.z = position.z;
}

// getters
cv::Point3d Landmark::get_position() const{
    return position_;
}

const std::vector<Observation>& Landmark::get_observations() const{
    return observations_;
}