/**
 * @file test_landmark.cpp
 * @brief Test executable verifying Landmark construction and observation tracking.
 *
 * @author Rudraksha R. Bandodkar
 * @copyright Copyright (c) 2026 Rudraksha R. Bandodkar. MIT License, see
 *            LICENSE file in the project root for full terms.
 */

#include "landmark.hpp"

#include <iostream>

void print_get(const Landmark& lm){
    std::cout << "position:" <<lm.get_position()<<std::endl;
    const std::vector<Observation>& observations = lm.get_observations();
    int i = 0;
    for (auto obs:observations){
        std::cout << "obs " << i <<": "<<"frame_id: "<< obs.frame_idx << ", keypoint_idx: " << obs.keypoint_idx << std::endl;
        i+=1;
    }
}

int main()
{
    // dummy data
    cv::Point3d dummy_point{2.0,1.0,3.0};
    Observation obs1{1,3};
    Observation obs2{3,5};
    Observation obs3{5,7};
    Observation obs4{2,10};

    Landmark lm(dummy_point);

    // before setting
    print_get(lm);

    // setting
    lm.add_observation(obs1);
    lm.add_observation(obs2);
    lm.add_observation(obs3);
    lm.add_observation(obs4);

    // after setting
    print_get(lm);
    
    return 0;
}