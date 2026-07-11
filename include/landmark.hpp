/**
 * @file landmark.hpp
 * @brief Declares the Landmark class, representing a single triangulated
 *        3D point in the reconstruction. Holds the point's position and
 *        the list of observations, frame and keypoint index pairs, that
 *        saw it. Position is known at construction from triangulation;
 *        observations are added incrementally as later frames match into
 *        this same landmark.
 *
 * @author Rudraksha R. Bandodkar
 * @copyright Copyright (c) 2026 Rudraksha R. Bandodkar. MIT License, see
 *            LICENSE file in the project root for full terms.
 */

#ifndef LANDMARK_HPP
#define LANDMARK_HPP

#include <opencv2/opencv.hpp>
#include <vector>

struct Observation{
    int frame_idx;
    int keypoint_idx;
};

class Landmark{
    public:
        Landmark(cv::Point3d position);

        // setters
        void add_observation(const Observation& observation);
        void update_position(const cv::Point3d& del_position);
        void set_position(const cv::Point3d& position);

        // getters
        cv::Point3d get_position() const;

        const std::vector<Observation>& get_observations() const;


    private:
        cv::Point3d position_;
        std::vector<Observation> observations_;
};

#endif