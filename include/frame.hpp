/**
 * @file frame.hpp
 * @brief Declares the Frame class, representing a single image in the
 *        sequence. Holds the image path, ORB keypoints and descriptors,
 *        and pose (R, t) once known. Data arrives in stages via setters
 *        as later pipeline modules process the frame.
 *
 * @author Rudraksha R. Bandodkar
 * @copyright Copyright (c) 2026 Rudraksha R. Bandodkar. MIT License, see
 *            LICENSE file in the project root for full terms.
 */

#ifndef FRAME_HPP
#define FRAME_HPP

#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

class Frame{
    public:
        Frame(const std::string& image_path);

        // copy constructor
        Frame(const Frame& other);

        // setters
        void set_keypoints_and_descriptors(const std::vector<cv::KeyPoint>& keypoints,
                                            const cv::Mat& descriptors);
        void set_pose(const cv::Mat& R, const cv::Mat& t);

        // getters
        const std::vector<cv::KeyPoint>& get_keypoints() const;
        const cv::Mat& get_descriptors() const;
        const cv::Mat& get_R() const;
        const cv::Mat& get_t() const;
        bool get_has_pose() const;
        int get_id() const;
        const std::string get_image_path() const;

    private:
        std::string image_path_;
        std::vector<cv::KeyPoint> keypoints_;
        cv::Mat descriptors_;

        cv::Mat R_;
        cv::Mat t_;
        bool has_pose_;

        // id
        int id_;

        // static counter for id
        static int next_id_;
};

#endif