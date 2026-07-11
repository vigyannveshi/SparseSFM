/**
 * @file frame_io.hpp
 * @brief Frame I/O utilities. extract_frames writes sampled video frames
 *        to disk as zero-padded PNGs (one-time). load_frames reads them
 *        back and constructs an ordered vector of Frame objects.
 *
 * @author Rudraksha R. Bandodkar
 * @copyright Copyright (c) 2026 Rudraksha R. Bandodkar. MIT License, see
 *            LICENSE file in the project root for full terms.
 */

#ifndef FRAME_IO_HPP
#define FRAME_IO_HPP

#include <string>
#include <vector>
#include "frame.hpp"

bool extract_frames(const std::string& video_path, const std::string& output_dir);

std::vector<Frame> load_frames(const std::string& output_dir); 

#endif