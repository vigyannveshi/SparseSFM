/**
 * @file calib_io.hpp
 * @brief Reads calibration output (calib.txt) and constructs a Camera.
 *
 * @author Rudraksha R. Bandodkar
 * @copyright Copyright (c) 2026 Rudraksha R. Bandodkar. MIT License, see
 *            LICENSE file in the project root for full terms.
 */

#ifndef CALIB_IO_HPP
#define CALIB_IO_HPP

#include "camera.hpp"
#include <string>

Camera read_calib(const std::string& calib_path);

#endif