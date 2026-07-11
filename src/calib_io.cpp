/**
 * @file calib_io.cpp
 * @brief Implements read_calib declared in calib_io.hpp.
 *
 * @author Rudraksha R. Bandodkar
 * @copyright Copyright (c) 2026 Rudraksha R. Bandodkar. MIT License, see
 *            LICENSE file in the project root for full terms.
 */

#include "calib_io.hpp"
#include <fstream>
#include <iostream>

Camera read_calib(const std::string& calib_path)
{
    // open calibration file and check if opened
    std::ifstream calib(calib_path);

    if(!calib.is_open()){
        throw std::runtime_error("Could not open calib file: " + calib_path);
    }

    // declare variables to extract parameters
    double fx, fy, cx, cy, s, k1, k2, p1, p2, k3;

    // extract parameters from file
    calib >> fx >> fy >> cx >> cy >> s >> k1 >> k2 >> p1 >> p2 >> k3;

    // create the camera object
    Camera camera(fx, fy, cx, cy, s, k1, k2, p1, p2, k3);
    
    return camera;
}