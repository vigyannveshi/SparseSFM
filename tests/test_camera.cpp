/**
 * @file test_camera.cpp
 * @brief Test executable verifying Camera construction against real calib.txt values.
 *
 * @author Rudraksha R. Bandodkar
 * @copyright Copyright (c) 2026 Rudraksha R. Bandodkar. MIT License, see
 *            LICENSE file in the project root for full terms.
 */

#include "camera.hpp"

#include <iostream>
#include <fstream>

int main()
{
    Camera cam(1326.65, 1323.59, 552.553, 934.213, 0,
           0.151409, -1.1273, -6e-05, 1.75e-04, 2.02308);

    std::cout << "K = " << cam.get_K() << std::endl;
    std::cout << "D = " << cam.get_D() << std::endl;
    return 0;
}