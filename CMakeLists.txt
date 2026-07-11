# Project setup
cmake_minimum_required(VERSION 3.10)
project(sparse_sfm)

# Compiler standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find packages
find_package(OpenCV REQUIRED)
find_package(Pangolin REQUIRED)

# include paths
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${CMAKE_SOURCE_DIR}/include)

# shared library from all pipeline sources
add_library(sparse_sfm_lib
    src/camera.cpp 
    src/calib_io.cpp
    src/frame_io.cpp
    src/frame.cpp
    src/landmark.cpp
    src/orb_extractor.cpp
    src/orb_matcher.cpp
    src/sift_matcher.cpp
    src/sift_extractor.cpp
    src/initializer.cpp
    src/triangulator.cpp
    src/visualizer.cpp
    src/features.cpp
    src/pose.cpp
    src/p3p_solver.cpp
    src/pipeline.cpp
    src/bundle_adjustments.cpp
)
target_link_libraries(sparse_sfm_lib ${OpenCV_LIBS} ${Pangolin_LIBRARIES})

# option to toggle building test executables
option(BUILD_TESTS "Build test executables" ON)


# build targets

### utilities
add_executable(calibrate utils/calibrate.cpp)
add_executable(undistort_visualization utils/undistort_visualization.cpp)

target_link_libraries(calibrate ${OpenCV_LIBS})
target_link_libraries(undistort_visualization ${OpenCV_LIBS})

### tests
if(BUILD_TESTS)
    add_executable(test_camera tests/test_camera.cpp)
    add_executable(test_frame tests/test_frame.cpp)
    add_executable(test_landmark tests/test_landmark.cpp)
    add_executable(test_frame_io tests/test_frame_io.cpp)
    add_executable(test_orb_ext_match tests/test_orb_ext_match.cpp)
    add_executable(test_sift_ext_match tests/test_sift_ext_match.cpp)
    add_executable(test_initialization tests/test_initialization.cpp)
    add_executable(test_triangulator tests/test_triangulator.cpp)
    add_executable(test_p3p tests/test_p3p.cpp)
    add_executable(test_global_ba tests/test_global_ba.cpp)

    target_link_libraries(test_camera sparse_sfm_lib)
    target_link_libraries(test_frame sparse_sfm_lib)
    target_link_libraries(test_landmark sparse_sfm_lib)
    target_link_libraries(test_frame_io sparse_sfm_lib)
    target_link_libraries(test_orb_ext_match sparse_sfm_lib)
    target_link_libraries(test_sift_ext_match sparse_sfm_lib)
    target_link_libraries(test_initialization sparse_sfm_lib)
    target_link_libraries(test_triangulator sparse_sfm_lib)
    target_link_libraries(test_p3p sparse_sfm_lib)
    target_link_libraries(test_global_ba sparse_sfm_lib)

endif()

# Driver
add_executable(sparse_sfm main.cpp)
target_link_libraries(sparse_sfm sparse_sfm_lib)

# Default to an optimized Release build if no build type was specified on
# the command line (e.g. via -DCMAKE_BUILD_TYPE=Debug). Without this, CMake
# leaves CMAKE_BUILD_TYPE empty, which compiles with no optimization flags
# at all, noticeably slower at runtime. This only applies the default when
# nothing was already chosen, so an explicit Debug build still wins.
if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()