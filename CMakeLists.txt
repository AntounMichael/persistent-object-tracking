cmake_minimum_required(VERSION 3.10)
project(red-barn-robotics)

find_package(PkgConfig REQUIRED)
pkg_check_modules(OPENCV REQUIRED opencv4)
include_directories(${OPENCV_INCLUDE_DIRS})
link_directories(${OPENCV_LIBRARY_DIRS})

# Add Eigen include directory
include_directories(${CMAKE_SOURCE_DIR}/external/eigen-3.4.0)
include_directories(${CMAKE_SOURCE_DIR}/include)
include_directories(${CMAKE_SOURCE_DIR}/external/munkres-cpp/src)

add_executable(red-barn-robotics
    src/main.cpp
    src/tracker.cpp
    src/kalman_filter.cpp
    src/ransac.cpp
    src/visualizer.cpp
)

add_executable(generate_test_data test_data/generate_test_data.cpp)

target_link_libraries(red-barn-robotics ${OPENCV_LIBRARIES})

add_executable(tracking-solution
    src/main.cpp
    src/tracker.cpp
    src/kalman_filter.cpp
    src/ransac.cpp
    src/visualizer.cpp
)

target_link_libraries(tracking-solution ${OPENCV_LIBRARIES})
