cmake_minimum_required(VERSION 3.10)
project(red-barn-robotics)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

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

target_link_libraries(red-barn-robotics ${OpenCV_LIBS})