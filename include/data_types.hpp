#pragma once
#include <string>
#include <vector>

struct Detection {
    double x, y, width, height;
};

struct Frame {
    int frame_id;
    std::string timestamp;
    std::vector<Detection> detections;
};
