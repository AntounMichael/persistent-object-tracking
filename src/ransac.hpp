#pragma once
#include <vector>
#include <Eigen/Dense>
#include "data_types.hpp"
#include "tracker.hpp"

struct RansacResult {
    Eigen::Vector2d translation;
    int inliers;
};

RansacResult ransac_translation(const std::vector<Eigen::Vector2d>& src,
                                const std::vector<Eigen::Vector2d>& dst,
                                int iterations = 100, double threshold = .20, double max_offset = 0.05);
