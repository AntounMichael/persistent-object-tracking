#include "ransac.hpp"
#include <random>
#include <iostream>

RansacResult ransac_translation(const std::vector<Eigen::Vector2d>& src,
                                const std::vector<Eigen::Vector2d>& dst,
                                int iterations, double threshold, double max_offset) {
    int n = std::min(src.size(), dst.size());
    if (n < 2) return {Eigen::Vector2d(0,0), 0};
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> dis(0, n-1);
    int best_inliers = 0;
    Eigen::Vector2d best_t(0,0);
    for (int it = 0; it < iterations; ++it) {
        int i = dis(gen);
        Eigen::Vector2d t = dst[i] - src[i];
        if (t.norm() > max_offset) continue; // skip implausible translations
        int inliers = 0;
        for (int j = 0; j < n; ++j) {
            if ((dst[j] - (src[j] + t)).norm() < threshold) {
                ++inliers;
            }
        }
        if (inliers > best_inliers) {
            best_inliers = inliers;
            best_t = t;
        }
    }
    std::cout << "RANSAC: num observations = " << n << ", best inliers = " << best_inliers << std::endl;
    return {best_t, best_inliers};
}
