#pragma once
#include "data_types.hpp"
#include <vector>
#include "kalman_filter.hpp"

struct Track {
    int id;
    int missed_count;
    Detection state; // For legacy/simple access
    KalmanFilter2D kf; // Kalman filter for this track
};

class Tracker {
public:
    Tracker();
    void process_frame(const Frame& frame);
    std::vector<int> process_frame_with_ids(const Frame& frame);
private:
    std::vector<Track> tracks;
    int next_id;
    double last_time = 0; // Track last timestamp for dt computation
    void predict_tracks(double dt);
    void update_tracks(const std::vector<Detection>& detections, double dt);
    std::vector<int> update_tracks_with_ids(const std::vector<Detection>& detections, double dt);
    void remove_old_tracks();
};

// Stubs for RANSAC, Kalman, Hungarian
double estimate_global_translation_ransac(const std::vector<Track>&, const std::vector<Detection>&);
int hungarian_assignment(const std::vector<std::vector<double>> &cost_matrix, std::vector<int> &assignment);
void kalman_predict(Track& track, double dt);
void kalman_update(Track& track, const Detection& detection);
