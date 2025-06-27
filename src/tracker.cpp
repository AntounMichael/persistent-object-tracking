#include "tracker.hpp"
#include "kalman_filter.hpp"
#include "ransac.hpp"
#include "munkres.h"
#include "matrix.h"
#include <iostream>
#include <limits>
#include <iomanip>
#include <sstream>
#include <ctime>
#include <algorithm>
#include <set>

Tracker::Tracker() : next_id(0), last_time(-1.0) {}

static double parse_time(const std::string& timestamp) {
    // Parse ISO 8601 using std::get_time
    std::tm tm = {};
    double seconds = 0.0;
    char dot;
    int micro = 0;
    std::istringstream ss(timestamp);
    ss >> std::get_time(&tm, "%Y-%m-%dT%H:%M:%S");
    if (ss.fail()) return 0.0;
    if (ss.peek() == '.') {
        ss >> dot;
        std::string micro_str;
        ss >> micro_str;
        micro = std::stoi(micro_str.substr(0, 6));
    }
    seconds = tm.tm_hour * 3600 + tm.tm_min * 60 + tm.tm_sec + micro / 1e6;
    return seconds;
}

void Tracker::process_frame(const Frame& frame) {
    double current_time = frame.timestamp.empty() ? -1.0 : parse_time(frame.timestamp);
    double dt = 1.0;
    if (last_time >= 0.0 && current_time >= 0.0) {
        dt = current_time - last_time;
        if (dt <= 0.0) dt = 1.0; // fallback if timestamps are not increasing
    }
    last_time = current_time;
    predict_tracks(dt);
    const auto& detections = frame.detections;
    update_tracks(detections, dt);
    remove_old_tracks();
}

std::vector<int> Tracker::process_frame_with_ids(const Frame& frame) {
    double current_time = frame.timestamp.empty() ? -1.0 : parse_time(frame.timestamp);
    double dt = 1.0;
    if (last_time > 0.0 && current_time >= 0.0) {
        dt = current_time - last_time;
        if (dt <= 0.0) dt = 1.0;
    }
    last_time = current_time;
    predict_tracks(dt);
    const auto& detections = frame.detections;
    std::vector<int> ids = update_tracks_with_ids(detections, dt);
    remove_old_tracks();
    return ids;
}

void Tracker::predict_tracks(double dt) {
    for (auto& track : tracks) {
        kalman_predict(track, dt);
    }
}

void Tracker::update_tracks(const std::vector<Detection>& detections, double dt) {
    size_t num_detections = detections.size();
    if (num_detections == 0) {
        for (auto& track : tracks) {
            track.missed_count++;
        }
        return;
    }
    // TODO: RANSAC/global translation, cost matrix, Hungarian, update logic
    // For now, just match detections to tracks 1:1 if possible
    size_t n = std::min(tracks.size(), detections.size());
    for (size_t i = 0; i < n; ++i) {
        kalman_update(tracks[i], detections[i]);
        tracks[i].missed_count = 0;
    }
    // Unmatched tracks
    for (size_t i = n; i < tracks.size(); ++i) {
        tracks[i].missed_count++;
    }
    // Unmatched detections
    for (size_t i = n; i < detections.size(); ++i) {
        Track new_track;
        new_track.id = next_id++;
        new_track.missed_count = 0;
        new_track.state = detections[i];
        tracks.push_back(new_track);
    }
}

std::vector<int> Tracker::update_tracks_with_ids(const std::vector<Detection>& detections, double dt) {
    std::vector<int> ids(detections.size(), -1);
    size_t num_tracks = tracks.size();
    size_t num_detections = detections.size();
    std::set<int> updated_tracks, unobserved_tracks, dead_tracks;
    std::vector<double> track_moves(num_tracks, 0.0);
    std::cout << "[DEBUG] num_tracks: " << num_tracks << ", num_detections: " << num_detections << std::endl;
    if (num_tracks == 0) {
        // No tracks: create a new track for each detection
        for (size_t j = 0; j < num_detections; ++j) {
            std::cout << "[DEBUG] No tracks. Creating new track for detection " << j << " with ID " << next_id << std::endl;
            Track new_track;
            new_track.id = next_id++;
            new_track.missed_count = 0;
            new_track.state = detections[j];
            new_track.kf.init(detections[j].x, detections[j].y, 0, 0);
            tracks.push_back(new_track);
            ids[j] = new_track.id;
        }
        return ids;
    }
    if (num_detections == 0) {
        for (auto& track : tracks) track.missed_count++;
        for (const auto& track : tracks) unobserved_tracks.insert(track.id);
        std::cout << "No detections. All tracks predict forward. Unobserved: ";
        for (int id : unobserved_tracks) std::cout << id << " ";
        std::cout << std::endl;
        return ids;
    }
    // Apply RANSAC global translation if enough detections
    Eigen::Vector2d translation(0, 0);
    bool ran_ransac = false;
    bool single_point_translation = false;
    // Special non-RANSAC global offset: if not enough for RANSAC, but at least one track and one detection
    if (num_tracks >= 2 && num_detections >= 2) {
        std::vector<Eigen::Vector2d> src, dst;
        size_t n = std::min(num_tracks, num_detections);
        for (size_t i = 0; i < n; ++i) {
            src.emplace_back(tracks[i].state.x, tracks[i].state.y);
            dst.emplace_back(detections[i].x, detections[i].y);
        }
        auto ransac_result = ransac_translation(src, dst);
        translation = ransac_result.translation;
        ran_ransac = true;
        std::cout << "RANSAC run: translation = [" << translation.x() << ", " << translation.y() << "]\n";
    } else if (num_tracks >= 1 && num_detections >= 1) {
        // Find the track-detection pair with the lowest offset
        double min_dist = std::numeric_limits<double>::max();
        int best_track = -1, best_det = -1;
        for (size_t i = 0; i < num_tracks; ++i) {
            for (size_t j = 0; j < num_detections; ++j) {
                double dx = detections[j].x - tracks[i].state.x;
                double dy = detections[j].y - tracks[i].state.y;
                double dist = std::sqrt(dx*dx + dy*dy);
                if (dist < min_dist) {
                    min_dist = dist;
                    best_track = (int)i;
                    best_det = (int)j;
                }
            }
        }
        double single_pt_threshold = 0.2; // You can tune this
        if (min_dist < single_pt_threshold && best_track >= 0 && best_det >= 0) {
            translation = Eigen::Vector2d(detections[best_det].x - tracks[best_track].state.x,
                                          detections[best_det].y - tracks[best_track].state.y);
            single_point_translation = true;
            std::cout << "Special global offset (best pair: track " << tracks[best_track].id << ", detection " << best_det << ") applied: ["
                      << translation.x() << ", " << translation.y() << "]\n";
            // Optionally update the best track directly (not strictly necessary since translation is applied to all below)
        } else {
            std::cout << "Special global offset NOT applied (min distance " << min_dist << " > threshold " << single_pt_threshold << ")\n";
        }
    }
    // Apply translation to all predicted tracks
    for (auto& track : tracks) {
        track.state.x += translation.x();
        track.state.y += translation.y();
    }
    if (!ran_ransac && !single_point_translation) {
        std::cout << "RANSAC not run (not enough tracks/detections)\n";
    }
    // Build cost matrix (Euclidean distance + weighted width/height dissimilarity)
    double wh_weight = 1.0; // Set to >0 to enable width/height dissimilarity
    std::vector<std::vector<double>> cost(num_tracks, std::vector<double>(num_detections, 1e6));
    for (size_t i = 0; i < num_tracks; ++i) {
        for (size_t j = 0; j < num_detections; ++j) {
            double dx = tracks[i].state.x - detections[j].x;
            double dy = tracks[i].state.y - detections[j].y;
            double dpos = std::sqrt(dx*dx + dy*dy);
            double dwh = std::abs(tracks[i].state.width - detections[j].width) +
                         std::abs(tracks[i].state.height - detections[j].height);
            cost[i][j] = dpos + wh_weight * dwh;
        }
    }
    // Log the cost matrix
    std::cout << "Cost matrix (track x detection):\n";
    for (size_t i = 0; i < num_tracks; ++i) {
        std::cout << "Track " << i << " (ID " << tracks[i].id << "): ";
        for (size_t j = 0; j < num_detections; ++j) {
            std::cout << std::fixed << std::setprecision(3) << cost[i][j] << " ";
        }
        std::cout << std::endl;
    }
    // Expose new track penalty as a variable
    const double new_track_penalty = 0.35; // <-- Tune this as needed
    // Use a robust assignment matrix size: tracks + detections
    size_t dim = num_tracks + num_detections;
    // Build augmented cost matrix
    std::vector<std::vector<double>> cost_aug(dim, std::vector<double>(dim, new_track_penalty));
    // Fill real costs in top-left block
    for (size_t i = 0; i < num_tracks; ++i) {
        for (size_t j = 0; j < num_detections; ++j) {
            cost_aug[i][j] = cost[i][j];
        }
    }
    // Use munkres-cpp for assignment
    Munkres<double> munkres;
    Matrix<double> munkres_matrix(dim, dim);
    for (size_t i = 0; i < dim; ++i)
        for (size_t j = 0; j < dim; ++j)
            munkres_matrix(i, j) = cost_aug[i][j];
    munkres.solve(munkres_matrix);

    std::vector<bool> detection_assigned(num_detections, false);
    // Combined assignment extraction and track update loop
    const double threshold = 0.2; // threshold for assignment (tune as needed)
    for (size_t i = 0; i < num_tracks; ++i) {
        int assigned_det = -1;
        double assigned_cost = new_track_penalty;
        // Find assignment for this track from the munkres result
        for (size_t j = 0; j < num_detections; ++j) {
            if (munkres_matrix(i, j) == 0 && cost_aug[i][j] < new_track_penalty) {
                assigned_det = j;
                assigned_cost = cost[i][j];
                break;
            }
        }
        if (assigned_det >= 0 && assigned_cost < threshold) {
            // Accept assignment, update track
            double dx = tracks[i].state.x - detections[assigned_det].x;
            double dy = tracks[i].state.y - detections[assigned_det].y;
            double move = std::sqrt(dx*dx + dy*dy);
            track_moves[i] = move;
            kalman_update(tracks[i], detections[assigned_det]);
            tracks[i].missed_count = 0;
            ids[assigned_det] = tracks[i].id;
            updated_tracks.insert(tracks[i].id);
            detection_assigned[assigned_det] = true;
        } else {
            // No valid assignment or cost too high
            tracks[i].missed_count++;
            unobserved_tracks.insert(tracks[i].id);
        }
    }
    // Create new tracks for unassigned detections (i.e., those assigned to dummy rows)
    for (size_t j = 0; j < num_detections; ++j) {
        bool assigned = false;
        for (size_t i = 0; i < num_tracks; ++i) {
            if (ids[j] == tracks[i].id) {
                assigned = true;
                break;
            }
        }
        if (!assigned) {
            // This detection was not assigned to any existing track, so create a new track
            std::cout << "Detection " << j << " not assigned to any track. Creating new track with ID " << next_id << std::endl;
            Track new_track;
            new_track.id = next_id++;
            new_track.missed_count = 0;
            new_track.state = detections[j];
            new_track.kf.init(detections[j].x, detections[j].y, 0, 0);
            tracks.push_back(new_track);
            ids[j] = new_track.id;
        }
    }
    // Log updated tracks
    if (!updated_tracks.empty()) {
        std::cout << "Updated tracks: ";
        for (size_t i = 0; i < num_tracks; ++i) {
            if (updated_tracks.count(tracks[i].id)) {
                std::cout << tracks[i].id << " (move: " << track_moves[i] << ") ";
            }
        }
        std::cout << std::endl;
    }
    // Log unobserved tracks
    if (!unobserved_tracks.empty()) {
        std::cout << "Unobserved tracks: ";
        for (int id : unobserved_tracks) std::cout << id << " ";
        std::cout << std::endl;
    }
    // Log dead tracks (after remove_old_tracks)
    std::set<int> before;
    for (const auto& t : tracks) before.insert(t.id);
    // Remove old tracks
    const int max_age = 5;
    auto old_size = tracks.size();
    tracks.erase(std::remove_if(tracks.begin(), tracks.end(),
        [max_age](const Track& t) { return t.missed_count > max_age; }), tracks.end());
    if (tracks.size() < old_size) {
        std::set<int> after;
        for (const auto& t : tracks) after.insert(t.id);
        for (int id : before) {
            if (!after.count(id)) dead_tracks.insert(id);
        }
        if (!dead_tracks.empty()) {
            std::cout << "Dead tracks: ";
            for (int id : dead_tracks) std::cout << id << " ";
            std::cout << std::endl;
        }
    }
    return ids;
}

void Tracker::remove_old_tracks() {
    const int max_age = 5;
    tracks.erase(std::remove_if(tracks.begin(), tracks.end(),
        [max_age](const Track& t) { return t.missed_count > max_age; }), tracks.end());
}

double estimate_global_translation_ransac(const std::vector<Track>& tracks, const std::vector<Detection>& detections) {
    std::vector<Eigen::Vector2d> src, dst;
    size_t n = std::min(tracks.size(), detections.size());
    for (size_t i = 0; i < n; ++i) {
        src.emplace_back(tracks[i].state.x, tracks[i].state.y);
        dst.emplace_back(detections[i].x, detections[i].y);
    }
    auto result = ransac_translation(src, dst);
    return result.translation.norm(); // or return translation vector if needed
}

int hungarian_assignment(const std::vector<std::vector<double>> &cost_matrix, std::vector<int> &assignment) {
    // Stub
    return 0;
}

void kalman_predict(Track& track, double dt) {
    double x_before = track.kf.getX();
    double y_before = track.kf.getY();
    track.kf.predict(dt);
    double x_after = track.kf.getX();
    double y_after = track.kf.getY();
    std::cout << "Kalman predict for track " << track.id << ": (" << x_before << ", " << y_before << ") -> (" << x_after << ", " << y_after << ")\n";
    track.state.x = x_after;
    track.state.y = y_after;
    // Optionally update width/height if you want to track them too
}

void kalman_update(Track& track, const Detection& detection) {
    if (!track.kf.getX() && !track.kf.getY()) {
        // Not initialized
        track.kf.init(detection.x, detection.y, 0, 0);
    } else {
        track.kf.update(detection.x, detection.y);
    }
    track.state = detection;
}
