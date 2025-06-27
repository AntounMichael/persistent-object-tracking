#include <iostream>
#include <fstream>
#include <vector>
#include <random>
#include <cmath>
#include <chrono>
#include <iomanip>
#include <json.hpp>

using json = nlohmann::json;

// Structs for object and frame
struct Object {
    int id;
    double x, y;
    double width, height;
    double true_width, true_height;
    std::vector<std::pair<int, int>> dropouts; // (start_frame, end_frame)
};


// Helper: Euclidean distance
double dist(double x1, double y1, double x2, double y2) {
    return std::sqrt((x1-x2)*(x1-x2) + (y1-y2)*(y1-y2));
}

// Helper: Check if point is in rectangle
bool in_rect(double x, double y, double rx, double ry, double rw, double rh) {
    return (x >= rx && x <= rx+rw && y >= ry && y <= ry+rh);
}

// Helper: Generate ISO timestamp
std::string make_timestamp(int frame_id, int fps = 10) {
    int seconds = frame_id / fps;
    int ms = (frame_id % fps) * (1000 / fps);
    std::ostringstream oss;
    oss << "2025-03-24T18:" << std::setw(2) << std::setfill('0') << (seconds/60)
        << ":" << std::setw(2) << std::setfill('0') << (seconds%60)
        << "." << std::setw(6) << std::setfill('0') << (ms*1000);
    return oss.str();
}

// Main generator
int main() {
    // Parameters (can be made configurable)
    int num_objects = 15;
    int num_frames = 60;
    double min_object_distance = 0.15;
    double dropout_probability = 0.0;
    int dropout_min = 1, dropout_max = 3;
    double noise_stddev_pos = 0; //0.01;
    double noise_stddev_size = 0; //0.0025;
    double obj_size_min = 0.04, obj_size_max = 0.07;
    // Camera moves left to right across the plane
    double cam_w = 0.7, cam_h = 0.7;
    double cam_y0 = 0;
    double cam_y1 = 1.0; // Camera moves vertically from y0 to y1
    // Calculate the area traversed by the camera over all frames
    std::string output_path = "test_data/data/input_data.json";

    // Random number generator and distributions
    std::mt19937 rng(std::random_device{}());
    std::uniform_real_distribution<double> pos_dist(0.0, 1.0);
    std::uniform_real_distribution<double> size_dist(obj_size_min, obj_size_max);
    std::normal_distribution<double> noise_pos(0.0, noise_stddev_pos);

    // 1. Place objects within the camera's traversed rectangle (no clamping)
    std::vector<Object> objects;
    for (int i = 0; i < num_objects; ++i) {
        double x, y, w, h;
        int tries = 0;
        while (true) {
            x = pos_dist(rng) * cam_w; // x in [0, cam_w]
            y = cam_y0 + pos_dist(rng) * (cam_y1 + cam_h - cam_y0); // y in [cam_y0, cam_y1 + cam_h]
            w = size_dist(rng);
            h = size_dist(rng);
            bool ok = true;
            for (const auto& obj : objects) {
                if (dist(x, y, obj.x, obj.y) < min_object_distance)
                    ok = false;
            }
            if (ok) break;
            if (++tries > 1000) throw std::runtime_error("Can't place objects with given min distance");
        }
        objects.push_back({i, x, y, w, h, w, h, {}});
    }

    // 2. Assign dropouts
    std::uniform_real_distribution<double> drop_chance(0.0, 1.0);
    std::uniform_int_distribution<int> drop_len(dropout_min, dropout_max);
    for (auto& obj : objects) {
        int f = 0;
        while (f < num_frames) {
            if (drop_chance(rng) < dropout_probability) {
                int len = drop_len(rng);
                obj.dropouts.push_back({f, std::min(f+len-1, num_frames-1)});
                f += len;
            } else {
                ++f;
            }
        }
    }

    // 3. Generate frames
    std::vector<json> frames_json;
    for (int frame = 0; frame < num_frames; ++frame) {
        // Interpolate camera position
        double t = double(frame) / (num_frames-1);
        double cam_x = 0;
        double cam_y = cam_y0 + t * (cam_y1 - cam_y0);

        json frame_json;
        frame_json["frame_id"] = frame;
        frame_json["timestamp"] = make_timestamp(frame);

        json detections = json::array();
        for (const auto& obj : objects) {
            // Check dropout
            bool dropped = false;
            for (const auto& d : obj.dropouts) {
                if (frame >= d.first && frame <= d.second) {
                    dropped = true;
                    break;
                }
            }
            if (dropped) continue;
            // Check if in camera rect
            if (!in_rect(obj.x, obj.y, cam_x, cam_y, cam_w, cam_h)) continue;
            // Add noise
            double nx = obj.x + noise_pos(rng);
            double ny = obj.y + noise_pos(rng);
            std::normal_distribution<double> noise_size(0.0, noise_stddev_size * obj.true_width);
            double nw = obj.true_width + noise_size(rng);
            double nh = obj.true_height + noise_size(rng);
            detections.push_back({
                {"x", nx},
                {"y", ny - cam_y},
                {"width", nw},
                {"height", nh}
            });
        }
        frame_json["detections"] = detections;
        frames_json.push_back(frame_json);
    }

    // 4. Write to file
    std::ofstream out(output_path);
    out << std::setw(2) << frames_json << std::endl;
    std::cout << "Test data written to " << output_path << std::endl;
    return 0;
}