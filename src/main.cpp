#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include "json.hpp"
#include "data_types.hpp"
#include "tracker.hpp"
#include "visualizer.hpp"
#include <filesystem>

using json = nlohmann::json;

std::vector<Frame> load_frames(const std::string& filename) {
    std::vector<Frame> frames;
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Failed to open file: " << filename << std::endl;
        return frames;
    }
    json j;
    file >> j;
    int frame_id = 0;
    for (const auto& frame_json : j) {
        Frame frame;
        frame.frame_id = frame_id++;
        // Optionally parse timestamp if present
        if (frame_json.contains("timestamp")) {
            frame.timestamp = frame_json["timestamp"].get<std::string>();
        }
        for (const auto& det_json : frame_json["detections"]) {
            Detection det;
            det.x = det_json["x"].get<double>();
            det.y = det_json["y"].get<double>();
            det.width = det_json["width"].get<double>();
            det.height = det_json["height"].get<double>();
            frame.detections.push_back(det);
        }
        frames.push_back(frame);
    }
    return frames;
}

std::string get_arg(int argc, char** argv, const std::string& flag, const std::string& def) {
    for (int i = 1; i < argc - 1; ++i) {
        if (std::string(argv[i]) == flag) return argv[i + 1];
    }
    return def;
}

int main(int argc, char** argv) {
    std::string input_file = get_arg(argc, argv, "--input", "../test_data/data/input_data.json");
    std::string output_file = get_arg(argc, argv, "--output", "output.json");
    std::string vis_dir = get_arg(argc, argv, "--vis-dir", "visualizations");
    std::filesystem::create_directories(vis_dir);
    auto frames = load_frames(input_file);
    Tracker tracker;
    json output_json = json::array();
    std::vector<std::vector<int>> all_ids;
    for (const auto& frame : frames) {
        std::cout << "*******Processing frame " << frame.frame_id << " with " << frame.detections.size() << " detections." << std::endl;
        std::vector<int> ids = tracker.process_frame_with_ids(frame);
        all_ids.push_back(ids);
        json frame_json;
        frame_json["frame_id"] = frame.frame_id;
        frame_json["timestamp"] = frame.timestamp;
        frame_json["tracked_objects"] = json::array();
        for (size_t i = 0; i < frame.detections.size(); ++i) {
            json det_json;
            det_json["id"] = ids[i];
            det_json["x"] = frame.detections[i].x;
            det_json["y"] = frame.detections[i].y;
            det_json["width"] = frame.detections[i].width;
            det_json["height"] = frame.detections[i].height;
            frame_json["tracked_objects"].push_back(det_json);
        }
        output_json.push_back(frame_json);
    }
    std::ofstream out(output_file);
    out << output_json.dump(2) << std::endl;
    std::cout << "Wrote output to " << output_file << std::endl;
    // Visualization step
    run_visualization(frames, all_ids, vis_dir);
    return 0;
}
