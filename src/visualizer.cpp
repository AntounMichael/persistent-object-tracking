// Simple visualizer for detection JSON files
// Requires OpenCV (install with: sudo apt-get install libopencv-dev)
#include <opencv2/opencv.hpp>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <json.hpp>
#include "visualizer.hpp"

namespace fs = std::filesystem;
using json = nlohmann::json;

constexpr int IMG_SIZE = 800;
constexpr int RECT_THICKNESS = 2;
const cv::Scalar RECT_COLOR(0, 0, 255); // Red
const cv::Scalar BG_COLOR(255, 255, 255); // White

void draw_detections(const json& detections, cv::Mat& img) {
    for (const auto& det : detections) {
        int x = static_cast<int>((det["x"].get<double>() - det["width"].get<double>()/2) * IMG_SIZE);
        int y = static_cast<int>((det["y"].get<double>() - det["height"].get<double>()/2) * IMG_SIZE);
        int w = static_cast<int>(det["width"].get<double>() * IMG_SIZE);
        int h = static_cast<int>(det["height"].get<double>() * IMG_SIZE);
        cv::rectangle(img, cv::Rect(x, y, w, h), RECT_COLOR, RECT_THICKNESS);
    }
}

// Assign a color for each track id
cv::Scalar get_color(int id) {
    static std::map<int, cv::Scalar> color_map;
    if (color_map.count(id)) return color_map[id];
    // Generate a new color
    int r = (id * 77) % 256;
    int g = (id * 151) % 256;
    int b = (id * 211) % 256;
    color_map[id] = cv::Scalar(b, g, r);
    return color_map[id];
}

void run_visualization(const std::vector<Frame>& frames, const std::vector<std::vector<int>>& all_ids, const std::string& vis_dir) {
    namespace fs = std::filesystem;
    fs::create_directories(vis_dir);
    std::map<int, std::vector<cv::Point>> track_history;
    for (size_t i = 0; i < frames.size(); ++i) {
        const auto& frame = frames[i];
        const auto& ids = all_ids[i];
        cv::Mat img(IMG_SIZE, IMG_SIZE, CV_8UC3, BG_COLOR);
        for (size_t j = 0; j < frame.detections.size(); ++j) {
            const auto& det = frame.detections[j];
            int id = ids[j];
            int x = static_cast<int>((det.x - det.width/2) * IMG_SIZE);
            int y = static_cast<int>((det.y - det.height/2) * IMG_SIZE);
            int w = static_cast<int>(det.width * IMG_SIZE);
            int h = static_cast<int>(det.height * IMG_SIZE);
            cv::Scalar color = get_color(id);
            cv::rectangle(img, cv::Rect(x, y, w, h), color, RECT_THICKNESS);
            char label[32];
            snprintf(label, sizeof(label), "%d", id);
            cv::putText(img, label, cv::Point(x, y-5), cv::FONT_HERSHEY_SIMPLEX, 0.7, color, 2);
            // Save center for history
            int cx = static_cast<int>(det.x * IMG_SIZE);
            int cy = static_cast<int>(det.y * IMG_SIZE);
            track_history[id].push_back(cv::Point(cx, cy));
        }
        // Draw history trails
        for (const auto& [id, pts] : track_history) {
            for (size_t k = 1; k < pts.size(); ++k) {
                cv::line(img, pts[k-1], pts[k], get_color(id), 1);
            }
        }
        // Draw timestamp
        if (!frame.timestamp.empty()) {
            cv::putText(img, frame.timestamp, cv::Point(20, 40), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0,0,0), 2);
        }
        char out_name[64];
        snprintf(out_name, sizeof(out_name), "frame_%03zu.png", i);
        cv::imwrite((fs::path(vis_dir) / out_name).string(), img);
    }
}
