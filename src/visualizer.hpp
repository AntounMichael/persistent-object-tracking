#pragma once
#include <string>
#include <vector>
#include "data_types.hpp"

// all_ids[i][j] is the track id for detection j in frame i
void run_visualization(const std::vector<Frame>& frames, const std::vector<std::vector<int>>& all_ids, const std::string& vis_dir);

