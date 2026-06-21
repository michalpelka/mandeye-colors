#include "Trajectory.h"
#include <fstream>
#include <sstream>
#include <algorithm>
#include <cmath>

// Multiply 4×4 row-major M by 3×4 row-major pose T → result stored in T (in-place).
// T is treated as a 4×4 with bottom row [0,0,0,1].
static void applyMRP(const float M[16], float T[12]) {
    float out[12];
    for (int r = 0; r < 3; r++) {
        // rotation columns 0..2
        for (int c = 0; c < 3; c++)
            out[r*4+c] = M[r*4+0]*T[0*4+c] + M[r*4+1]*T[1*4+c] + M[r*4+2]*T[2*4+c];
        // translation column 3
        out[r*4+3] = M[r*4+0]*T[3] + M[r*4+1]*T[7] + M[r*4+2]*T[11] + M[r*4+3];
    }
    for (int i = 0; i < 12; i++) T[i] = out[i];
}

bool Trajectory::loadCSV(const std::string& path, const float* mrpTransform) {
    std::ifstream f(path);
    if (!f) return false;

    std::string line;
    std::getline(f, line); // skip header

    while (std::getline(f, line)) {
        if (line.empty()) continue;
        std::istringstream ss(line);
        TrajPose p;
        ss >> p.ts_ns;
        for (int i = 0; i < 12; i++) ss >> p.m[i];
        if (!ss) continue;
        if (mrpTransform) applyMRP(mrpTransform, p.m);
        poses.push_back(p);
    }
    return true;
}

void Trajectory::sort() {
    std::sort(poses.begin(), poses.end(),
              [](const TrajPose& a, const TrajPose& b){ return a.ts_ns < b.ts_ns; });
}

const TrajPose* Trajectory::nearest(int64_t ts_ns) const {
    if (poses.empty()) return nullptr;
    auto it = std::lower_bound(poses.begin(), poses.end(), ts_ns,
        [](const TrajPose& p, int64_t t){ return p.ts_ns < t; });
    if (it == poses.end()) return &poses.back();
    if (it == poses.begin()) return &poses.front();
    auto prev = std::prev(it);
    return (std::abs(it->ts_ns - ts_ns) < std::abs(prev->ts_ns - ts_ns)) ? &*it : &*prev;
}