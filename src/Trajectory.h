#pragma once
#include <vector>
#include <string>
#include <cstdint>

// One LiDAR pose from the trajectory CSV.
// m[0..11] = row-major 3×4 T_world_lidar:
//   [ m0  m1  m2  m3  ]
//   [ m4  m5  m6  m7  ]
//   [ m8  m9  m10 m11 ]
// p_world = R * p_lidar + t  (t = m[3], m[7], m[11])
struct TrajPose {
    int64_t ts_ns = 0;
    float   m[12] = {};
};

struct Trajectory {
    std::vector<TrajPose> poses;  // sorted by ts_ns

    // Load one trajectory_lio_N.csv. Appends to poses.
    // If mrpTransform != nullptr it must be a row-major 4×4 float[16] and
    // is applied to every pose: T_corrected = M_mrp * T_pose.
    bool loadCSV(const std::string& path, const float* mrpTransform = nullptr);

    // After all CSVs loaded, sort by timestamp.
    void sort();

    // Nearest-neighbour lookup by timestamp (nanoseconds).
    // Returns nullptr if empty.
    const TrajPose* nearest(int64_t ts_ns) const;

    bool empty() const { return poses.empty(); }
};