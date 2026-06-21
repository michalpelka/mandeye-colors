#pragma once
#include <vector>
#include <string>

struct Point3D {
    float x, y, z;
    float intensity; // normalized to [0,1]
};

struct PointCloud {
    std::vector<Point3D> points;
    float minX = 0.f, maxX = 0.f;
    float minY = 0.f, maxY = 0.f;
    float minZ = 0.f, maxZ = 0.f;

    bool load(const std::string& path);
    void clear();
    bool empty() const { return points.empty(); }
};
