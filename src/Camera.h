#pragma once
#include <cmath>

struct Vec3 {
    float x = 0.f, y = 0.f, z = 0.f;
};

// Row-major 3x3: element at row r, col c → m[r*3+c]
struct Mat3 {
    float m[9] = {};
    Vec3 mul(Vec3 v) const {
        return {
            m[0]*v.x + m[1]*v.y + m[2]*v.z,
            m[3]*v.x + m[4]*v.y + m[5]*v.z,
            m[6]*v.x + m[7]*v.y + m[8]*v.z
        };
    }
};

struct Intrinsics {
    float fx = 800.f, fy = 800.f;
    float cx = 640.f, cy = 360.f;
    // OpenCV rational distortion model:
    // radial = (1 + k1 r² + k2 r⁴ + k3 r⁶) / (1 + k4 r² + k5 r⁴ + k6 r⁶)
    float k1 = 0.f, k2 = 0.f, k3 = 0.f;
    float k4 = 0.f, k5 = 0.f, k6 = 0.f;
    // tangential
    float p1 = 0.f, p2 = 0.f;
};

struct Extrinsics {
    // Camera position in LiDAR/world frame
    float tx = 0.f, ty = 0.f, tz = 0.f;
    // Camera orientation in LiDAR/world frame — ZYX Euler, degrees.
    // Default: standard camera (X=right, Y=down, Z=forward) aligned with LiDAR (X=forward).
    float rx = -90.f, ry = 0.f, rz = -90.f;
};

// Rotation matrix from ZYX Euler angles (degrees): R = Rz * Ry * Rx.
// In world-frame convention this gives R_wc (camera orientation in world).
Mat3 eulerZYXtoMat3(float rx_deg, float ry_deg, float rz_deg);

// Project a point from LiDAR frame to image pixel (u, v).
// depth = z component in camera frame (positive = in front).
// Returns false if depth <= 0 (behind camera).
bool projectPoint(float px, float py, float pz,
                  const Intrinsics& K, const Mat3& R, const Vec3& t,
                  float& u, float& v, float& depth);
