#include "Camera.h"

Mat3 eulerZYXtoMat3(float rx_deg, float ry_deg, float rz_deg) {
    const float deg2rad = static_cast<float>(M_PI) / 180.f;
    float cx = std::cos(rx_deg * deg2rad), sx = std::sin(rx_deg * deg2rad);
    float cy = std::cos(ry_deg * deg2rad), sy = std::sin(ry_deg * deg2rad);
    float cz = std::cos(rz_deg * deg2rad), sz = std::sin(rz_deg * deg2rad);

    // R = Rz * Ry * Rx
    Mat3 R;
    R.m[0] =  cy*cz;              R.m[1] = cz*sx*sy - cx*sz;  R.m[2] = cx*cz*sy + sx*sz;
    R.m[3] =  cy*sz;              R.m[4] = cx*cz + sx*sy*sz;  R.m[5] = cx*sy*sz - cz*sx;
    R.m[6] = -sy;                 R.m[7] = cy*sx;              R.m[8] = cx*cy;
    return R;
}

bool projectPoint(float px, float py, float pz,
                  const Intrinsics& K, const Mat3& R, const Vec3& t,
                  float& u, float& v, float& depth) {
    // World-frame convention: R = R_wc (camera orientation in world), t = camera position in world
    // p_cam = R_wc^T * (p_lidar - C)
    float dx = px - t.x, dy = py - t.y, dz = pz - t.z;
    Vec3 pc = {
        R.m[0]*dx + R.m[3]*dy + R.m[6]*dz,
        R.m[1]*dx + R.m[4]*dy + R.m[7]*dz,
        R.m[2]*dx + R.m[5]*dy + R.m[8]*dz
    };

    depth = pc.z;
    if (depth <= 1e-4f)
        return false;

    // Normalized image coordinates
    float xn = pc.x / pc.z;
    float yn = pc.y / pc.z;

    // Rational radial + tangential distortion
    float r2 = xn*xn + yn*yn;
    float r4 = r2 * r2;
    float r6 = r4 * r2;
    float radial = (1.f + K.k1*r2 + K.k2*r4 + K.k3*r6)
                 / (1.f + K.k4*r2 + K.k5*r4 + K.k6*r6);
    float xd = xn*radial + 2.f*K.p1*xn*yn + K.p2*(r2 + 2.f*xn*xn);
    float yd = yn*radial + K.p1*(r2 + 2.f*yn*yn) + 2.f*K.p2*xn*yn;

    u = K.fx * xd + K.cx;
    v = K.fy * yd + K.cy;
    return true;
}
