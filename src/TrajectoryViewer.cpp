#include "raylib.h"
#include "rlgl.h"
#include "raymath.h"
#include "external/glad.h"
#include "rlImGui.h"
#include "imgui.h"
#include "Trajectory.h"
#include "Camera.h"
#include "PointCloud.h"
#include <laszip/laszip_api.h>
#include <nlohmann/json.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <vector>
#include <map>
#include <string>
#include <filesystem>
#include <algorithm>
#include <fstream>
#include <sstream>
#include <cstring>
#include <cstdint>
#include <cmath>

namespace fs = std::filesystem;

// ── GPU point cloud shader ────────────────────────────────────────────────────
// colorPacked: float bits = 0x00RRGGBB; colorMode: 0=jet depth, 1=RGB
static const char* kVS = R"(
#version 330
layout(location = 0) in vec3 pos;
layout(location = 1) in float colorPacked;
uniform mat4 mvp;
uniform float pointSize;
out float fragDist;
out vec4 vertColor;
void main() {
    gl_Position  = mvp * vec4(pos, 1.0);
    gl_PointSize = pointSize;
    fragDist     = length(pos);
    uint p = floatBitsToUint(colorPacked);
    vertColor = vec4(float((p >> 16) & 0xFFu) / 255.0,
                     float((p >>  8) & 0xFFu) / 255.0,
                     float( p        & 0xFFu) / 255.0,
                     1.0);
}
)";
static const char* kFS = R"(
#version 330
in float fragDist;
in vec4 vertColor;
uniform float maxDist;
uniform int colorMode;
out vec4 finalColor;
vec3 jet(float t) {
    t = clamp(t, 0.0, 1.0);
    return clamp(vec3(1.5 - abs(4.0*t - 3.0),
                      1.5 - abs(4.0*t - 2.0),
                      1.5 - abs(4.0*t - 1.0)), 0.0, 1.0);
}
void main() {
    if (colorMode == 1) finalColor = vertColor;
    else finalColor = vec4(jet(fragDist / max(maxDist, 1.0)), 1.0);
}
)";

struct GpuCloud {
    unsigned int vao = 0, vbo = 0;
    int count = 0;
    float maxDist = 50.f;

    void upload(const std::vector<float>& data, float mx) {
        unload();
        if (data.empty()) return;
        maxDist = mx;
        vao = rlLoadVertexArray();
        rlEnableVertexArray(vao);
        vbo = rlLoadVertexBuffer(data.data(), (int)(data.size()*sizeof(float)), false);
        const int stride = 4 * sizeof(float);
        rlSetVertexAttribute(0, 3, RL_FLOAT, false, stride, 0);
        rlEnableVertexAttribute(0);
        rlSetVertexAttribute(1, 1, RL_FLOAT, false, stride, 3*sizeof(float));
        rlEnableVertexAttribute(1);
        rlDisableVertexArray();
        count = (int)(data.size() / 4);
    }
    void unload() {
        if (vao) { rlUnloadVertexArray(vao); vao = 0; }
        if (vbo) { rlUnloadVertexBuffer(vbo); vbo = 0; }
        count = 0;
    }
};

// ── Orbit camera (same as CalibrationApp) ─────────────────────────────────────
struct Orbit {
    float az = 30.f, el = 25.f, dist = 30.f;
    Vector3 target = {};
    Camera3D toRaylib() const {
        float a = az*(float)DEG2RAD, e = el*(float)DEG2RAD;
        Camera3D c;
        c.position   = {target.x + dist*std::cos(e)*std::sin(a),
                         target.y + dist*std::sin(e),
                         target.z + dist*std::cos(e)*std::cos(a)};
        c.target     = target;
        c.up         = {0,1,0};
        c.fovy       = 45.f;
        c.projection = CAMERA_PERSPECTIVE;
        return c;
    }
    void update(bool active) {
        if (!active) return;
        if (IsMouseButtonDown(MOUSE_BUTTON_LEFT)) {
            Vector2 d = GetMouseDelta();
            az -= d.x*0.4f; el += d.y*0.4f;
            el = std::max(-89.f, std::min(89.f, el));
        }
        if (IsMouseButtonDown(MOUSE_BUTTON_RIGHT)) {
            Camera3D cam = toRaylib();
            Vector3 fwd = Vector3Normalize(Vector3Subtract(cam.target, cam.position));
            Vector3 right = Vector3Normalize(Vector3CrossProduct(fwd, cam.up));
            Vector3 up = Vector3CrossProduct(right, fwd);
            Vector2 d = GetMouseDelta(); float sp = dist*0.002f;
            target = Vector3Add(target, Vector3Scale(right, -d.x*sp));
            target = Vector3Add(target, Vector3Scale(up,     d.y*sp));
        }
        float w = GetMouseWheelMove();
        if (w != 0.f) dist = std::max(0.5f, dist - w*dist*0.1f);
    }
};

struct ColorPt { float x, y, z; uint8_t r, g, b; };

// ── Application state ─────────────────────────────────────────────────────────
struct State {
    Trajectory           traj;
    std::vector<int64_t> imageTsNs;
    Intrinsics           K;
    Extrinsics           E;
    bool                 calibLoaded = false;
    int                  imgW = 4656, imgH = 3496;

    // loaded camera images: timestamp → resized BGR Mat
    std::map<int64_t, cv::Mat> images;
    float imgScale = 0.25f;  // resize factor when loading

    GpuCloud             cloud;
    Shader               shader = {};
    bool                 shaderOk = false;
    int                  locMVP = -1, locPS = -1, locMD = -1, locCM = -1;

    Orbit                orbit;

    // controls
    bool  showPath      = true;
    bool  showFrustums  = true;
    float frustumScale  = 0.5f;
    float pointSize     = 2.f;
    int   cloudDecim    = 1;
    bool  useImageColor = false;  // set when images+calib available after load

    char  sessionBuf[512] = {};
    char  calibBuf[512]   = {};
    char  exportBuf[512]  = "colored.laz";
    std::vector<ColorPt> exportCloud;
    std::string status;
};

// ── helpers ───────────────────────────────────────────────────────────────────
static Vector3 toRL(float x, float y, float z) { return {x, z, -y}; }

static Vector3 applyPose(const float m[12], float px, float py, float pz) {
    return toRL(m[0]*px + m[1]*py + m[2]*pz + m[3],
                m[4]*px + m[5]*py + m[6]*pz + m[7],
                m[8]*px + m[9]*py + m[10]*pz + m[11]);
}

// rotate-only (no translation)
static void applyRot3(const float m[12], float px, float py, float pz,
                      float& ox, float& oy, float& oz) {
    ox = m[0]*px + m[1]*py + m[2]*pz;
    oy = m[4]*px + m[5]*py + m[6]*pz;
    oz = m[8]*px + m[9]*py + m[10]*pz;
}

// Load all cam0_*.jpg from CAMERA_0 (sibling of session dir) into s.images, resized by s.imgScale.
static void loadImages(State& s) {
    s.images.clear();
    fs::path d(s.sessionBuf);
    fs::path camDir = d.parent_path() / "CAMERA_0";
    if (!fs::is_directory(camDir)) { s.status = "No CAMERA_0 dir found"; return; }

    int loaded = 0;
    for (auto& e : fs::directory_iterator(camDir)) {
        std::string n = e.path().filename().string();
        if (n.rfind("cam0_", 0) != 0 || e.path().extension() != ".jpg") continue;
        try {
            // filename: cam0_<timestamp_ns>.jpg  → strip prefix (5) and ext (4)
            int64_t ts = std::stoll(n.substr(5, n.size() - 9));
            cv::Mat img = cv::imread(e.path().string(), cv::IMREAD_COLOR);
            if (img.empty()) continue;
            if (s.imgScale != 1.0f) {
                cv::Mat small;
                cv::resize(img, small, cv::Size(), s.imgScale, s.imgScale, cv::INTER_AREA);
                s.images[ts] = std::move(small);
            } else {
                s.images[ts] = std::move(img);
            }
            ++loaded;
        } catch (...) {}
    }
    s.status = "Images loaded: " + std::to_string(loaded)
             + " (scale " + std::to_string(s.imgScale) + ")";
}

// Parse session_poses.mrp → map from chunk stem (e.g. "scan_lio_0") to 4×4 row-major matrix.
static std::map<std::string, std::array<float,16>> parseMRP(const fs::path& mrpPath) {
    std::map<std::string, std::array<float,16>> result;
    std::ifstream f(mrpPath);
    if (!f) return result;
    int n; f >> n;
    for (int i = 0; i < n; i++) {
        std::string name; f >> name;
        // strip extension → stem key
        auto dot = name.rfind('.');
        std::string key = (dot != std::string::npos) ? name.substr(0, dot) : name;
        std::array<float,16> M{};
        for (int r = 0; r < 16; r++) f >> M[r];
        if (f) result[key] = M;
    }
    return result;
}

static void loadSession(State& s) {
    s.traj.poses.clear();
    s.imageTsNs.clear();
    s.exportCloud.clear();

    fs::path d(s.sessionBuf);
    if (!fs::is_directory(d)) { s.status = "Not a directory"; return; }

    // parse MRP if present (session_poses.mrp or session_ini_poses.mri)
    auto mrp = parseMRP(d / "session_poses.mrp");
    if (mrp.empty()) mrp = parseMRP(d / "session_ini_poses.mri");

    // trajectory CSVs — apply corresponding MRP transform per chunk
    int csvCount = 0;
    std::vector<fs::path> csvPaths;
    for (auto& e : fs::directory_iterator(d)) {
        std::string n = e.path().filename().string();
        if (n.rfind("trajectory_lio_", 0) == 0 && e.path().extension() == ".csv")
            csvPaths.push_back(e.path());
    }
    std::sort(csvPaths.begin(), csvPaths.end());
    for (auto& cp : csvPaths) {
        // trajectory_lio_N.csv  ↔  scan_lio_N.laz  (same index N)
        std::string stem = cp.stem().string(); // "trajectory_lio_N"
        std::string idx  = stem.substr(stem.rfind('_') + 1); // "N"
        std::string key  = "scan_lio_" + idx;
        const float* M   = mrp.count(key) ? mrp.at(key).data() : nullptr;
        if (s.traj.loadCSV(cp.string(), M)) csvCount++;
    }
    s.traj.sort();

    // camera image timestamps  (CAMERA_0 is sibling of session dir)
    fs::path camDir = d.parent_path() / "CAMERA_0";
    if (fs::is_directory(camDir)) {
        for (auto& e : fs::directory_iterator(camDir)) {
            std::string n = e.path().filename().string();
            if (n.rfind("cam0_", 0) == 0 && e.path().extension() == ".jpg") {
                try {
                    int64_t ts = std::stoll(n.substr(5, n.size() - 9));
                    s.imageTsNs.push_back(ts);
                } catch (...) {}
            }
        }
        std::sort(s.imageTsNs.begin(), s.imageTsNs.end());
    }

    // point cloud: load scan_lio_N.laz, apply MRP, decimate
    std::vector<fs::path> lazPaths;
    for (auto& e : fs::directory_iterator(d)) {
        std::string n = e.path().filename().string();
        if (n.rfind("scan_lio_", 0) == 0 && e.path().extension() == ".laz")
            lazPaths.push_back(e.path());
    }
    std::sort(lazPaths.begin(), lazPaths.end());

    // prepare coloring: need calibration + at least one image loaded
    bool canColor = s.calibLoaded && !s.images.empty();
    Mat3 R_wc = canColor ? eulerZYXtoMat3(s.E.rx, s.E.ry, s.E.rz) : Mat3{};
    float K_fx = s.K.fx * s.imgScale, K_fy = s.K.fy * s.imgScale;
    float K_cx = s.K.cx * s.imgScale, K_cy = s.K.cy * s.imgScale;
    float C_x = s.E.tx, C_y = s.E.ty, C_z = s.E.tz;

    // binary search for nearest image timestamp
    auto nearestImgTs = [&](int64_t ts) -> int64_t {
        auto it = std::lower_bound(s.imageTsNs.begin(), s.imageTsNs.end(), ts);
        if (it == s.imageTsNs.end()) return s.imageTsNs.back();
        if (it == s.imageTsNs.begin()) return *it;
        auto prev = std::prev(it);
        return (std::abs(*it - ts) < std::abs(*prev - ts)) ? *it : *prev;
    };

    // pack gray intensity (0-1) as 0x00GGGGGG (same value in all channels)
    auto packGray = [](float intensity) -> float {
        uint8_t g = (uint8_t)(std::min(1.f, std::max(0.f, intensity)) * 255.f);
        uint32_t p = (uint32_t(g) << 16) | (uint32_t(g) << 8) | uint32_t(g);
        float f; std::memcpy(&f, &p, 4); return f;
    };

    std::vector<float> gpuData;
    float mx = 0.f;
    float sumX = 0, sumY = 0, sumZ = 0; int cnt = 0;
    int step = std::max(1, s.cloudDecim);
    int coloredChunks = 0;

    for (auto& lp : lazPaths) {
        std::string key = lp.stem().string(); // "scan_lio_N"
        const float* M  = mrp.count(key) ? mrp.at(key).data() : nullptr;
        std::string idx = key.substr(key.rfind('_') + 1); // "N"

        // find the camera image nearest to the middle of this chunk's trajectory
        const TrajPose* imgPose = nullptr;
        const cv::Mat*  chunkImg = nullptr;
        if (canColor && !s.imageTsNs.empty()) {
            // quick read of CSV to get first+last timestamp
            fs::path csvPath = d / ("trajectory_lio_" + idx + ".csv");
            std::ifstream cf(csvPath);
            int64_t first = 0, last = 0;
            if (cf) {
                std::string line; std::getline(cf, line); // header
                while (std::getline(cf, line)) {
                    if (line.empty()) continue;
                    std::istringstream ss(line); int64_t ts; ss >> ts;
                    if (!ss) continue;
                    if (!first) first = ts; last = ts;
                }
            }
            if (first && last) {
                int64_t midTs  = (first + last) / 2;
                int64_t imgTs  = nearestImgTs(midTs);
                auto    imgIt  = s.images.find(imgTs);
                imgPose = s.traj.nearest(imgTs);
                if (imgIt != s.images.end() && imgPose)
                    chunkImg = &imgIt->second;
            }
        }
        if (chunkImg) ++coloredChunks;

        PointCloud pc;
        if (!pc.load(lp.string())) continue;

        for (int i = 0; i < (int)pc.points.size(); i += step) {
            auto& pt = pc.points[i];
            float x = pt.x, y = pt.y, z = pt.z;
            if (M) {
                float nx = M[0]*x + M[1]*y + M[2]*z  + M[3];
                float ny = M[4]*x + M[5]*y + M[6]*z  + M[7];
                float nz = M[8]*x + M[9]*y + M[10]*z + M[11];
                x = nx; y = ny; z = nz;
            }
            // world coords → raylib
            gpuData.push_back(x);
            gpuData.push_back(z);
            gpuData.push_back(-y);

            // color: try to project world point into chunk's camera image
            float colorF = packGray(pt.intensity);
            if (chunkImg && imgPose) {
                const float* m = imgPose->m;
                // world → lidar body: p_lidar = R_wl^T * (p_world - t)
                float dx = x - m[3], dy_= y - m[7], dz = z - m[11];
                float lx = m[0]*dx + m[4]*dy_ + m[8]*dz;
                float ly = m[1]*dx + m[5]*dy_ + m[9]*dz;
                float lz = m[2]*dx + m[6]*dy_ + m[10]*dz;
                // lidar body → camera: R_wc^T * (p_lidar - C)
                float ex = lx - C_x, ey = ly - C_y, ez = lz - C_z;
                float cx_ = R_wc.m[0]*ex + R_wc.m[3]*ey + R_wc.m[6]*ez;
                float cy_ = R_wc.m[1]*ex + R_wc.m[4]*ey + R_wc.m[7]*ez;
                float cz_ = R_wc.m[2]*ex + R_wc.m[5]*ey + R_wc.m[8]*ez;
                if (cz_ > 0.05f) {
                    int iu = (int)std::round(K_fx * (cx_/cz_) + K_cx);
                    int iv = (int)std::round(K_fy * (cy_/cz_) + K_cy);
                    if (iu >= 0 && iu < chunkImg->cols && iv >= 0 && iv < chunkImg->rows) {
                        cv::Vec3b bgr = chunkImg->at<cv::Vec3b>(iv, iu);
                        uint32_t p = (uint32_t(bgr[2]) << 16) |
                                     (uint32_t(bgr[1]) <<  8) |
                                      uint32_t(bgr[0]);
                        std::memcpy(&colorF, &p, 4);
                    }
                }
            }
            gpuData.push_back(colorF);

            uint32_t packed; std::memcpy(&packed, &colorF, 4);
            s.exportCloud.push_back({x, y, z,
                (uint8_t)((packed >> 16) & 0xFF),
                (uint8_t)((packed >>  8) & 0xFF),
                (uint8_t)( packed        & 0xFF)});

            float d2 = x*x + y*y + z*z;
            if (d2 > mx*mx) mx = std::sqrt(d2);
            sumX += x; sumY += z; sumZ += -y; cnt++;
        }
    }
    s.useImageColor = canColor && (coloredChunks > 0);

    if (cnt > 0) {
        s.cloud.upload(gpuData, mx);
        s.orbit.target = {sumX/cnt, sumY/cnt, sumZ/cnt};
        s.orbit.dist   = std::max(5.f, mx * 0.3f);
    }

    s.status = "Poses: "  + std::to_string(s.traj.poses.size())
             + "  Img: "  + std::to_string(s.imageTsNs.size())
             + "  Pts: "  + std::to_string(s.cloud.count)
             + (mrp.empty() ? "  (no MRP)" : "  +MRP")
             + (s.useImageColor ? "  +RGB" : "");
}

static void loadCalib(State& s) {
    std::ifstream f(s.calibBuf);
    if (!f) { s.status = std::string("Cannot open: ") + s.calibBuf; return; }
    nlohmann::json j; f >> j;
    if (j.contains("intrinsics")) {
        auto& ji = j["intrinsics"];
        s.K.fx = ji.value("fx", s.K.fx); s.K.fy = ji.value("fy", s.K.fy);
        s.K.cx = ji.value("cx", s.K.cx); s.K.cy = ji.value("cy", s.K.cy);
    }
    if (j.contains("extrinsics")) {
        auto& je = j["extrinsics"];
        if (je.contains("camera_position_in_world_xyz") &&
            je["camera_position_in_world_xyz"].size() >= 3) {
            s.E.tx = je["camera_position_in_world_xyz"][0];
            s.E.ty = je["camera_position_in_world_xyz"][1];
            s.E.tz = je["camera_position_in_world_xyz"][2];
        }
        if (je.contains("camera_rotation_in_world_euler_zyx_deg") &&
            je["camera_rotation_in_world_euler_zyx_deg"].size() >= 3) {
            s.E.rz = je["camera_rotation_in_world_euler_zyx_deg"][0];
            s.E.ry = je["camera_rotation_in_world_euler_zyx_deg"][1];
            s.E.rx = je["camera_rotation_in_world_euler_zyx_deg"][2];
        }
    }
    s.calibLoaded = true;
    s.status = "Calibration loaded";
}

static void exportLAZ(State& s) {
    if (s.exportCloud.empty()) { s.status = "No cloud to export"; return; }

    double xmin = s.exportCloud[0].x, xmax = xmin;
    double ymin = s.exportCloud[0].y, ymax = ymin;
    double zmin = s.exportCloud[0].z, zmax = zmin;
    for (auto& p : s.exportCloud) {
        xmin = std::min(xmin,(double)p.x); xmax = std::max(xmax,(double)p.x);
        ymin = std::min(ymin,(double)p.y); ymax = std::max(ymax,(double)p.y);
        zmin = std::min(zmin,(double)p.z); zmax = std::max(zmax,(double)p.z);
    }

    laszip_POINTER writer = nullptr;
    if (laszip_create(&writer)) { s.status = "laszip_create failed"; return; }

    laszip_header* header = nullptr;
    laszip_get_header_pointer(writer, &header);

    header->version_major            = 1;
    header->version_minor            = 2;
    header->header_size              = 227;
    header->offset_to_point_data     = 227;
    header->point_data_format        = 2;   // XYZ + RGB
    header->point_data_record_length = 26;
    header->number_of_point_records  = (uint32_t)s.exportCloud.size();
    header->x_scale_factor = 0.001; header->y_scale_factor = 0.001; header->z_scale_factor = 0.001;
    header->x_offset = xmin; header->y_offset = ymin; header->z_offset = zmin;
    header->min_x = xmin;   header->max_x = xmax;
    header->min_y = ymin;   header->max_y = ymax;
    header->min_z = zmin;   header->max_z = zmax;

    laszip_BOOL compress = (std::strstr(s.exportBuf, ".laz") != nullptr) ? 1 : 0;
    if (laszip_open_writer(writer, s.exportBuf, compress)) {
        laszip_CHAR* err = nullptr; laszip_get_error(writer, &err);
        s.status = std::string("Export failed: ") + (err ? err : "?");
        laszip_destroy(writer); return;
    }

    laszip_point* point = nullptr;
    laszip_get_point_pointer(writer, &point);

    laszip_F64 coords[3];
    for (auto& p : s.exportCloud) {
        coords[0] = p.x; coords[1] = p.y; coords[2] = p.z;
        laszip_set_coordinates(writer, coords);
        point->rgb[0] = (laszip_U16)p.r << 8;
        point->rgb[1] = (laszip_U16)p.g << 8;
        point->rgb[2] = (laszip_U16)p.b << 8;
        laszip_write_point(writer);
    }

    laszip_close_writer(writer);
    laszip_destroy(writer);
    s.status = "Exported " + std::to_string(s.exportCloud.size()) + " pts → " + s.exportBuf;
}

static void drawScene(State& s) {
    auto toRL = [](float x, float y, float z) -> Vector3 { return {x, z, -y}; };

    // ── trajectory path ───────────────────────────────────────────────────────
    if (s.showPath) {
        for (size_t i = 1; i < s.traj.poses.size(); i++) {
            auto& a = s.traj.poses[i-1]; auto& b = s.traj.poses[i];
            DrawLine3D(toRL(a.m[3], a.m[7], a.m[11]),
                       toRL(b.m[3], b.m[7], b.m[11]),
                       Color{100, 200, 255, 220});
        }
    }

    // ── camera frustums ───────────────────────────────────────────────────────
    if (s.showFrustums && s.calibLoaded) {
        Mat3 R_wc = eulerZYXtoMat3(s.E.rx, s.E.ry, s.E.rz);
        Vec3 C    = {s.E.tx, s.E.ty, s.E.tz};
        float fs  = s.frustumScale;
        float ncx[4] = {(0.f        - s.K.cx)/s.K.fx, (float(s.imgW)-s.K.cx)/s.K.fx,
                        (float(s.imgW)-s.K.cx)/s.K.fx, (0.f        - s.K.cx)/s.K.fx};
        float ncy[4] = {(0.f        - s.K.cy)/s.K.fy, (0.f        - s.K.cy)/s.K.fy,
                        (float(s.imgH)-s.K.cy)/s.K.fy, (float(s.imgH)-s.K.cy)/s.K.fy};

        for (int64_t ts : s.imageTsNs) {
            const TrajPose* pose = s.traj.nearest(ts);
            if (!pose) continue;

            // camera origin in world
            Vec3 camW;
            { float ox, oy, oz;
              applyRot3(pose->m, C.x, C.y, C.z, ox, oy, oz);
              camW = {ox + pose->m[3], oy + pose->m[7], oz + pose->m[11]}; }
            Vector3 origin = toRL(camW.x, camW.y, camW.z);

            Vector3 w[4];
            for (int k = 0; k < 4; k++) {
                // p_cam → p_lidar (body) via R_wc
                Vec3 pc = {ncx[k]*fs, ncy[k]*fs, fs};
                Vec3 pl = R_wc.mul(pc);
                pl.x += C.x; pl.y += C.y; pl.z += C.z;
                // p_lidar → world via trajectory pose
                float ox, oy, oz;
                applyRot3(pose->m, pl.x, pl.y, pl.z, ox, oy, oz);
                w[k] = toRL(ox + pose->m[3], oy + pose->m[7], oz + pose->m[11]);
            }
            Color fc = ORANGE;
            DrawLine3D(origin,w[0],fc); DrawLine3D(origin,w[1],fc);
            DrawLine3D(origin,w[2],fc); DrawLine3D(origin,w[3],fc);
            DrawLine3D(w[0],w[1],fc);   DrawLine3D(w[1],w[2],fc);
            DrawLine3D(w[2],w[3],fc);   DrawLine3D(w[3],w[0],fc);
        }
    }

    // ── GPU point cloud ───────────────────────────────────────────────────────
    if (s.cloud.count > 0 && s.shaderOk) {
        rlDrawRenderBatchActive();
        Matrix mvp = MatrixMultiply(rlGetMatrixModelview(), rlGetMatrixProjection());
        rlEnableShader(s.shader.id);
        rlSetUniformMatrix(s.locMVP, mvp);
        rlSetUniform(s.locPS, &s.pointSize,     RL_SHADER_UNIFORM_FLOAT, 1);
        rlSetUniform(s.locMD, &s.cloud.maxDist, RL_SHADER_UNIFORM_FLOAT, 1);
        int cm = s.useImageColor ? 1 : 0;
        rlSetUniform(s.locCM, &cm, RL_SHADER_UNIFORM_INT, 1);
        rlEnableVertexArray(s.cloud.vao);
        glDrawArrays(GL_POINTS, 0, s.cloud.count);
        rlDisableVertexArray();
        rlDisableShader();
    }
}

// ── main ──────────────────────────────────────────────────────────────────────
int main(int argc, char* argv[]) {
    State s;
    if (argc > 1) strncpy(s.sessionBuf, argv[1], sizeof(s.sessionBuf)-1);
    if (argc > 2) strncpy(s.calibBuf,   argv[2], sizeof(s.calibBuf)-1);

    SetConfigFlags(FLAG_WINDOW_RESIZABLE | FLAG_MSAA_4X_HINT);
    InitWindow(1400, 900, "Trajectory Viewer");
    SetTargetFPS(60);
    rlImGuiSetup(true);

    s.shader   = LoadShaderFromMemory(kVS, kFS);
    s.shaderOk = s.shader.id > 0;
    if (s.shaderOk) {
        s.locMVP = rlGetLocationUniform(s.shader.id, "mvp");
        s.locPS  = rlGetLocationUniform(s.shader.id, "pointSize");
        s.locMD  = rlGetLocationUniform(s.shader.id, "maxDist");
        s.locCM  = rlGetLocationUniform(s.shader.id, "colorMode");
    }
    glEnable(GL_PROGRAM_POINT_SIZE);

    // auto-load if args given
    if (s.sessionBuf[0]) loadSession(s);
    if (s.calibBuf[0])   loadCalib(s);

    const float PANEL_W = 420.f;

    while (!WindowShouldClose()) {
        bool imguiWants = ImGui::GetIO().WantCaptureMouse;
        s.orbit.update(!imguiWants);

        BeginDrawing();
        ClearBackground(Color{25, 25, 25, 255});

        Camera3D cam = s.orbit.toRaylib();
        BeginMode3D(cam);
        drawScene(s);
        DrawGrid(20, 1.f);
        // axes
        DrawLine3D({0,0,0},{2,0,0},RED);
        DrawLine3D({0,0,0},{0,2,0},GREEN);
        DrawLine3D({0,0,0},{0,0,-2},BLUE);
        EndMode3D();

        // ── ImGui panel ───────────────────────────────────────────────────────
        rlImGuiBegin();
        ImGuiIO& io = ImGui::GetIO();
        ImGui::SetNextWindowPos(ImVec2(io.DisplaySize.x - PANEL_W, 0), ImGuiCond_Always);
        ImGui::SetNextWindowSize(ImVec2(PANEL_W, io.DisplaySize.y), ImGuiCond_Always);
        ImGui::Begin("##panel", nullptr,
            ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize |
            ImGuiWindowFlags_NoMove     | ImGuiWindowFlags_NoCollapse);

        ImGui::TextColored(ImVec4(0.4f,0.8f,1.f,1.f), "Trajectory Viewer");
        ImGui::Separator();

        if (ImGui::CollapsingHeader("Session", ImGuiTreeNodeFlags_DefaultOpen)) {
            ImGui::PushItemWidth(-1);
            ImGui::Text("LIO result directory:");
            ImGui::InputText("##sess", s.sessionBuf, sizeof(s.sessionBuf));
            ImGui::InputInt("Point decimation", &s.cloudDecim);
            s.cloudDecim = std::max(1, s.cloudDecim);
            float hw = (ImGui::GetContentRegionAvail().x - ImGui::GetStyle().ItemSpacing.x) * 0.5f;
            if (ImGui::Button("Load session", ImVec2(hw, 0))) loadSession(s);
            ImGui::SameLine();
            if (ImGui::Button("Load images", ImVec2(hw, 0))) {
                loadImages(s);
                // re-colorize if session already loaded
                if (!s.traj.poses.empty()) loadSession(s);
            }
            ImGui::SliderFloat("Img scale", &s.imgScale, 0.1f, 1.0f, "%.2f");
            if (!s.images.empty())
                ImGui::TextDisabled("%d images in RAM", (int)s.images.size());
            ImGui::PopItemWidth();
        }

        if (ImGui::CollapsingHeader("Calibration", ImGuiTreeNodeFlags_DefaultOpen)) {
            ImGui::PushItemWidth(-1);
            ImGui::Text("Calibration JSON:");
            ImGui::InputText("##cal", s.calibBuf, sizeof(s.calibBuf));
            if (ImGui::Button("Load calibration", ImVec2(-1,0))) loadCalib(s);
            if (s.calibLoaded) {
                ImGui::Text("fx=%.0f fy=%.0f", s.K.fx, s.K.fy);
                ImGui::Text("cx=%.0f cy=%.0f", s.K.cx, s.K.cy);
                ImGui::InputInt("Image W", &s.imgW);
                ImGui::InputInt("Image H", &s.imgH);
            }
            ImGui::PopItemWidth();
        }

        if (ImGui::CollapsingHeader("Visualization", ImGuiTreeNodeFlags_DefaultOpen)) {
            ImGui::Checkbox("Show path",     &s.showPath);
            ImGui::Checkbox("Show frustums", &s.showFrustums);
            ImGui::SliderFloat("Frustum scale", &s.frustumScale, 0.05f, 5.f, "%.2f");
            ImGui::SliderFloat("Point size",    &s.pointSize,    1.f,  20.f, "%.1f");
            if (!s.images.empty()) {
                ImGui::Separator();
                ImGui::Checkbox("Color by image (RGB)", &s.useImageColor);
            }
        }

        if (ImGui::CollapsingHeader("Export", ImGuiTreeNodeFlags_DefaultOpen)) {
            ImGui::PushItemWidth(-1);
            ImGui::Text("Output file (.laz / .las):");
            ImGui::InputText("##out", s.exportBuf, sizeof(s.exportBuf));
            if (ImGui::Button("Export colored LAZ", ImVec2(-1, 0))) exportLAZ(s);
            if (!s.exportCloud.empty())
                ImGui::TextDisabled("%d pts ready to export", (int)s.exportCloud.size());
            ImGui::PopItemWidth();
        }

        if (!s.status.empty()) {
            ImGui::Separator();
            ImGui::TextColored(ImVec4(1,1,0,1), "%s", s.status.c_str());
        }

        ImGui::Separator();
        ImGui::TextDisabled("LMB: orbit  RMB: pan  Scroll: zoom");

        ImGui::End();
        rlImGuiEnd();
        EndDrawing();
    }

    s.cloud.unload();
    if (s.shaderOk) UnloadShader(s.shader);
    rlImGuiShutdown();
    CloseWindow();
    return 0;
}