#include "App.h"
#include <string>
#include <algorithm>

static std::string ext(const char* path) {
    std::string s = path;
    auto pos = s.rfind('.');
    if (pos == std::string::npos) return "";
    std::string e = s.substr(pos + 1);
    std::transform(e.begin(), e.end(), e.begin(), ::tolower);
    return e;
}

int main(int argc, char* argv[]) {
    App app;
    for (int i = 1; i < argc; ++i) {
        std::string e = ext(argv[i]);
        if (e == "jpg" || e == "jpeg" || e == "png" || e == "bmp")
            app.preloadImage(argv[i]);
        else if (e == "laz" || e == "las")
            app.preloadCloud(argv[i]);
        else if (e == "yml" || e == "yaml")
            app.preloadIntrinsics(argv[i]);
        else if (e == "json")
            app.preloadCalibration(argv[i]); // handles both full calib and intrinsics-only
    }
    app.run();
    return 0;
}
