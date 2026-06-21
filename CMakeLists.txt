cmake_minimum_required(VERSION 3.20)
project(CalibrationApp CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(FetchContent)

# ── raylib ──────────────────────────────────────────────────────────────────
set(BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(BUILD_GAMES    OFF CACHE BOOL "" FORCE)
FetchContent_Declare(
    raylib
    GIT_REPOSITORY https://github.com/raysan5/raylib.git
    GIT_TAG        5.5
    GIT_SHALLOW    TRUE
)
FetchContent_MakeAvailable(raylib)

# ── Dear ImGui ───────────────────────────────────────────────────────────────
FetchContent_Declare(
    imgui
    GIT_REPOSITORY https://github.com/ocornut/imgui.git
    GIT_TAG        v1.92.8
    GIT_SHALLOW    TRUE
)
FetchContent_GetProperties(imgui)
if(NOT imgui_POPULATED)
    FetchContent_Populate(imgui)
    add_library(imgui STATIC
        ${imgui_SOURCE_DIR}/imgui.cpp
        ${imgui_SOURCE_DIR}/imgui_draw.cpp
        ${imgui_SOURCE_DIR}/imgui_tables.cpp
        ${imgui_SOURCE_DIR}/imgui_widgets.cpp
        ${imgui_SOURCE_DIR}/imgui_demo.cpp
    )
    target_include_directories(imgui PUBLIC ${imgui_SOURCE_DIR})
    target_compile_definitions(imgui PUBLIC IMGUI_DISABLE_OBSOLETE_FUNCTIONS)
endif()

# ── rlImGui ──────────────────────────────────────────────────────────────────
FetchContent_Declare(
    rlimgui
    GIT_REPOSITORY https://github.com/raylib-extras/rlImGui.git
    GIT_TAG        main
    GIT_SHALLOW    TRUE
)
FetchContent_GetProperties(rlimgui)
if(NOT rlimgui_POPULATED)
    FetchContent_Populate(rlimgui)
    add_library(rlimgui STATIC ${rlimgui_SOURCE_DIR}/rlImGui.cpp)
    target_include_directories(rlimgui PUBLIC ${rlimgui_SOURCE_DIR})
    target_link_libraries(rlimgui PUBLIC raylib imgui)
endif()

# ── nlohmann/json ────────────────────────────────────────────────────────────
set(JSON_BuildTests OFF CACHE BOOL "" FORCE)
set(JSON_Install    OFF CACHE BOOL "" FORCE)
FetchContent_Declare(
    nlohmann_json
    GIT_REPOSITORY https://github.com/nlohmann/json.git
    GIT_TAG        v3.11.3
    GIT_SHALLOW    TRUE
)
FetchContent_MakeAvailable(nlohmann_json)

# ── liblaszip (system) ───────────────────────────────────────────────────────
message("LASzip library")
add_subdirectory(3rdparty/LASzip)
include_directories(3rdparty/LASzip/include)

# ── OpenCV (system) ──────────────────────────────────────────────────────────
find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs calib3d)


# ── executable ───────────────────────────────────────────────────────────────
add_executable(CalibrationApp
    src/main.cpp
    src/App.cpp
    src/PointCloud.cpp
    src/Camera.cpp
    src/UI.cpp
    src/Renderer.cpp
    src/Trajectory.cpp
)

target_include_directories(CalibrationApp PRIVATE
    src
    ${LASZIP_INCLUDE}
)

target_link_libraries(CalibrationApp PRIVATE
    raylib
    imgui
    rlimgui
    nlohmann_json::nlohmann_json
    laszip
    ${OpenCV_LIBS}
)

target_compile_options(CalibrationApp PRIVATE -Wall -Wextra)

# ── TrajectoryViewer ─────────────────────────────────────────────────────────
add_executable(TrajectoryViewer
    src/TrajectoryViewer.cpp
    src/Trajectory.cpp
    src/Camera.cpp
    src/PointCloud.cpp
)
target_include_directories(TrajectoryViewer PRIVATE src ${LASZIP_INCLUDE})
target_link_libraries(TrajectoryViewer PRIVATE
    raylib imgui rlimgui nlohmann_json::nlohmann_json laszip ${OpenCV_LIBS})
target_compile_options(TrajectoryViewer PRIVATE -Wall -Wextra)

# ── IntrinsicsCalib ───────────────────────────────────────────────────────────
add_executable(IntrinsicsCalib src/IntrinsicsCalib.cpp)
target_link_libraries(IntrinsicsCalib PRIVATE
    raylib imgui rlimgui nlohmann_json::nlohmann_json ${OpenCV_LIBS})
target_compile_options(IntrinsicsCalib PRIVATE -Wall -Wextra)
