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

# ── Eigen ────────────────────────────────────────────────────────────────────
find_package(Eigen3 3.4 REQUIRED NO_MODULE)

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
    Eigen3::Eigen
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
    src/RosExport.cpp
)
target_include_directories(TrajectoryViewer PRIVATE src ${LASZIP_INCLUDE})
target_link_libraries(TrajectoryViewer PRIVATE
    raylib imgui rlimgui nlohmann_json::nlohmann_json Eigen3::Eigen laszip ${OpenCV_LIBS})
target_compile_options(TrajectoryViewer PRIVATE -Wall -Wextra)

# ── Optional ROS 2 bag export ─────────────────────────────────────────────────
# OFF by default so the project still builds on machines without ROS. Enable with
#   cmake -DCALIB_ENABLE_ROS_EXPORT=ON   (source a ROS 2 install first).
option(CALIB_ENABLE_ROS_EXPORT "Enable ROS 2 bag export in TrajectoryViewer (needs ROS 2)" OFF)
if(CALIB_ENABLE_ROS_EXPORT)
    find_package(rclcpp        REQUIRED)
    find_package(rosbag2_cpp   REQUIRED)
    find_package(rosbag2_storage REQUIRED)
    find_package(builtin_interfaces REQUIRED)
    find_package(std_msgs      REQUIRED)
    find_package(geometry_msgs REQUIRED)
    find_package(sensor_msgs   REQUIRED)
    find_package(tf2_msgs      REQUIRED)
    # Link the exported targets with the keyword (PRIVATE) form so it matches the
    # rest of this target's link calls (ament_target_dependencies uses the plain
    # form, which CMake forbids mixing).
    target_link_libraries(TrajectoryViewer PRIVATE
        rclcpp::rclcpp
        rosbag2_cpp::rosbag2_cpp
        rosbag2_storage::rosbag2_storage
        ${builtin_interfaces_TARGETS}
        ${std_msgs_TARGETS}
        ${geometry_msgs_TARGETS}
        ${sensor_msgs_TARGETS}
        ${tf2_msgs_TARGETS})
    target_compile_definitions(TrajectoryViewer PRIVATE CALIB_ENABLE_ROS_EXPORT)
    message(STATUS "ROS 2 bag export: ENABLED")
else()
    message(STATUS "ROS 2 bag export: disabled (set -DCALIB_ENABLE_ROS_EXPORT=ON to enable)")
endif()

# ── IntrinsicsCalib ───────────────────────────────────────────────────────────
add_executable(IntrinsicsCalib src/IntrinsicsCalib.cpp)
target_link_libraries(IntrinsicsCalib PRIVATE
    raylib imgui rlimgui nlohmann_json::nlohmann_json Eigen3::Eigen ${OpenCV_LIBS})
target_compile_options(IntrinsicsCalib PRIVATE -Wall -Wextra)
