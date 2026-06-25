cmake_minimum_required(VERSION 3.20)
project(CalibrationApp CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(FetchContent)

get_filename_component(REPOSITORY_DIRECTORY ${CMAKE_CURRENT_LIST_DIR} ABSOLUTE)
set(THIRDPARTY_DIRECTORY_BINARY ${REPOSITORY_DIRECTORY}/3rdpartyBinary)

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

add_subdirectory(3rdparty/eigen)
set(EIGEN3_INCLUDE_DIR ${REPOSITORY_DIRECTORY}/3rdparty/eigen)
MESSAGE(STATUS "Using bundled Eigen3 : ${EIGEN3_INCLUDE_DIR}")

# ── liblaszip (system) ───────────────────────────────────────────────────────
message("LASzip library")
add_subdirectory(3rdparty/LASzip)

# LASzip appends the major version to the lib name only on Windows (see
# 3rdparty/LASzip/CMakeLists.txt), so the target is laszip3 there and laszip elsewhere.
if(WIN32)
    set(LASZIP_LIB laszip3)
else()
    set(LASZIP_LIB laszip)
endif()

# LASzip's bundled cmake pins its library target to C++11, but its mydefs.cpp uses
# std::filesystem (C++17). Override the standard on the target here so we don't have
# to modify the submodule; the target-level flag wins over the bundled -std=c++11.
set_target_properties(${LASZIP_LIB} PROPERTIES CXX_STANDARD 17 CXX_STANDARD_REQUIRED ON)

file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/LASzip/dll/laszip_api.h 
     DESTINATION ${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/LASzip/include/laszip/)
file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/LASzip/include/laszip/laszip_common.h
     ${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/LASzip/dll/laszip_api.c
     DESTINATION ${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/LASzip/include/laszip/)

set(LASZIP_INCLUDE 
    ${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/LASzip/include 
    ${CMAKE_CURRENT_BINARY_DIR}/_deps)
include_directories(${LASZIP_INCLUDE})

# Collect LASzip DLLs on Windows
if(WIN32)
    set(LASZIP_DLLS
        $<TARGET_FILE:${LASZIP_LIB}>
    )
endif()

# ── OpenCV (system) ──────────────────────────────────────────────────────────

# Download or Search for OpenCV

include(${THIRDPARTY_DIRECTORY_BINARY}/OpenCV/CMakeLists.txt)
find_package(OpenCV REQUIRED)
MESSAGE(STATUS "OpenCV include dir: ${OpenCV_INCLUDE_DIRS}, OpenCV librarys: ${OpenCV_LIBS}")

# Copy OpenCV DLLs to output directory on Windows
if(WIN32 AND OpenCV_DIR)
    file(GLOB OPENCV_DLLS "${OpenCV_DIR}/x64/vc16/bin/*.dll")
    if(OPENCV_DLLS)
        message(STATUS "Found OpenCV DLLs: ${OPENCV_DLLS}")
    else()
        message(WARNING "No OpenCV DLLs found in ${OpenCV_DIR}/x64/vc16/bin/")
    endif()
endif()



# ── executable ───────────────────────────────────────────────────────────────
add_executable(CalibrationApp
    src/main.cpp
    src/App.cpp
    src/PointCloud.cpp
    src/Camera.cpp
    src/UI.cpp
    src/Renderer.cpp
    src/Trajectory.cpp
    src/CliArgs.cpp
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
    ${LASZIP_LIB}
    ${OpenCV_LIBS}
)

if(MSVC)
    target_compile_options(CalibrationApp PRIVATE /W4)
    target_compile_definitions(CalibrationApp PRIVATE _USE_MATH_DEFINES LASZIP_API_VERSION)
else()
    target_compile_options(CalibrationApp PRIVATE -Wall -Wextra)
    target_compile_definitions(CalibrationApp PRIVATE LASZIP_API_VERSION)
endif()

# Copy OpenCV DLLs to executable directory on Windows
if(WIN32 AND OPENCV_DLLS)
    add_custom_command(TARGET CalibrationApp POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
        ${OPENCV_DLLS}
        $<TARGET_FILE_DIR:CalibrationApp>
        COMMENT "Copying OpenCV DLLs to CalibrationApp output directory"
    )
endif()

# Copy LASzip DLL to executable directory on Windows
if(WIN32)
    add_custom_command(TARGET CalibrationApp POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
        $<TARGET_FILE:${LASZIP_LIB}>
        $<TARGET_FILE_DIR:CalibrationApp>
        COMMENT "Copying LASzip DLL to CalibrationApp output directory"
    )
endif()

# ── TrajectoryViewer ─────────────────────────────────────────────────────────
add_executable(TrajectoryViewer
    src/TrajectoryViewer.cpp
    src/Trajectory.cpp
    src/Camera.cpp
    src/PointCloud.cpp
    src/RosExport.cpp
    src/CliArgs.cpp
)
target_include_directories(TrajectoryViewer PRIVATE src ${LASZIP_INCLUDE})
target_link_libraries(TrajectoryViewer PRIVATE
    raylib imgui rlimgui nlohmann_json::nlohmann_json Eigen3::Eigen ${LASZIP_LIB} ${OpenCV_LIBS})
if(MSVC)
    target_compile_options(TrajectoryViewer PRIVATE /W4)
    target_compile_definitions(TrajectoryViewer PRIVATE _USE_MATH_DEFINES LASZIP_API_VERSION)
else()
    target_compile_options(TrajectoryViewer PRIVATE -Wall -Wextra)
    target_compile_definitions(TrajectoryViewer PRIVATE LASZIP_API_VERSION)
endif()

# Copy OpenCV DLLs to executable directory on Windows
if(WIN32 AND OPENCV_DLLS)
    add_custom_command(TARGET TrajectoryViewer POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
        ${OPENCV_DLLS}
        $<TARGET_FILE_DIR:TrajectoryViewer>
        COMMENT "Copying OpenCV DLLs to TrajectoryViewer output directory"
    )
endif()

# Copy LASzip DLL to executable directory on Windows
if(WIN32)
    add_custom_command(TARGET TrajectoryViewer POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
        $<TARGET_FILE:${LASZIP_LIB}>
        $<TARGET_FILE_DIR:TrajectoryViewer>
        COMMENT "Copying LASzip DLL to TrajectoryViewer output directory"
    )
endif()

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
add_executable(IntrinsicsCalib src/IntrinsicsCalib.cpp src/CliArgs.cpp)
target_link_libraries(IntrinsicsCalib PRIVATE
    raylib imgui rlimgui nlohmann_json::nlohmann_json Eigen3::Eigen ${OpenCV_LIBS})
if(MSVC)
    target_compile_options(IntrinsicsCalib PRIVATE /W4)
else()
    target_compile_options(IntrinsicsCalib PRIVATE -Wall -Wextra)
endif()

# Copy OpenCV DLLs to executable directory on Windows
if(WIN32 AND OPENCV_DLLS)
    add_custom_command(TARGET IntrinsicsCalib POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
        ${OPENCV_DLLS}
        $<TARGET_FILE_DIR:IntrinsicsCalib>
        COMMENT "Copying OpenCV DLLs to IntrinsicsCalib output directory"
    )
endif()

# Copy LASzip DLL to executable directory on Windows
if(WIN32)
    add_custom_command(TARGET IntrinsicsCalib POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
        $<TARGET_FILE:${LASZIP_LIB}>
        $<TARGET_FILE_DIR:IntrinsicsCalib>
        COMMENT "Copying LASzip DLL to IntrinsicsCalib output directory"
    )
endif()
