cmake_minimum_required(VERSION 3.12)

# tools/replay 独立构建（不修改根 CMakeLists.txt）
# 与在线版本共用 detect/armor_corner_optimizer.cpp、detect/corner_refine_submodule.hpp（策略函数）
# 和 mathutils/CoordTransformer.cpp，不复制粘贴算法代码（B-2 要求）。
#
# 构建：
#   cmake -S tools/replay -B build-replay
#   cmake --build build-replay -j
#
# 产物：build-replay/replay_tool、build-replay/avi_to_r1

project(tools_replay CXX)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(REPO_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../..)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

# 注意：单线程确定性要求，不链接 OpenMP（B-2 验收①）
add_executable(replay_tool
    replay_main.cpp
    ${REPO_ROOT}/mathutils/CoordTransformer.cpp
    ${REPO_ROOT}/detect/armor_corner_optimizer.cpp
)

target_include_directories(replay_tool PRIVATE
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${REPO_ROOT}/common
    ${REPO_ROOT}/mathutils
    ${REPO_ROOT}/detect
)

target_link_libraries(replay_tool ${OpenCV_LIBS} Eigen3::Eigen)

add_executable(avi_to_r1
    avi_to_r1.cpp
)

target_include_directories(avi_to_r1 PRIVATE
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${REPO_ROOT}/common
)

target_link_libraries(avi_to_r1 ${OpenCV_LIBS})

# B-4 验收③：单图点选角点 + 与在线相同的 PnP，对照卷尺实测距离
add_executable(pnp_check
    pnp_check.cpp
    ${REPO_ROOT}/mathutils/CoordTransformer.cpp
)

target_include_directories(pnp_check PRIVATE
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${REPO_ROOT}/common
    ${REPO_ROOT}/mathutils
)

target_link_libraries(pnp_check ${OpenCV_LIBS} Eigen3::Eigen)
