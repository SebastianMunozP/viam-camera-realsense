cmake_minimum_required(VERSION 3.15)
project(viam-camera-realsense-cli)

find_package(realsense2 REQUIRED PATHS /Users/sebastian.munoz/Repos/librealsense/build/Debug)
# find_package(realsense2 REQUIRED)
add_executable(viam-camera-realsense-cli main.cpp)
target_link_libraries(viam-camera-realsense-cli PRIVATE realsense2::realsense2)