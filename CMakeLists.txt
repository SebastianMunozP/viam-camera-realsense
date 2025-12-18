cmake_minimum_required(VERSION 3.22 FATAL_ERROR)
set(CMAKE_PROJECT_VERSION 0.0.1)
set(CMAKE_CXX_STANDARD 17)


project(viam-camera-realsense
    DESCRIPTION "Viam Module for the Intel RealSense Camera"
    HOMEPAGE_URL https://github.com/viam-modules/viam-camera-realsense
    LANGUAGES CXX
)

include(cmake/simd_flags.cmake)

if (APPLE)
    list(PREPEND CMAKE_INSTALL_RPATH "@loader_path/../lib")
else()
    list(PREPEND CMAKE_INSTALL_RPATH "$ORIGIN/../lib")
endif()

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)
find_package(realsense2 REQUIRED)
find_package(viam-cpp-sdk REQUIRED)
find_package(libjpeg-turbo CONFIG REQUIRED)
find_package(PkgConfig REQUIRED)

add_executable(viam-camera-realsense src/module/main.cpp src/module/encoding.cpp)
target_link_libraries(viam-camera-realsense
    PUBLIC
        viam-cpp-sdk::viamsdk
        realsense2::realsense2
    PRIVATE
        libjpeg-turbo::turbojpeg-static
        ${CMAKE_DL_LIBS}
)

install(TARGETS viam-camera-realsense DESTINATION .)
install( FILES meta.json DESTINATION .)

add_executable(viam-camera-realsense-cli src/cli/main.cpp)
target_link_libraries(viam-camera-realsense-cli
    PUBLIC
        realsense2::realsense2
    PRIVATE
        ${CMAKE_DL_LIBS}
)

enable_testing()
add_subdirectory(test)
