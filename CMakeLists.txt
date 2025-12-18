cmake_minimum_required(VERSION 3.22 FATAL_ERROR)
set(CMAKE_PROJECT_VERSION 0.0.1)


project(viam-camera-realsense
    DESCRIPTION "Viam Module for the Intel RealSense Camera"
    HOMEPAGE_URL https://github.com/viam-modules/viam-camera-realsense
    LANGUAGES CXX
)

include(cmake/simd_flags.cmake)

option(VIAM_REALSENSE_ENABLE_TESTS "Enable unit tests" OFF)

if (APPLE)
    list(PREPEND CMAKE_INSTALL_RPATH "@loader_path/../lib")
else()
    list(PREPEND CMAKE_INSTALL_RPATH "$ORIGIN/../lib")
endif()

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)
# Manual configuration for local librealsense to avoid broken Config file
set(LRS_ROOT "/Users/sebastian.munoz/Repos/librealsense")

# Find Headers
find_path(realsense2_INCLUDE_DIR 
    NAMES librealsense2/rs.hpp
    PATHS "${LRS_ROOT}/include"
    NO_DEFAULT_PATH
)

# Find Library
find_library(realsense2_LIBRARY 
    NAMES realsense2
    PATHS "${LRS_ROOT}/build/Debug" "${LRS_ROOT}/build"
    NO_DEFAULT_PATH
)

if (realsense2_INCLUDE_DIR AND realsense2_LIBRARY)
    message(STATUS "Found local realsense2 headers: ${realsense2_INCLUDE_DIR}")
    message(STATUS "Found local realsense2 library: ${realsense2_LIBRARY}")
    
    # Define the target expected by the module
    if (NOT TARGET realsense2::realsense2)
        add_library(realsense2::realsense2 UNKNOWN IMPORTED)
        set_target_properties(realsense2::realsense2 PROPERTIES
            IMPORTED_LOCATION "${realsense2_LIBRARY}"
            INTERFACE_INCLUDE_DIRECTORIES "${realsense2_INCLUDE_DIR}"
        )
    endif()

    # FORCE local headers to the front using CXX_FLAGS
    # This overrides any order CMake decides for target_include_directories or system paths
    set(CMAKE_CXX_FLAGS "-I${realsense2_INCLUDE_DIR} ${CMAKE_CXX_FLAGS}")
else()
    message(FATAL_ERROR "Could not find local librealsense in ${LRS_ROOT}")
endif()
find_package(viam-cpp-sdk REQUIRED)
find_package(libjpeg-turbo CONFIG REQUIRED)
find_package(PkgConfig REQUIRED)

add_library(viamrealsense src/module/encoding.cpp)
target_include_directories(viamrealsense
    PUBLIC src/module
    
)

target_compile_features(viamrealsense PUBLIC
    cxx_std_17
)
target_link_libraries(viamrealsense
    PUBLIC
        realsense2::realsense2
        libjpeg-turbo::turbojpeg-static
    PRIVATE
        ${CMAKE_DL_LIBS}
)
add_executable(viam-camera-realsense src/module/main.cpp)
target_link_libraries(viam-camera-realsense PRIVATE viamrealsense)

install(TARGETS 
    viamrealsense 
    viam-camera-realsense 
    DESTINATION .)
install( FILES meta.json DESTINATION .)

add_executable(viam-camera-realsense-cli src/cli/main.cpp)
target_link_libraries(viam-camera-realsense-cli
    PUBLIC
        realsense2::realsense2
    PRIVATE
        ${CMAKE_DL_LIBS}
)

add_executable(viam-camera-realsense-cli src/cli/main.cpp)
target_link_libraries(viam-camera-realsense-cli
    PUBLIC
        realsense2::realsense2
    PRIVATE
        ${CMAKE_DL_LIBS}
)

enable_testing()
add_subdirectory(test)
