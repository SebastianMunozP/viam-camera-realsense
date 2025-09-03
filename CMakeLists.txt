cmake_minimum_required(VERSION 3.25 FATAL_ERROR)

project(viam-camera-realsense
    DESCRIPTION "Viam Module for the Intel RealSense Camera"
    HOMEPAGE_URL https://github.com/viam-modules/viam-camera-realsense
    LANGUAGES CXX
)
set(CMAKE_CXX_STANDARD 17)

# Define an option for enabling build with AddressSanitizer
option(CMAKE_BUILD_TYPE RelWithDebInfo)

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)
find_package(realsense2 REQUIRED)
find_package(viam-cpp-sdk REQUIRED)
# construct the library
add_library(viamrealsense src/module/encoding.cpp) 
# Make include paths explicit
target_include_directories(viamrealsense PRIVATE 
    ${CMAKE_CURRENT_SOURCE_DIR}/src/module
)
target_link_libraries(viamrealsense PUBLIC viam-cpp-sdk::viamsdk realsense2::realsense2 PRIVATE turbojpeg ${CMAKE_DL_LIBS})

# Conditionally apply SIMD flags based on the processor architecture for fpng acceleration
if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64|AMD64")
    if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
        target_compile_options(viamrealsense PRIVATE -msse4.1 -maes -mpclmul)
    elseif(CMAKE_CXX_COMPILER_ID STREQUAL "Intel")
        target_compile_options(viamrealsense PRIVATE -xsse4.1 -maes -mpclmul)
    elseif(CMAKE_CXX_COMPILER_ID STREQUAL "MSVC")
        target_compile_options(viamrealsense PRIVATE /arch:SSE2)
    endif()
endif()

add_executable(viam-camera-realsense src/module/main.cpp)
target_link_libraries(viam-camera-realsense PRIVATE viamrealsense)

# Add shared flags
target_compile_features(viamrealsense PUBLIC
    cxx_std_17
)
target_compile_options(viamrealsense PRIVATE 
    -O2 
    -Wredundant-move 
    -Wpessimizing-move 
)

enable_testing()
add_subdirectory(test)

