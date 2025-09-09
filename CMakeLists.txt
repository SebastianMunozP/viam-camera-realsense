cmake_minimum_required(VERSION 3.25 FATAL_ERROR)

project(viam-camera-realsense
    DESCRIPTION "Viam Module for the Intel RealSense Camera"
    HOMEPAGE_URL https://github.com/viam-modules/viam-camera-realsense
    LANGUAGES CXX
)
set(CMAKE_CXX_STANDARD 17)

# Define an option for enabling build with AddressSanitizer
option(ENABLE_SANITIZER "Enable AddressSanitizer" OFF)
option(ENABLE_PROFILING "Enable gperftools profiling" OFF)

if(ENABLE_SANITIZER)
    message(STATUS "Building with AddressSanitizer enabled")
endif()

if(ENABLE_PROFILING)
    message(STATUS "Building with gperftools profiling enabled")
endif()

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

if(ENABLE_SANITIZER)
    add_compile_options(
        -fsanitize=address
        -fsanitize-address-use-after-scope
        -fno-omit-frame-pointer
    )
    add_link_options(
        -fsanitize=address
        -fno-omit-frame-pointer
    )
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -static-libasan")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -static-libasan -fno-omit-frame-pointer")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -static-libasan")

    target_compile_options(viamrealsense PRIVATE
        -fsanitize=address
        -fsanitize-address-use-after-scope
        -fno-omit-frame-pointer
    )
    target_link_options(viamrealsense PRIVATE
        -fsanitize=address
        -fno-omit-frame-pointer
    )
    target_compile_options(viam-camera-realsense PRIVATE
        -fsanitize=address
        -fsanitize-address-use-after-scope
        -fno-omit-frame-pointer
    )
    target_link_options(viam-camera-realsense PRIVATE
        -fsanitize=address
        -fno-omit-frame-pointer
    )
endif()

# Apply profiling to all targets if enabled
if(ENABLE_PROFILING)
    # Find gperftools
    find_library(PROFILER_LIBRARY profiler)
    find_library(TCMALLOC_LIBRARY tcmalloc)
    
    if(NOT PROFILER_LIBRARY)
        message(FATAL_ERROR "gperftools profiler library not found. Install with: apt install google-perftools libgoogle-perftools-dev")
    endif()
    
    if(NOT TCMALLOC_LIBRARY)
        message(FATAL_ERROR "gperftools tcmalloc library not found. Install with: apt install google-perftools libgoogle-perftools-dev")
    endif()
    
    message(STATUS "Found profiler: ${PROFILER_LIBRARY}")
    message(STATUS "Found tcmalloc: ${TCMALLOC_LIBRARY}")
    
    # Add profiling to library
    target_link_libraries(viamrealsense PRIVATE ${PROFILER_LIBRARY} ${TCMALLOC_LIBRARY})
    target_compile_definitions(viamrealsense PRIVATE ENABLE_PROFILING)
    
    # Add profiling to executable
    target_link_libraries(viam-camera-realsense PRIVATE ${PROFILER_LIBRARY} ${TCMALLOC_LIBRARY})
    target_compile_definitions(viam-camera-realsense PRIVATE ENABLE_PROFILING)
endif()# Add shared flags

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
