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
    set(CMAKE_INSTALL_RPATH "@loader_path")
    
    # Use vendored librealsense for macOS
    set(LIBREALSENSE_VENDOR_DIR "${CMAKE_SOURCE_DIR}/vendor/librealsense-install")
    
    if(NOT EXISTS "${LIBREALSENSE_VENDOR_DIR}")
        message(FATAL_ERROR 
            "Vendored librealsense not found at ${LIBREALSENSE_VENDOR_DIR}\n"
            "Please run: ./bin/build_librealsense_macos.sh")
    endif()
    
    # Find the vendored libraries
    find_library(REALSENSE2_LIB 
        NAMES realsense2 librealsense2.a
        PATHS "${LIBREALSENSE_VENDOR_DIR}/lib"
        NO_DEFAULT_PATH
        REQUIRED
    )
    
    find_library(RSUTILS_LIB 
        NAMES rsutils librsutils.a
        PATHS "${LIBREALSENSE_VENDOR_DIR}/lib"
        NO_DEFAULT_PATH
        REQUIRED
    )
    
    find_library(REALSENSE_FILE_LIB 
        NAMES realsense-file librealsense-file.a
        PATHS "${LIBREALSENSE_VENDOR_DIR}/lib"
        NO_DEFAULT_PATH
        REQUIRED
    )
    
    # Find system dependencies
    find_library(IOKIT_FRAMEWORK IOKit)
    find_library(COREFOUNDATION_FRAMEWORK CoreFoundation)
    find_library(USB_LIB usb-1.0 REQUIRED)
    
    # Create imported target
    add_library(realsense2::realsense2 STATIC IMPORTED)
    set_target_properties(realsense2::realsense2 PROPERTIES
        IMPORTED_LOCATION "${REALSENSE2_LIB}"
        INTERFACE_INCLUDE_DIRECTORIES "${LIBREALSENSE_VENDOR_DIR}/include"
        INTERFACE_LINK_LIBRARIES "${REALSENSE_FILE_LIB};${RSUTILS_LIB};${USB_LIB};${IOKIT_FRAMEWORK};${COREFOUNDATION_FRAMEWORK}"
    )
    
    message(STATUS "Using vendored librealsense: ${REALSENSE2_LIB}")
    message(STATUS "Using vendored rsutils: ${RSUTILS_LIB}")
    message(STATUS "Using vendored realsense-file: ${REALSENSE_FILE_LIB}")
else()
    set(CMAKE_INSTALL_RPATH "$ORIGIN")
    find_package(realsense2 REQUIRED)
endif()

find_package(viam-cpp-sdk REQUIRED)
find_package(libjpeg-turbo CONFIG REQUIRED)

add_library(viamrealsense src/module/encoding.cpp)
target_include_directories(viamrealsense
    PUBLIC src/module
)

target_compile_features(viamrealsense PUBLIC
    cxx_std_17
)
target_link_libraries(viamrealsense
    PUBLIC
        viam-cpp-sdk::viam-cpp-sdk
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

if (VIAM_REALSENSE_ENABLE_TESTS)
    enable_testing()
    add_subdirectory(test)
endif()
