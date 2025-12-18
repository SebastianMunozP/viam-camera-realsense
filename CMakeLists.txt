cmake_minimum_required(VERSION 3.25 FATAL_ERROR)
set(CMAKE_PROJECT_VERSION 0.0.1)
set(CMAKE_CXX_STANDARD 17)


project(viam-camera-realsense
    DESCRIPTION "Viam Module for the Intel RealSense Camera"
    HOMEPAGE_URL https://github.com/viam-modules/viam-camera-realsense
    LANGUAGES CXX
)

include(cmake/simd_flags.cmake)

# option(VIAM_REALSENSE_ENABLE_SANITIZER "Enable AddressSanitizer" OFF)

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

add_executable(viam-camera-realsense src/module/main.cpp src/module/encoding.cpp)
target_link_libraries(viam-camera-realsense
    PUBLIC
        viam-cpp-sdk::viamsdk
        realsense2::realsense2
    PRIVATE
        libjpeg-turbo::turbojpeg-static
        ${CMAKE_DL_LIBS}
)
# target_link_libraries(viam-camera-realsense PRIVATE viamrealsense)

# install(TARGETS viamrealsense
#     LIBRARY DESTINATION lib
#     ARCHIVE DESTINATION lib
# )

install(TARGETS viam-camera-realsense DESTINATION .)
install( FILES meta.json DESTINATION .)

# set(CPACK_PACKAGE_NAME "viam-realsense")
# set(CPACK_PACKAGE_FILE_NAME "module")
# set(CPACK_GENERATOR "TGZ")
# set(CPACK_INCLUDE_TOPLEVEL_DIRECTORY 0)
# include(CPack)

# if(VIAM_REALSENSE_ENABLE_SANITIZER)
#     message(STATUS "Building with AddressSanitizer enabled")
#     target_compile_options(viamrealsense PUBLIC
#         -fsanitize=address
#         -fsanitize-address-use-after-scope
#         -fno-omit-frame-pointer
#         -static-libasan
#     )

#     target_link_options(viamrealsense PUBLIC
#         -fsanitize=address
#         -fno-omit-frame-pointer
#         -static-libasan
#     )
# endif()

# if (NOT MSVC)
#     target_compile_options(viamrealsense
#         PRIVATE
#             -Wredundant-move
#             -Wpessimizing-move
#     )
# endif()


enable_testing()
add_subdirectory(test)
