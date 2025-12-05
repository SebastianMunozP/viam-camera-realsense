# Custom CMake toolchain for librealsense on ARM64/Linux
# This disables the RSUSB backend to use Native V4L backend instead
#
# The RSUSB backend uses a user-space USB implementation that has
# limitations on ARM64 platforms (especially Jetson). The Native V4L
# backend uses kernel drivers and provides better performance.

# Only set this for librealsense specifically
if(PROJECT_NAME STREQUAL "realsense2")
    message(STATUS "ARM64: Forcing Native V4L backend for librealsense (disabling RSUSB)")
    set(FORCE_RSUSB_BACKEND OFF CACHE BOOL "Use Native V4L backend on ARM64" FORCE)
endif()
