# Conan Profiles for viam-camera-realsense

## ARM64 Profile (`arm64-profile`)

The ARM64 profile is specifically configured for building on ARM64/Linux platforms (including NVIDIA Jetson devices).

### Key Configuration

The profile includes a custom CMake toolchain (`librealsense_arm64_toolchain.cmake`) that disables the RSUSB backend in librealsense. This is **critical** for proper camera operation on ARM64 platforms.

### Why This Matters

librealsense supports two USB backends:

1. **RSUSB Backend** (user-space): Default on Windows/macOS, optional on Linux
   - Pros: Works everywhere, no kernel patches needed
   - Cons: Limited performance on ARM64, may fail to access cameras on Jetson

2. **Native V4L Backend** (kernel-based): Default on Linux x86_64
   - Pros: Better performance, proper kernel driver integration
   - Cons: Requires kernel with RealSense patches (standard on most modern Linux)

### Usage

When building for ARM64, use this profile:

```bash
conan create . --profile=etc/conan/arm64-profile --build=missing
```

Or in the Makefile (for CI/CD):

```bash
make module.tar.gz CONAN_PROFILE=etc/conan/arm64-profile
```

### Technical Details

The `librealsense_arm64_toolchain.cmake` file is injected into the Conan build using:

```
[conf]
tools.cmake.cmaketoolchain:user_toolchain=["{{profile_dir}}/librealsense_arm64_toolchain.cmake"]
```

This ensures that when librealsense is built from source, it receives:

```cmake
set(FORCE_RSUSB_BACKEND OFF CACHE BOOL "Use Native V4L backend on ARM64" FORCE)
```

This forces the Native V4L backend to be used, which provides proper camera access on ARM64 platforms.
