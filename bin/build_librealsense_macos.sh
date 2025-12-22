#!/bin/bash
set -e

# Script to build librealsense for macOS with the specific commit needed for the fix
# This creates a vendored library that can be committed to the repo

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
VENDOR_DIR="$REPO_ROOT/vendor"
LRS_COMMIT="67d20df877e9a049332ba4e104ba307f6e8b93e6"

echo "Building librealsense for macOS..."
echo "Commit: $LRS_COMMIT"
echo "Vendor directory: $VENDOR_DIR"

# Create vendor directory
mkdir -p "$VENDOR_DIR"
cd "$VENDOR_DIR"

# Clone librealsense if not already present
if [ ! -d "librealsense" ]; then
    echo "Cloning librealsense..."
    git clone https://github.com/IntelRealSense/librealsense.git
fi

cd librealsense
git fetch
git checkout "$LRS_COMMIT"

# Create build directory
rm -rf build
mkdir build
cd build

# Configure with CMake - static library, minimal dependencies
# Disable ROS bag support and other features to avoid linking issues
cmake .. \
    -DCMAKE_BUILD_TYPE=Release \
    -DBUILD_SHARED_LIBS=OFF \
    -DBUILD_EXAMPLES=OFF \
    -DBUILD_GRAPHICAL_EXAMPLES=OFF \
    -DBUILD_UNIT_TESTS=OFF \
    -DFORCE_RSUSB_BACKEND=ON \
    -DBUILD_WITH_STATIC_CRT=OFF \
    -DBUILD_EASYLOGGINGPP=ON \
    -DBUILD_PYTHON_BINDINGS=OFF \
    -DBUILD_NODEJS_BINDINGS=OFF \
    -DBUILD_CV_EXAMPLES=OFF \
    -DBUILD_PCL_EXAMPLES=OFF \
    -DBUILD_GLSL_EXTENSIONS=OFF \
    -DBUILD_NETWORK_DEVICE=OFF \
    -DBUILD_TOOLS=OFF \
    -DBUILD_WITH_ROS_BAG=OFF \
    -DCMAKE_INSTALL_PREFIX="$VENDOR_DIR/librealsense-install"

# Build and install
make -j$(sysctl -n hw.ncpu)
make install

echo ""
echo "✅ librealsense built successfully!"
echo "Installation directory: $VENDOR_DIR/librealsense-install"
echo ""
echo "Next steps:"
echo "1. Update CMakeLists.txt to use the vendored library"
echo "2. Commit vendor/librealsense-install to the repo"
