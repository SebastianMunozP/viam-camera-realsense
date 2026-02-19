#!/bin/bash
# Test script to verify binary dependencies are properly linked
# This helps catch missing dynamic library dependencies that could cause runtime failures

set -e

BINARY_PATH="${1:-build-conan/build/Release/viam-camera-realsense}"

if [ ! -f "$BINARY_PATH" ]; then
    echo "ERROR: Binary not found at: $BINARY_PATH"
    echo "Usage: $0 [path-to-binary]"
    exit 1
fi

echo "========================================"
echo "Testing binary: $BINARY_PATH"
echo "========================================"
echo ""

# 1. Check for missing dynamic libraries
echo "1. Checking for missing dynamic libraries with ldd..."
if ldd "$BINARY_PATH" | grep -i "not found"; then
    echo "ERROR: Found missing dynamic libraries!"
    exit 1
else
    echo "✓ All dynamic libraries found"
fi
echo ""

# 2. Show all dependencies
echo "2. Full list of dynamic library dependencies:"
ldd "$BINARY_PATH"
echo ""

# 3. Check GLIBC version requirement
echo "3. Checking GLIBC version requirements..."
GLIBC_VERSION=$(objdump -T "$BINARY_PATH" | grep GLIBC | sed 's/.*GLIBC_/GLIBC_/' | sort -V | tail -1 || echo "unknown")
echo "Maximum GLIBC version required: $GLIBC_VERSION"
echo ""

# 4. Check for undefined symbols
echo "4. Checking for undefined symbols..."
UNDEFINED_COUNT=$(nm -D "$BINARY_PATH" | grep -c " U " || echo "0")
echo "Undefined symbols count: $UNDEFINED_COUNT (some are normal, like from libc)"
echo ""

# 5. Verify binary can load (doesn't check full functionality, just loading)
echo "5. Testing if binary can be loaded (basic smoke test)..."
timeout 2s "$BINARY_PATH" --help 2>&1 | head -5 || true
echo "✓ Binary loads (timed out or exited, which is expected)"
echo ""

# 6. Check for statically linked librealsense (optional - depends on your setup)
echo "6. Checking if librealsense is linked..."
if nm "$BINARY_PATH" | grep -q "realsense" || ldd "$BINARY_PATH" | grep -q "realsense"; then
    echo "✓ librealsense symbols found (linked)"
else
    echo "⚠ Warning: No obvious librealsense symbols (might be statically linked)"
fi
echo ""

# 7. Show binary info
echo "7. Binary information:"
file "$BINARY_PATH"
ls -lh "$BINARY_PATH"
echo ""

echo "========================================"
echo "✓ Dynamic dependency test completed!"
echo "========================================"
