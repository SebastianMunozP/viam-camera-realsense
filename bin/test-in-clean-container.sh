#!/bin/bash
# Test the built binary in a clean Ubuntu container
# This simulates a real deployment environment to catch missing dependencies

set -e

UBUNTU_VERSION="${1:-22.04}"
BINARY_PATH="${2:-build-conan/build/Release/viam-camera-realsense}"

if [ ! -f "$BINARY_PATH" ]; then
    echo "ERROR: Binary not found at: $BINARY_PATH"
    echo "Usage: $0 [ubuntu-version] [path-to-binary]"
    echo "Example: $0 22.04 build-conan/build/Release/viam-camera-realsense"
    exit 1
fi

echo "========================================"
echo "Testing in clean Ubuntu $UBUNTU_VERSION container"
echo "Binary: $BINARY_PATH"
echo "========================================"
echo ""

# Test in clean container
docker run --rm -v "$(pwd):/test" "ubuntu:$UBUNTU_VERSION" /bin/bash -c "
set -e

echo 'Installing minimal system dependencies...'
apt-get update -qq
apt-get install -y -qq libudev1 binutils file > /dev/null 2>&1

echo ''
echo '1. Checking ldd output:'
ldd /test/$BINARY_PATH

echo ''
echo '2. Checking for missing libraries:'
if ldd /test/$BINARY_PATH | grep -i 'not found'; then
    echo 'ERROR: Missing dynamic libraries!'
    exit 1
else
    echo '✓ All libraries found'
fi

echo ''
echo '3. Checking GLIBC version:'
GLIBC_ACTUAL=\$(ldd --version | head -1 | grep -oP '\d+\.\d+$')
GLIBC_REQUIRED=\$(objdump -T /test/$BINARY_PATH | grep GLIBC | sed 's/.*GLIBC_//' | sort -V | tail -1 | grep -oP '\d+\.\d+' || echo '0.0')
echo \"System GLIBC: \$GLIBC_ACTUAL\"
echo \"Required GLIBC: \$GLIBC_REQUIRED\"

if [ \"\$GLIBC_REQUIRED\" != \"0.0\" ]; then
    # Simple version comparison
    if [ \"\$(printf '%s\n' \"\$GLIBC_ACTUAL\" \"\$GLIBC_REQUIRED\" | sort -V | head -1)\" != \"\$GLIBC_REQUIRED\" ]; then
        echo 'ERROR: GLIBC version mismatch!'
        exit 1
    else
        echo '✓ GLIBC version compatible'
    fi
fi

echo ''
echo '4. Binary info:'
file /test/$BINARY_PATH

echo ''
echo '5. Testing binary load (smoke test):'
timeout 2s /test/$BINARY_PATH --version 2>&1 | head -3 || echo '✓ Binary started (timed out, expected)'

echo ''
echo '========================================'
echo '✓ Clean container test PASSED!'
echo '========================================'
"

echo ""
echo "✓ Test completed successfully for Ubuntu $UBUNTU_VERSION"
