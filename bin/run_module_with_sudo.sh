#!/bin/bash
# Wrapper to run the module binary as root for device access
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

# Check if sudoers rule exists (test if we can run sudo without password)
if ! sudo -n true 2>/dev/null; then
    echo "ERROR: Permissions not configured for RealSense module" >&2
    echo "Please run: curl -fsSL https://raw.githubusercontent.com/viam-modules/viam-camera-realsense/main/install-macos-permissions.sh | sudo bash" >&2
    exit 1
fi

exec sudo "$SCRIPT_DIR/viam-camera-realsense" "$@"