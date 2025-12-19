#!/bin/bash
echo "Executing module with sudo from $SCRIPT_DIR"
# Wrapper to run the module binary as root for device access
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
echo "Executing module with sudo from $SCRIPT_DIR"
echo "EUID before sudo: $EUID"
# Use /bin/sh -c to ensure DYLD_LIBRARY_PATH is preserved after sudo
exec sudo /bin/sh -c "export DYLD_LIBRARY_PATH=\"$SCRIPT_DIR/../lib:\$DYLD_LIBRARY_PATH\"; exec \"$SCRIPT_DIR/viam-camera-realsense\" \"\$@\"" -- "$@"
