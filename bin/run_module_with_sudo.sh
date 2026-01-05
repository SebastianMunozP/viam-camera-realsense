#!/bin/bash
# Wrapper to run the module binary as root for device access
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
exec sudo "$SCRIPT_DIR/viam-camera-realsense" "$@"
