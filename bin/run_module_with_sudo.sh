#!/bin/bash
echo "Executing module with sudo from $SCRIPT_DIR"
# Wrapper to run the module binary as root for device access
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
echo "Executing module with sudo from $SCRIPT_DIR"
echo "EUID before sudo: $EUID"
exec sudo "$SCRIPT_DIR/viam-camera-realsense" "$@"