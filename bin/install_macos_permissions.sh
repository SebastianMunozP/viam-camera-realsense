#!/bin/bash
# install-macos-permissions.sh
# Sets up passwordless sudo for RealSense module on macOS

set -e

echo "Setting up permissions for Viam RealSense module on macOS..."

# Create sudoers rule
cat << 'EOF' | sudo tee /etc/sudoers.d/viam-realsense > /dev/null
# Allow all users to run viam-camera-realsense without password
ALL ALL=(root) NOPASSWD: /Users/*/.*viam*/packages/.data/module/*/viam-camera-realsense
ALL ALL=(root) NOPASSWD: /Users/*/.viam/packages/.data/module/*/viam-camera-realsense
EOF

# Set correct permissions
sudo chmod 0440 /etc/sudoers.d/viam-realsense

# Validate
if sudo visudo -c -f /etc/sudoers.d/viam-realsense; then
    echo "✓ Permissions configured successfully"
    echo "✓ RealSense module can now run without password prompts"
else
    echo "✗ Error in sudoers configuration"
    sudo rm /etc/sudoers.d/viam-realsense
    exit 1
fi