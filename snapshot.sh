#!/bin/bash

SNAPSHOT_DIR=~/quadruped/system_snapshot
mkdir -p $SNAPSHOT_DIR

echo "📦 Creating system snapshot..."

# System info
uname -a > $SNAPSHOT_DIR/system_info.txt
lsb_release -a >> $SNAPSHOT_DIR/system_info.txt 2>/dev/null

# APT packages (ALL)
dpkg -l > $SNAPSHOT_DIR/apt_packages.txt

# Only manually installed packages (IMPORTANT)
apt-mark showmanual > $SNAPSHOT_DIR/manual_packages.txt

# Python packages
pip3 freeze > $SNAPSHOT_DIR/pip_requirements.txt 2>/dev/null

# Environment variables
printenv > $SNAPSHOT_DIR/env_variables.txt

# Hardware + CPU info
lscpu > $SNAPSHOT_DIR/hardware_info.txt
lsblk >> $SNAPSHOT_DIR/hardware_info.txt

# Jetson-specific (if available)
dpkg-query --show | grep nvidia > $SNAPSHOT_DIR/nvidia_packages.txt 2>/dev/null

echo "✅ Snapshot saved at $SNAPSHOT_DIR"
