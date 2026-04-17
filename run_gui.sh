#!/bin/bash
# Run the full competition simulation with Gazebo GUI.
# Use this on an Ubuntu 24.04 desktop with GPU (not WSL2).
#
# Usage:
#   cd ~/path/to/mobilityyyyy
#   bash run_gui.sh
#
# Prerequisites:
#   - ROS 2 Jazzy installed
#   - Gazebo Harmonic installed
#   - pupil-apriltags: pip install --break-system-packages pupil-apriltags

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
WS_DIR="$SCRIPT_DIR/gaws_ws"

echo "=== Sourcing ROS 2 Jazzy ==="
source /opt/ros/jazzy/setup.bash

echo "=== Building workspace ==="
cd "$WS_DIR"
colcon build --symlink-install 2>&1 | tail -10
source install/setup.bash

echo ""
echo "=== Launching full run WITH GUI ==="
echo "    Robot spawns at START tile (-1.35, 1.80)"
echo "    Press Ctrl+C to stop."
echo ""

ros2 launch artpark_bringup full_run.launch.py headless:=false spawn_yaw:=0.0
