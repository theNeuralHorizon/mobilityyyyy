#!/bin/bash
# Run the full competition simulation in headless mode (WSL2 / SSH / CI).
#
# Usage:
#   cd ~/path/to/mobilityyyyy
#   bash run_headless.sh

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
echo "=== Launching full run HEADLESS ==="
echo "    Robot spawns at START tile (-1.35, 1.80)"
echo "    Press Ctrl+C to stop."
echo ""

ros2 launch artpark_bringup full_run.launch.py headless:=true spawn_yaw:=0.0
