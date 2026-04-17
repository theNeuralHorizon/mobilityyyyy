#!/bin/bash
set -e

# Resolve workspace root: this script lives at gaws_ws/build_and_run.sh
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
WS_DIR="$SCRIPT_DIR"

echo "=== Setting up ROS 2 Jazzy environment ==="
source /opt/ros/jazzy/setup.bash

echo "=== Installing Python dependencies ==="
pip install --break-system-packages pupil-apriltags numpy opencv-python-headless 2>/dev/null || \
pip install --break-system-packages pupil-apriltags numpy 2>/dev/null || true

echo "=== Initializing rosdep ==="
sudo rosdep init 2>/dev/null || true
rosdep update 2>/dev/null || true

echo "=== Installing ROS dependencies ==="
cd "$WS_DIR"
rosdep install --from-paths src --ignore-src -r -y --skip-keys "ament_python" 2>&1 | tail -5

echo "=== Building workspace ==="
colcon build --symlink-install 2>&1 | tail -20
source install/setup.bash

echo ""
echo "=== Build complete ==="
echo ""
echo "To run HEADLESS (WSL2 / SSH / CI):"
echo "  source $WS_DIR/install/setup.bash"
echo "  ros2 launch artpark_bringup full_run.launch.py"
echo ""
echo "To run WITH GUI (Ubuntu desktop):"
echo "  source $WS_DIR/install/setup.bash"
echo "  ros2 launch artpark_bringup full_run.launch.py headless:=false"
echo ""
