#!/bin/bash
# Complete build + headless-run script for artpark_bot on ROS2 Jazzy / Gazebo Harmonic
# Run this INSIDE Ubuntu-24.04 WSL after ROS2 Jazzy + ros-jazzy-ros-gz are installed.
set -e

SOURCE_WIN="/mnt/c/Users/kshit/OneDrive/Desktop/mobilityyyyy/gaws_ws"
WS="$HOME/gaws_ws"

echo "=== [1/5] Copying workspace from Windows to WSL home (fast build) ==="
rm -rf "$WS"
cp -r "$SOURCE_WIN" "$WS"
echo "    Done: $WS"

echo "=== [2/5] Installing Python dependencies ==="
pip install --break-system-packages --quiet pupil-apriltags opencv-python-headless 2>/dev/null || \
  pip install --break-system-packages pupil-apriltags opencv-python-headless
echo "    pupil-apriltags + opencv installed"

echo "=== [3/5] rosdep install ==="
source /opt/ros/jazzy/setup.bash
cd "$WS"
rosdep install --from-paths src --ignore-src -r -y \
  --skip-keys "python3-opencv python3-numpy" 2>&1 | tail -5

echo "=== [4/5] colcon build ==="
cd "$WS"
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release 2>&1 | tail -30
echo "    Build done"

echo "=== [5/5] Sourcing install ==="
source "$WS/install/setup.bash"
echo ""
echo "BUILD COMPLETE. To run headlessly:"
echo ""
echo "  source $WS/install/setup.bash"
echo "  ros2 launch artpark_bringup full_run.launch.py"
echo ""
echo "To monitor the robot:"
echo "  ros2 topic echo /thought --no-arr  (in another terminal)"
echo "  ros2 topic echo /scorecard_event   (in another terminal)"
