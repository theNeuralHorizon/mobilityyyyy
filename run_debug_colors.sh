#!/bin/bash
# Run simulation with floor cam debug images enabled.
# Saves raw + mask images to /tmp/floor_cam_debug/ every ~5 seconds.
# Use these to calibrate HSV thresholds for your specific rendering setup.
#
# After running for ~30 seconds, check:
#   ls /tmp/floor_cam_debug/
#   eog /tmp/floor_cam_debug/frame_000000_raw.png   # raw camera image
#   eog /tmp/floor_cam_debug/frame_000000_green.png  # green detection mask
#   eog /tmp/floor_cam_debug/frame_000000_orange.png # orange detection mask

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
WS_DIR="$SCRIPT_DIR/gaws_ws"

source /opt/ros/jazzy/setup.bash
cd "$WS_DIR"
colcon build --symlink-install 2>&1 | tail -10
source install/setup.bash

echo "=== DEBUG MODE: Floor cam images -> /tmp/floor_cam_debug/ ==="
echo "    Run for ~30s, then Ctrl+C and check the images."
echo ""

# Override the debug_save_images param at launch time
ros2 launch artpark_bringup full_run.launch.py headless:=true spawn_yaw:=0.0 &
LAUNCH_PID=$!

# Wait for nodes to start
sleep 12

# Set the debug parameter on the running node
ros2 param set /floor_logo_detector debug_save_images true 2>/dev/null || true

echo ""
echo "Debug image saving enabled. Let it run for 30+ seconds."
echo "Images saved to /tmp/floor_cam_debug/"
echo "Press Ctrl+C to stop."

wait $LAUNCH_PID
