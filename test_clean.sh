#!/bin/bash
source /opt/ros/jazzy/setup.bash
source ~/gaws_ws/install/setup.bash

# HARD cleanup - kill everything
echo "=== CLEANUP ==="
pkill -9 -f ruby 2>/dev/null || true
pkill -9 -f "gz sim" 2>/dev/null || true
pkill -9 -f parameter_bridge 2>/dev/null || true
pkill -9 -f state_machine 2>/dev/null || true
pkill -9 -f obstacle_monitor 2>/dev/null || true
pkill -9 -f tile_tracker 2>/dev/null || true
pkill -9 -f apriltag_handler 2>/dev/null || true
pkill -9 -f floor_logo 2>/dev/null || true
pkill -9 -f logger_node 2>/dev/null || true
pkill -9 -f robot_state_publisher 2>/dev/null || true
pkill -9 -f ros2 2>/dev/null || true
sleep 3

# Verify clean state
echo "Remaining gz processes: $(pgrep -c ruby 2>/dev/null || echo 0)"
echo "Remaining ros2 processes: $(pgrep -c ros2 2>/dev/null || echo 0)"

echo "=== LAUNCHING FRESH ==="
ros2 launch artpark_bringup full_run.launch.py debug_hold:=false 2>/dev/null &
LAUNCH_PID=$!

# Wait for Gazebo + robot spawn + perception nodes
sleep 25

echo "=== NODES ==="
ros2 node list 2>/dev/null

echo "=== ODOM (t=0) ==="
timeout 3 ros2 topic echo /odom --once --field pose.pose.position 2>/dev/null || echo "NO_ODOM"

echo "=== SCAN HZ ==="
timeout 5 ros2 topic hz /scan 2>&1 | head -2

echo "=== FRONT CAM HZ ==="
timeout 5 ros2 topic hz /front_cam/image_raw 2>&1 | head -2

echo "=== SM PHASE ==="
timeout 3 ros2 topic echo /thought --once --field phase 2>/dev/null || echo "NO_THOUGHT"
timeout 3 ros2 topic echo /thought --once --field hypothesis 2>/dev/null || echo ""

echo "=== Waiting 60s ==="
sleep 60

echo "=== ODOM (t=60s) ==="
timeout 3 ros2 topic echo /odom --once --field pose.pose.position 2>/dev/null || echo "NO_ODOM"

echo "=== SM PHASE NOW ==="
timeout 3 ros2 topic echo /thought --once --field phase 2>/dev/null || echo "NO_THOUGHT"
timeout 3 ros2 topic echo /thought --once --field hypothesis 2>/dev/null || echo ""

echo "=== CMD_VEL ==="
timeout 3 ros2 topic echo /cmd_vel --once 2>/dev/null || echo "NO_CMD"

echo "=== SCORECARD ==="
LATEST=$(ls -td ~/artpark_runs/*/ 2>/dev/null | head -1)
if [ -n "$LATEST" ]; then
    echo "Run: $LATEST"
    cat "$LATEST/judge_scorecard.csv"
else
    echo "No run dir"
fi

echo "=== CLEANUP ==="
kill $LAUNCH_PID 2>/dev/null
pkill -9 -f ruby 2>/dev/null || true
pkill -9 -f parameter_bridge 2>/dev/null || true
pkill -9 -f state_machine 2>/dev/null || true
pkill -9 -f obstacle_monitor 2>/dev/null || true
sleep 1
echo "CLEAN_TEST_DONE"
