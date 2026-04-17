#!/bin/bash
source /opt/ros/jazzy/setup.bash
source ~/gaws_ws/install/setup.bash

pkill -9 -f ruby 2>/dev/null || true
pkill -9 -f parameter_bridge 2>/dev/null || true
pkill -9 -f state_machine 2>/dev/null || true
pkill -9 -f obstacle_monitor 2>/dev/null || true
sleep 2

echo "=== LAUNCHING FULL RUN ==="
ros2 launch artpark_bringup full_run.launch.py debug_hold:=false 2>/dev/null &
LAUNCH_PID=$!

sleep 20

echo "=== NODES ==="
ros2 node list 2>/dev/null

echo "=== ODOM START ==="
timeout 3 ros2 topic echo /odom --once --field pose.pose.position 2>/dev/null || echo "NO_ODOM"

echo "=== Waiting 60s for exploration ==="
sleep 60

echo "=== ODOM AFTER 60s ==="
timeout 3 ros2 topic echo /odom --once --field pose.pose.position 2>/dev/null || echo "NO_ODOM"

echo "=== CMD_VEL ==="
timeout 3 ros2 topic echo /cmd_vel --once 2>/dev/null || echo "NO_CMD"

echo "=== LATEST THOUGHTS ==="
timeout 3 ros2 topic echo /thought --once --field hypothesis 2>/dev/null || echo "NO_THOUGHT"

echo "=== SCORECARD ==="
LATEST=$(ls -td ~/artpark_runs/*/ 2>/dev/null | head -1)
if [ -n "$LATEST" ]; then
    echo "Run dir: $LATEST"
    cat "$LATEST/judge_scorecard.csv"
else
    echo "No run dir"
fi

echo "=== SCAN CHECK ==="
timeout 3 ros2 topic hz /scan 2>&1 | head -3

echo "=== FRONT CAM CHECK ==="
timeout 3 ros2 topic hz /front_cam/image_raw 2>&1 | head -3

kill $LAUNCH_PID 2>/dev/null
pkill -9 -f ruby 2>/dev/null || true
pkill -9 -f parameter_bridge 2>/dev/null || true
pkill -9 -f state_machine 2>/dev/null || true
sleep 1
echo "FULL_TEST_DONE"
