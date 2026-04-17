#!/bin/bash
source /opt/ros/jazzy/setup.bash
source ~/gaws_ws/install/setup.bash

pkill -9 -f ruby 2>/dev/null || true
sleep 1

export LIBGL_ALWAYS_SOFTWARE=1
export MESA_GL_VERSION_OVERRIDE=3.3
export QT_QPA_PLATFORM=offscreen

WORLD=$(ros2 pkg prefix grid_world --share)/worlds/grid_world_FINAL.sdf
echo "WORLD=$WORLD"

gz sim -r -s -v 1 "$WORLD" &
sleep 8

pgrep -c ruby && echo "GZ_ALIVE" || echo "GZ_DEAD"

# Clock bridge
ros2 run ros_gz_bridge parameter_bridge '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock' &
sleep 3

timeout 2 ros2 topic echo /clock --once --field clock.sec 2>/dev/null && echo "CLOCK_OK" || echo "NO_CLOCK"

# Start obstacle monitor + state machine
ros2 run artpark_perception obstacle_monitor --ros-args -p use_sim_time:=true &
ros2 run artpark_decision state_machine --ros-args -p use_sim_time:=true -p v_explore:=0.20 -p wall_target_m:=0.40 -p wall_kp:=1.0 -p wall_kd:=0.5 &
ros2 run artpark_decision tile_tracker --ros-args -p use_sim_time:=true -p spawn_world_x:=-1.35 -p spawn_world_y:=1.80 &

# Spawn robot via launch
ros2 launch artpark_robot robot.launch.py spawn_x:=-1.35 spawn_y:=1.80 spawn_yaw:=0.0 &
sleep 8

echo "=== NODES ==="
ros2 node list 2>/dev/null

echo "=== ODOM ==="
timeout 3 ros2 topic echo /odom --once --field pose.pose.position 2>/dev/null || echo "NO_ODOM"

echo "=== Waiting 40s ==="
sleep 40

echo "=== ODOM AFTER 40s ==="
timeout 3 ros2 topic echo /odom --once --field pose.pose.position 2>/dev/null || echo "NO_ODOM"

echo "=== CMD_VEL ==="
timeout 3 ros2 topic echo /cmd_vel --once 2>/dev/null || echo "NO_CMD"

echo "=== SCORECARD ==="
LATEST=$(ls -td ~/artpark_runs/*/ 2>/dev/null | head -1)
if [ -n "$LATEST" ]; then
    cat "$LATEST/judge_scorecard.csv"
else
    echo "No run dir"
fi

pkill -9 -f ruby 2>/dev/null || true
pkill -9 -f parameter_bridge 2>/dev/null || true
pkill -9 -f state_machine 2>/dev/null || true
pkill -9 -f obstacle_monitor 2>/dev/null || true
pkill -9 -f tile_tracker 2>/dev/null || true
pkill -9 -f robot_state_publisher 2>/dev/null || true
echo "INTEGRATION_TEST_DONE"
