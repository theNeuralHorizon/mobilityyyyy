#!/bin/bash
source /opt/ros/jazzy/setup.bash
source ~/gaws_ws/install/setup.bash

# Kill old processes
pkill -9 -f ruby 2>/dev/null || true
pkill -9 -f parameter_bridge 2>/dev/null || true
pkill -9 -f state_machine 2>/dev/null || true
pkill -9 -f obstacle_monitor 2>/dev/null || true
pkill -9 -f floor_logo 2>/dev/null || true
sleep 2

echo "=== LAUNCHING ==="
ros2 launch artpark_bringup full_run.launch.py debug_hold:=true 2>/dev/null &
LAUNCH_PID=$!

sleep 25

echo "=== FLOOR CAM HZ ==="
timeout 5 ros2 topic hz /floor_cam/image_raw 2>&1 | head -3

echo "=== FRONT CAM HZ ==="
timeout 5 ros2 topic hz /front_cam/image_raw 2>&1 | head -3

echo "=== SAVING FRAMES ==="
# Save one frame from floor cam using a small python script
python3 -c "
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
import cv2
import numpy as np
from cv_bridge import CvBridge
import sys

rclpy.init()
node = rclpy.create_node('frame_saver')
bridge = CvBridge()
saved = {'floor': False, 'front': False}

def save_floor(msg):
    if saved['floor']:
        return
    img = bridge.imgmsg_to_cv2(msg, 'bgr8')
    cv2.imwrite('/tmp/floor_cam_debug.png', img)
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    # Check green
    mask_g = cv2.inRange(hsv, np.array([35,80,60]), np.array([85,255,255]))
    # Check orange
    mask_o = cv2.inRange(hsv, np.array([5,130,120]), np.array([22,255,255]))
    print(f'FLOOR: shape={img.shape} green_px={cv2.countNonZero(mask_g)} orange_px={cv2.countNonZero(mask_o)}')
    print(f'FLOOR: mean_bgr={img.mean(axis=(0,1))} min={img.min()} max={img.max()}')
    print(f'FLOOR: hsv_mean={hsv.mean(axis=(0,1))}')
    cv2.imwrite('/tmp/floor_cam_debug.png', img)
    cv2.imwrite('/tmp/floor_cam_hsv_g.png', mask_g)
    cv2.imwrite('/tmp/floor_cam_hsv_o.png', mask_o)
    saved['floor'] = True

def save_front(msg):
    if saved['front']:
        return
    img = bridge.imgmsg_to_cv2(msg, 'bgr8')
    cv2.imwrite('/tmp/front_cam_debug.png', img)
    print(f'FRONT: shape={img.shape} mean_bgr={img.mean(axis=(0,1))} min={img.min()} max={img.max()}')
    saved['front'] = True

qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
node.create_subscription(Image, '/floor_cam/image_raw', save_floor, qos)
node.create_subscription(Image, '/front_cam/image_raw', save_front, qos)

import time
start = time.time()
while time.time() - start < 10:
    rclpy.spin_once(node, timeout_sec=0.5)
    if saved['floor'] and saved['front']:
        break

if not saved['floor']:
    print('FLOOR: NO IMAGES RECEIVED')
if not saved['front']:
    print('FRONT: NO IMAGES RECEIVED')
node.destroy_node()
rclpy.shutdown()
" 2>&1

echo "=== SAVED FILES ==="
ls -la /tmp/floor_cam_debug.png /tmp/front_cam_debug.png /tmp/floor_cam_hsv_g.png /tmp/floor_cam_hsv_o.png 2>/dev/null || echo "No files saved"

echo "=== CLEANUP ==="
kill $LAUNCH_PID 2>/dev/null
pkill -9 -f ruby 2>/dev/null || true
pkill -9 -f parameter_bridge 2>/dev/null || true
sleep 1
echo "DEBUG_DONE"
