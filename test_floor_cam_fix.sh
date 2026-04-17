#!/bin/bash
source /opt/ros/jazzy/setup.bash
source /root/gaws_ws/install/setup.bash
export MESA_GL_VERSION_OVERRIDE=4.1

# Clean
pkill -9 -f ruby 2>/dev/null || true
pkill -9 -f gz 2>/dev/null || true
pkill -9 -f parameter_bridge 2>/dev/null || true
sleep 2

# Launch
ros2 launch artpark_bringup full_run.launch.py debug_hold:=true > /tmp/launch.log 2>&1 &
LAUNCH_PID=$!
sleep 22

# Check floor cam
timeout 8 python3 -c "
import rclpy, time, numpy as np
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image

rclpy.init()
node = rclpy.create_node('checker')
done = [False]

def cb(msg):
    if done[0]: return
    data = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
    unique = len(np.unique(data.reshape(-1,3), axis=0))
    print(f'FLOOR: {msg.width}x{msg.height} mean_rgb=({data[:,:,0].mean():.1f},{data[:,:,1].mean():.1f},{data[:,:,2].mean():.1f}) min={data.min()} max={data.max()} unique_colors={unique}')
    done[0] = True

qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
node.create_subscription(Image, '/floor_cam/image_raw', cb, qos)
t=time.time()
while time.time()-t<6 and not done[0]:
    rclpy.spin_once(node, timeout_sec=0.5)
if not done[0]: print('FLOOR: NO IMAGES')
rclpy.shutdown()
" 2>&1

# Cleanup
kill $LAUNCH_PID 2>/dev/null
pkill -9 -f ruby 2>/dev/null || true
pkill -9 -f gz 2>/dev/null || true
pkill -9 -f parameter_bridge 2>/dev/null || true
sleep 1
echo "TEST_DONE"
