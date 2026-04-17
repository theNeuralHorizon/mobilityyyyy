#!/bin/bash
source /opt/ros/jazzy/setup.bash
source /root/gaws_ws/install/setup.bash
export MESA_GL_VERSION_OVERRIDE=4.1

pkill -9 -f ruby 2>/dev/null || true
pkill -9 -f gz 2>/dev/null || true
pkill -9 -f parameter_bridge 2>/dev/null || true
pkill -9 -f state_machine 2>/dev/null || true
pkill -9 -f floor_logo 2>/dev/null || true
pkill -9 -f obstacle_monitor 2>/dev/null || true
sleep 2

# Full launch (not debug_hold)
ros2 launch artpark_bringup full_run.launch.py > /tmp/launch.log 2>&1 &
LAUNCH_PID=$!

# Wait for sim + nodes to start
sleep 25

echo "=== TOPIC CHECK ==="
timeout 3 ros2 topic hz /floor_cam/image_raw 2>&1 | head -2
timeout 3 ros2 topic hz /front_cam/image_raw 2>&1 | head -2
timeout 3 ros2 topic hz /scan 2>&1 | head -2

echo "=== RUN FOR 60s ==="
sleep 60

echo "=== THOUGHT LOG ==="
timeout 3 ros2 topic echo /thought --once 2>&1 | head -20

echo "=== SCORECARD ==="
timeout 5 python3 -c "
import rclpy, time
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

rclpy.init()
node = rclpy.create_node('scorecard')
events = []

def cb(msg):
    events.append(msg.data)

qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE)
node.create_subscription(String, '/tag_event', cb, qos)

# Also check /thought for phase info
thoughts = []
def tcb(msg):
    thoughts.append(str(msg))

from artpark_msgs.msg import Thought
node.create_subscription(Thought, '/thought', tcb, QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT))

t=time.time()
while time.time()-t<3:
    rclpy.spin_once(node, timeout_sec=0.3)

print(f'TAG_EVENTS: {len(events)}')
for e in events:
    print(f'  {e}')
if thoughts:
    print(f'LAST_THOUGHT: {thoughts[-1][:200]}')
rclpy.shutdown()
" 2>&1

echo "=== FLOOR CAM COLOR CHECK ==="
timeout 8 python3 -c "
import rclpy, time, numpy as np, cv2
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

rclpy.init()
node = rclpy.create_node('floor_check')
bridge = CvBridge()
done = [False]

def cb(msg):
    if done[0]: return
    img = bridge.imgmsg_to_cv2(msg, 'bgr8')
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    mask_g = cv2.inRange(hsv, np.array([35,80,60]), np.array([85,255,255]))
    mask_o = cv2.inRange(hsv, np.array([5,130,120]), np.array([22,255,255]))
    total = img.shape[0] * img.shape[1]
    g_px = cv2.countNonZero(mask_g)
    o_px = cv2.countNonZero(mask_o)
    unique = len(np.unique(img.reshape(-1,3), axis=0))
    print(f'FLOOR_CAM: unique={unique} green={100*g_px/total:.1f}% orange={100*o_px/total:.1f}%')
    done[0] = True

qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
node.create_subscription(Image, '/floor_cam/image_raw', cb, qos)
t=time.time()
while time.time()-t<6 and not done[0]:
    rclpy.spin_once(node, timeout_sec=0.5)
if not done[0]: print('FLOOR_CAM: NO IMAGES')
rclpy.shutdown()
" 2>&1

echo "=== ODOM CHECK ==="
timeout 3 python3 -c "
import rclpy, time
from rclpy.qos import QoSProfile, ReliabilityPolicy
from nav_msgs.msg import Odometry

rclpy.init()
node = rclpy.create_node('odom_check')
done = [False]

def cb(msg):
    if done[0]: return
    p = msg.pose.pose.position
    print(f'ODOM: x={p.x:.2f} y={p.y:.2f} z={p.z:.2f}')
    done[0] = True

qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
node.create_subscription(Odometry, '/odom', cb, qos)
t=time.time()
while time.time()-t<2 and not done[0]:
    rclpy.spin_once(node, timeout_sec=0.3)
rclpy.shutdown()
" 2>&1

kill $LAUNCH_PID 2>/dev/null
pkill -9 -f ruby 2>/dev/null || true
pkill -9 -f gz 2>/dev/null || true
pkill -9 -f parameter_bridge 2>/dev/null || true
pkill -9 -f state_machine 2>/dev/null || true
sleep 1
echo "INTEGRATION_DONE"
