#!/bin/bash
source /opt/ros/jazzy/setup.bash
source /root/gaws_ws/install/setup.bash
export MESA_GL_VERSION_OVERRIDE=4.1

pkill -9 -f ruby 2>/dev/null || true
pkill -9 -f gz 2>/dev/null || true
pkill -9 -f parameter_bridge 2>/dev/null || true
sleep 2

ros2 launch artpark_bringup full_run.launch.py debug_hold:=true > /tmp/launch.log 2>&1 &
LAUNCH_PID=$!
sleep 22

timeout 10 python3 -c "
import rclpy, time, numpy as np, cv2
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

rclpy.init()
node = rclpy.create_node('color_checker')
bridge = CvBridge()
done = [False]

def cb(msg):
    if done[0]: return
    img = bridge.imgmsg_to_cv2(msg, 'bgr8')
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    mask_g = cv2.inRange(hsv, np.array([35,80,60]), np.array([85,255,255]))
    mask_o = cv2.inRange(hsv, np.array([5,130,120]), np.array([22,255,255]))
    mask_r1 = cv2.inRange(hsv, np.array([0,100,100]), np.array([10,255,255]))
    mask_r2 = cv2.inRange(hsv, np.array([170,100,100]), np.array([180,255,255]))
    mask_r = mask_r1 | mask_r2
    total = img.shape[0] * img.shape[1]
    g_px = cv2.countNonZero(mask_g)
    o_px = cv2.countNonZero(mask_o)
    r_px = cv2.countNonZero(mask_r)
    print(f'FLOOR: shape={img.shape} unique={len(np.unique(img.reshape(-1,3), axis=0))}')
    print(f'FLOOR: mean_bgr=({img[:,:,0].mean():.1f},{img[:,:,1].mean():.1f},{img[:,:,2].mean():.1f})')
    print(f'FLOOR: green_px={g_px} ({100*g_px/total:.1f}%) orange_px={o_px} ({100*o_px/total:.1f}%) red_px={r_px} ({100*r_px/total:.1f}%)')
    print(f'FLOOR: hsv_mean=({hsv[:,:,0].mean():.1f},{hsv[:,:,1].mean():.1f},{hsv[:,:,2].mean():.1f})')
    cv2.imwrite('/tmp/floor_cam_prod.png', img)
    cv2.imwrite('/tmp/floor_cam_hsv_g.png', mask_g)
    cv2.imwrite('/tmp/floor_cam_hsv_o.png', mask_o)
    done[0] = True

qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
node.create_subscription(Image, '/floor_cam/image_raw', cb, qos)
t=time.time()
while time.time()-t<8 and not done[0]:
    rclpy.spin_once(node, timeout_sec=0.5)
if not done[0]: print('FLOOR: NO IMAGES')
rclpy.shutdown()
" 2>&1

kill $LAUNCH_PID 2>/dev/null
pkill -9 -f ruby 2>/dev/null || true
pkill -9 -f gz 2>/dev/null || true
pkill -9 -f parameter_bridge 2>/dev/null || true
sleep 1
echo "TEST_DONE"
