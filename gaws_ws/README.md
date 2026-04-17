# Artpark Hackathon R3 — Workspace

## What this is
End-to-end navigation stack for the MIT/Artpark mobility hackathon Round 3.
The robot must explore a walled arena, detect 5 AprilTags, follow green then
orange floor markers, and reach a STOP tile — all without any pre-built map.

Target stack: **ROS 2 Jazzy** on **Ubuntu 24.04**, **Gazebo Harmonic**.

## Packages
| Name | Role |
|---|---|
| `grid_world` | Judge's base world (UNTOUCHED) |
| `artpark_msgs` | Custom msgs: `TagEvent`, `TileEvent`, `EdgeSample`, `Thought` |
| `artpark_robot` | URDF + spawn launch (diff-drive, tilted RGB, floor cam, LiDAR, IMU) |
| `artpark_perception` | `apriltag_handler`, `floor_logo_detector`, `obstacle_monitor` |
| `artpark_decision` | `state_machine`, `tile_tracker`, edge-sampling logic |
| `artpark_logger` | Per-run `judge_scorecard.csv`, `thought_log.jsonl`, images/ |
| `artpark_bringup` | Master launch + all configs |

## Build
```bash
cd ~/gaws_ws
# AprilTag detection uses pupil_apriltags (cv2.aruco AprilTag support
# segfaults on Ubuntu 24.04's OpenCV 4.6).
pip install --break-system-packages pupil-apriltags

rosdep install --from-paths src --ignore-src -r -y --skip-keys "ament_python"
colcon build --symlink-install
source install/setup.bash
```

## Run (full run)
```bash
ros2 launch artpark_bringup full_run.launch.py spawn_yaw:=0.0
```
Outputs land in `~/artpark_runs/<YYYYmmddTHHMMSS>/`.

## P0 — One-time tag-id verification run
Before trusting the action mapping:
```bash
# Terminal 1
ros2 launch artpark_bringup teleop_verify.launch.py
# Terminal 2
ros2 topic echo /detections --field detections
# Terminal 3
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
Drive past all 5 tags. Note which `id` appears at which map-labelled position
(1..5 per the reference map). Edit
`src/artpark_bringup/config/tag_label_map.yaml` to set `tag_id_to_label`
accordingly, then rebuild and do full runs.

## Hardcoding policy (per Atharva 2026-04-17)
Only allowed AprilTag hardcode:
| Label | Action |
|---|---|
| 1 | LEFT |
| 2 | RIGHT |
| 3 | start GREEN_FOLLOW |
| 4 | U-TURN |
| 5 | start ORANGE_FOLLOW |

**Not allowed**: tag world poses, label-to-tile maps, SLAM outputs.

## Run artefacts
Each run creates `~/artpark_runs/<stamp>/`:
- `judge_scorecard.csv` — ONE row per scoreable event (tag log, tag decision,
  tile enter, color marker hit, stop reached). This is what the judges grade.
- `thought_log.jsonl` — every node's reasoning, one JSON per line.
- `raw_sensor.csv` — periodic edge-sample dump for post-hoc tuning.
- `images/<stamp>_<seq>_<event>_tag<label>.png` — PNG per tag commit.

## Files you SHOULDN'T edit
- Anything under `src/grid_world/` — byte-identical to the judge's zip.
