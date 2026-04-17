# Implementation Plan — Artpark Mobility Hackathon R3

**Target:** ROS 2 Jazzy on Ubuntu 24.04 with Gazebo Harmonic. Base world `grid_world/` is byte-identical from the judge's zip and MUST NOT be modified.

## Package layout
```
gaws_ws/src/
├── grid_world/               # UNTOUCHED (judge's base)
├── artpark_robot/            # URDF + spawn + robot-specific launch
├── artpark_perception/       # AprilTag handler, floor logo detector, obstacle monitor
├── artpark_decision/         # State machine + edge sampler + tile tracker
├── artpark_logger/           # CSV + JSONL + image writer
└── artpark_bringup/          # master launch (Gazebo + robot + all nodes)
```

## Phases
| # | Deliverable |
|---|---|
| P0 | Workspace skeleton + ROS2 package manifests + build passes empty |
| P1 | Robot URDF (diff-drive, front RGB tilted 10°, depth, 2D LiDAR, IMU) spawns into `grid_world_FINAL.sdf` |
| P2 | AprilTag handler (apriltag_ros wrapper) + 3-frame vote + pose-in-camera publisher |
| P3 | Floor logo detector (HSV per edge strip, publishes edge green/orange counts) |
| P4 | Obstacle monitor (LiDAR min-distance per octant → `/obstacle_near`) |
| P5 | Tile tracker (odom + LiDAR → current tile index + entry edge) |
| P6 | State machine (INIT → EXPLORE → ON_TAG → ACT → GREEN_FOLLOW → ORANGE_FOLLOW → STOP) |
| P7 | Logger (judge_scorecard.csv + thought_log.jsonl + images/) |
| P8 | Bringup launch + first end-to-end sim run |
| P9 | Tuning (HSV thresholds, edge strip size, approach speed) |
| P10 | Recording + final deliverable |

## Hard rules from the judge
- No pre-built map, no SLAM.
- Only hardcoded AprilTag knowledge allowed: `label → action` (Tag 1=LEFT, Tag 2=RIGHT, Tag 3=GREEN, Tag 4=U-TURN, Tag 5=ORANGE).
- Tag `id → label` mapping is derived by ONE teleop verification run, then locked into `config/tag_label_map.yaml`.
- Tag world poses must NOT be pre-loaded. A landmark map is built online and refined each visit.
- Do not log color markers outside the designated green/orange zones.
- Do not retrace except Tag 1 → Tag 2.
- Image filenames encode ISO timestamp + sequence.

## Scoring target (150)
Full-compliance run:
- 5 tag logs × 10 = 50
- 5 decisions × 5 = 25
- Green + Orange = 25
- ~15 tile logs × 1 = 15
- Time bonus ~10
- ~8 color-marker tiles × 2.5 = 20
- Penalties target = 0
- **Total ~145**

## Open items (to resolve at P0 or before)
- [ ] Robot spawn pose yaw (face into maze from green tile)
- [ ] Tag 1 = first-seen-tag rule (save 4 or 5 PNGs?) — awaiting user confirmation
- [ ] HSV thresholds for green and orange on physical arena (calibrate at 8pm arena visit)
