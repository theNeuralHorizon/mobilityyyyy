# P0 — Tag-ID to Logical-Label verification

Run this ONCE on the sim (or the physical arena if sim is down). Lock the
result into `src/artpark_bringup/config/tag_label_map.yaml`.

## Steps
1. **Launch sim + robot + apriltag_ros** in one terminal:
   ```bash
   ros2 launch artpark_bringup teleop_verify.launch.py spawn_yaw:=0.0
   ```
   Confirm in the Gazebo GUI:
   - the solid-green tile is where the judge expects START;
   - the solid-red tile is where the judge expects STOP;
   - take a screenshot and compare with the reference map image.
   If orientation is rotated 180°, re-launch with `spawn_yaw:=3.14159`.

2. **Echo detections** in terminal 2:
   ```bash
   ros2 topic echo /detections
   ```

3. **Drive** in terminal 3:
   ```bash
   ros2 run teleop_twist_keyboard teleop_twist_keyboard
   ```

4. Pilot the robot past each of the 5 AprilTags. For each sighting, note:
   - the raw `id` printed on `/detections`;
   - the tile/label as marked on the reference map (1..5).

5. Build the table:

   | reference-map label | action (fixed) | observed SDF tag_id |
   |---|---|---|
   | 1 | LEFT          | ??? |
   | 2 | RIGHT         | ??? |
   | 3 | GREEN_FOLLOW  | ??? |
   | 4 | U_TURN        | ??? |
   | 5 | ORANGE_FOLLOW | ??? |

6. Edit `src/artpark_bringup/config/tag_label_map.yaml` and replace the
   `tag_label_map` entries. Also set `spawn_yaw` in `full_run.launch.py`
   to whatever yaw faced "into the maze" from START.

7. Rebuild:
   ```bash
   colcon build --packages-select artpark_bringup
   ```

8. Run full stack: `ros2 launch artpark_bringup full_run.launch.py`.

## Why this is the only allowed hardcode
Atharva (2026-04-17 19:29): "This is the only content you are allowed to
hardcode with apriltag" — referring to the label→action table. Tag world
poses, map-to-tile assignments, and SLAM outputs are all disallowed. The
`tag_id → logical_label` lookup is a runtime-observed constant; we lock
it once after this P0 run and never recompute it.
