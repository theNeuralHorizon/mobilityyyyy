# Emitted by the AprilTag handler once the 3-frame vote commits a detection.
builtin_interfaces/Time stamp
int32 tag_id                  # raw decoded id from the AprilTag family
int32 logical_label           # 1..5 per the hardcoded label map; 0 = not yet mapped
string decision               # "LEFT" | "RIGHT" | "GREEN" | "U_TURN" | "ORANGE" | "UNKNOWN"
geometry_msgs/Pose tag_in_camera    # tag pose in camera frame
float64 distance              # forward distance to tag (meters)
float64 bearing               # bearing in camera frame (radians, + is left)
int32 vote_frames             # how many frames agreed before commit
bool first_sighting           # true if this is the first time this tag_id is seen this run
