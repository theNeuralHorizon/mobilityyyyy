# Published by the floor logo detector at tile-rate. Raw green/orange pixel
# counts on each of the 4 edge strips of the current tile, plus the chosen
# exit direction after the state machine applies its rule.
builtin_interfaces/Time stamp
int32 green_n
int32 green_s
int32 green_e
int32 green_w
int32 orange_n
int32 orange_s
int32 orange_e
int32 orange_w
string entry_edge            # propagated back for logging context
string recommended_exit      # the algorithm's choice (may be vetoed by LiDAR)
float32 confidence           # 0..1; ratio of top edge to second edge
int32 red_total
