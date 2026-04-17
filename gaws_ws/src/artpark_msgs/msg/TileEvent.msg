# Emitted when the tile tracker concludes the robot has entered a new tile.
builtin_interfaces/Time stamp
int32 tile_row                # 0..4
int32 tile_col                # 0..3
string entry_edge             # "N" | "S" | "E" | "W"
string dominant_color         # "green" | "orange" | "none"
bool in_green_zone            # true if this tile is a valid green log target
bool in_orange_zone           # true if this tile is a valid orange log target
