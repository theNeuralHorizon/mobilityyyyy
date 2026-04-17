# Structured thought/trace entry published by any node that wants to land in
# thought_log.jsonl. The logger subscribes to /thought and serialises each to
# one line of JSONL.
builtin_interfaces/Time stamp
int32 seq
string phase                  # INIT | EXPLORE | APPROACH_TAG | GREEN_FOLLOW | ORANGE_FOLLOW | SEEK_TAG_5 | REACH_STOP | ...
string hypothesis             # short human-readable summary of what the node concluded
string action_chosen          # cmd_vel class label, tag log, mode switch, etc.
string rule_applied           # which rule fired (e.g. GREEN_FOLLOW.single_edge_exit)
string alt_considered         # pipe-separated alternatives that lost
string extras_json            # JSON blob with sensor snapshot / floats; opaque to ROS graph
float32 confidence
