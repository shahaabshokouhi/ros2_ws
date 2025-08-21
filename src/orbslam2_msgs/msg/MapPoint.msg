# Unique ID from the sender’s map
uint64 id

string agent_name
# World position in the sender's frame
geometry_msgs/Point position

# Viewing normal and distance invariance range
geometry_msgs/Vector3 normal
float32 min_distance
float32 max_distance

# ORB descriptor (256 bits = 32 bytes)
uint8[32] descriptor

# Status and timing
bool is_bad
builtin_interfaces/Time stamp

# (Optional) Reference keyframe id if you have it
uint64[] keyframe_ids