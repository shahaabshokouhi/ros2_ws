# A full keyframe, sent so a peer can track it against its own map exactly as
# if it were a newly observed frame (2D-3D), then fuse it into that map.
#
# Keypoints are the KF's UNDISTORTED keypoints (mvKeysUn) -- all of them, not
# just the ones already carrying a map point -- because unmatched keypoints
# with valid depth become NEW map points in the receiving agent's map.
# Interpret them with the source agent's CameraCalib (requested separately).

string source_agent
string requester_agent
uint64 keyframe_id

uint32 num_keypoints        # N; all arrays below are sized from this
float32[] kp_u              # N, undistorted pixel u
float32[] kp_v              # N, undistorted pixel v
int32[]   kp_octave         # N, ORB pyramid level (drives search radius + sigma)
float32[] kp_depth          # N, metric depth; <= 0 means no valid depth
uint8[]   descriptors       # N*32, row-major ORB descriptors

builtin_interfaces/Time stamp
