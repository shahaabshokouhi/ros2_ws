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

# The sender's OWN pose for this keyframe (Tcw: world -> camera), in the
# sender's map frame. It is never used to place the keyframe -- the receiver
# works that out itself with PnP -- but it is needed to check whether several
# inter-agent matches agree with each other (PCM). That check walks a loop
# through both robots' own trajectories, and one leg of it is the sender's
# relative pose between two of its own keyframes, which only the sender knows.
#
# Note it is a snapshot: the sender's poses all shift when it closes a loop or
# merges maps. The DIFFERENCE between two of its keyframes barely changes under
# those corrections, which is why PCM uses differences rather than raw poses.
bool has_pose
geometry_msgs/Transform pose

builtin_interfaces/Time stamp
