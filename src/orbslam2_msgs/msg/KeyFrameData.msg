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

# ---------------------------------------------------------------------------
# The sender's OWN landmarks for this keyframe.
#
# A keypoint whose 3D point already exists in the sender's map carries far
# more information than one raw depth reading: the sender triangulated it from
# several views and bundle-adjusted it, and it has already survived
# MapPointCulling probation. Passing that work along means the receiver does
# not have to re-derive the point from a single noisy depth pixel, and does not
# have to re-prove a landmark the sender already proved.
#
# This is a COMPACT list -- only keypoints that actually carry a map point --
# so it is short (a few hundred entries against ~1500 keypoints). Every array
# below has the same length M, indexed together, and mp_kp_index says which
# keypoint each entry belongs to.
uint32 num_landmarks         # M
uint32[] mp_kp_index         # M, index into the kp_* arrays above
uint64[] mp_id               # M, the SENDER's MapPoint id -- see below
uint16[] mp_num_obs          # M, how many of the sender's KFs observe it

# Position in THIS KEYFRAME'S CAMERA FRAME (metres), not in the sender's world
# frame. That is deliberate: a camera-frame position is immune to the sender's
# global drift. The sender's world frame may have wandered, but where a
# landmark sits relative to the keyframe observing it stays locally accurate,
# and the receiver already knows that keyframe's pose in its own frame from
# PnP. So the receiver just applies its own pose and the point lands correctly,
# with the sender's bundle-adjusted precision rather than its drift.
float32[] mp_cam_x           # M
float32[] mp_cam_y           # M
float32[] mp_cam_z           # M

# mp_id lets the receiver recognise the SAME landmark arriving in several
# different keyframes from the same sender. Without it, two fused keyframes
# that both see one wall corner create two separate points, each with a single
# observation. With it, the second keyframe adds an observation to the point
# the first one created, so fused keyframes share landmarks and form a
# connected graph instead of isolated islands.

builtin_interfaces/Time stamp
