# Broadcast by the OWNER of keyframes and landmarks that peers hold copies of
# ("new" method). Every keyframe and landmark has exactly one owner -- the
# agent that created it -- and only the owner optimizes it. Peers hold copies
# that are fixed in their own bundle adjustment and kept current by this.
#
# Event-triggered: only what moved since the last broadcast, plus a periodic
# full refresh of the keyframe poses.

string source_agent

# Keyframe poses (Tcw, world -> camera) in the OWNER's frame. A peer places
# its copy through its own estimate of how the two maps align.
uint64[]  keyframe_id
float32[] keyframe_pose      # 7 per keyframe: tx ty tz qx qy qz qw

# Landmarks whose position RELATIVE TO THEIR ANCHOR KEYFRAME changed. A pose
# correction moves anchor and landmark together and needs nothing here; only
# genuine refinement of the landmark itself is sent.
uint64[]  landmark_id
uint64[]  landmark_anchor_kf # the owner's keyframe the position is relative to
float32[] landmark_xc        # 3 per landmark: x y z in that keyframe's camera frame

# Landmarks the owner deleted or merged away. A peer adopts its copy if it has
# evidence of its own for it (>= 2 of its own keyframes), otherwise deletes it.
uint64[]  removed_landmark_id

builtin_interfaces/Time stamp
