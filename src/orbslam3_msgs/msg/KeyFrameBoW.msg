uint64 keyframe_id
string agent_name
uint32[] word_ids
float64[] word_values
builtin_interfaces/Time stamp

# Covisibility reference score: the weakest BoW score between this keyframe and
# the keyframes it shares the most map points with. Those provably see the same
# place, so this is the sender's own answer to "in my scene, with my camera, how
# similar do two views of the same place look?" -- the scale an absolute BoW
# threshold lacks. 0 means "not provided" (older agent); the receiver then falls
# back to its own host-side reference only.
float32 bow_min_score

# ---------------------------------------------------------------------------
# Where this keyframe sits in the SENDER's own map. Once two maps are aligned,
# the receiver uses this to copy the rest of the sender's map in a sensible
# order before it holds any of it: outward from where the maps overlap (a
# keyframe linked to one already copied), nearest to the receiver first, and
# skipping near-duplicates of views it already has. About 76 bytes.
bool       in_main_map    # the pose is in the sender's main-map frame
bool       has_pose
float32[7] pose           # Tcw, sender's frame: tx ty tz qx qy qz qw
bool       has_parent
uint64     parent_id      # spanning-tree parent (sender's id)
uint64[]   neighbour_ids  # strongest covisible keyframes, best first (<= 5)
