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
