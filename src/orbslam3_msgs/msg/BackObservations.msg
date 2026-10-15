# "I saw your landmark from my keyframe, at this keypoint."
#
# Sent by an agent that holds a copy of another agent's landmark to that
# landmark's OWNER. The copy holder never optimizes the landmark itself; the
# owner adds these sightings to its own bundle adjustment (with the observer's
# keyframe held fixed), so the one agent allowed to move the landmark uses
# everyone's evidence. The owner can use a sighting once it holds a copy of the
# observer's keyframe; keypoint indices match because keyframes are shared with
# all their keypoints in their original order.

string observer_agent
string owner_agent

uint64[] owner_landmark_id    # the owner's MapPoint id
uint64[] observer_keyframe_id # the observer's own keyframe id
uint32[] keypoint_index       # keypoint in that keyframe

builtin_interfaces/Time stamp
