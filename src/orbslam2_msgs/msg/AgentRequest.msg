# Unified inter-agent request. `request_type` selects what is being asked for,
# so all agent-to-agent pulls share one topic and one message type.
# Distributed SLAM: every agent both sends and answers these.

uint8 CALIBRATION=0     # ask for the target's camera + ORB config (one-time)
uint8 KEYFRAME=1        # ask for one full keyframe (features + depth)

uint8 request_type
string requester_agent  # agent asking
string target_agent     # agent that owns the data
uint64 keyframe_id      # only meaningful when request_type == KEYFRAME
builtin_interfaces/Time stamp
