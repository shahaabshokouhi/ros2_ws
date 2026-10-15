# One-time camera + ORB configuration for an agent, so peers can reconstruct
# that agent's keyframes as trackable frames with correct projection and scale.
# Broadcast (no requester field): a CALIBRATION request triggers it, and every
# peer caches it, so a late joiner only costs one extra broadcast.

uint8 PINHOLE=0
uint8 KANNALA_BRANDT8=1

string source_agent
uint8 model_type

# Projection parameters for the model above:
#   PINHOLE          -> [fx, fy, cx, cy]
#   KANNALA_BRANDT8  -> [fx, fy, cx, cy, k1, k2, k3, k4]
# Keyframe keypoints are transmitted already UNDISTORTED (mvKeysUn), so no
# separate distortion vector is needed: pinhole needs none, and the fisheye
# coefficients above are part of its projection model.
float32[] intrinsics

float32 bf              # stereo baseline * fx (RGB-D virtual baseline)
float32 th_depth        # close/far depth threshold, metric
int32 width
int32 height

# ORB pyramid, so scale-dependent matching and the pose-optimization
# information matrix are exact for this agent's keypoint octaves.
int32 n_scale_levels
float32 scale_factor

builtin_interfaces/Time stamp
