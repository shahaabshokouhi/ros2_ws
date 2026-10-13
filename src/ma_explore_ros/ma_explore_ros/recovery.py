# Copied from ma_explore (3605e76) by scripts/sync_robot_package.sh: edit it there, not here.
"""Back-tracking after a SLAM tracking loss (pure geometry, no ROS).

When tracking is lost the car drives back, slowly and in reverse, along the
path it drove just before the loss, steering on wheel odometry. The camera
then looks again at what it saw before the loss (the forward camera faced the
same way while driving that path), and ORB-SLAM3 can relocalize in its map.

Frames: the path is in the SLAM map frame; the car is followed in the wheel
odometry frame. At the loss, the last SLAM pose P and the wheel pose W at the
same moment give T = W * P^-1 (SE(2)), which carries the path into the wheel
frame (wheel odometry drifts slowly, so over a metre this is enough).

Control: reverse pure pursuit. The look-ahead point on the path, in the car's
frame (x forward, y left), is (x, y) with x < 0; the curvature to reach it is
k = 2 y / (x^2 + y^2) and the yaw rate w = v k (v < 0).
"""
import math

import numpy as np


def se2(x, y, th):
    c, s = math.cos(th), math.sin(th)
    return np.array([[c, -s, x], [s, c, y], [0.0, 0.0, 1.0]])


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


class BackTrack:
    """Follow `path_map` (list of (x, y, yaw), oldest first, ending at the pose
    where tracking was lost) backwards, from the newest point to the oldest."""

    def __init__(self, path_map, slam_at_loss, wheel_at_loss, speed=0.1, lookahead=0.25, min_radius=0.3):
        T = se2(*wheel_at_loss) @ np.linalg.inv(se2(*slam_at_loss))
        P = np.array([[p[0], p[1], 1.0] for p in path_map]) @ T.T
        self.path = P[::-1, :2]                       # newest (the loss) first
        self.length = float(np.linalg.norm(np.diff(self.path, axis=0), axis=1).sum()) if len(self.path) > 1 else 0.0
        self.speed, self.lookahead, self.min_radius = speed, lookahead, min_radius
        self.i = 0                                    # progress along the path (never goes back)
        self.start = None
        self.driven = 0.0

    def sync(self, wheel_pose):
        """Start the progress at the path point nearest the car (a later attempt starts part-way back)."""
        if len(self.path):
            self.i = int(np.argmin(np.linalg.norm(self.path - np.array(wheel_pose[:2]), axis=1)))

    def command(self, wheel_pose):
        """(v, w, done) for the car at wheel_pose = (x, y, yaw) in the wheel frame."""
        x, y, yaw = wheel_pose
        if self.start is not None:
            self.driven += math.hypot(x - self.start[0], y - self.start[1])
        self.start = (x, y)
        if len(self.path) < 2:
            return 0.0, 0.0, True
        d = np.linalg.norm(self.path - np.array([x, y]), axis=1)
        # progress: the nearest path point from here on (a short window keeps it from jumping ahead)
        win = slice(self.i, min(len(self.path), self.i + 12))
        self.i = self.i + int(np.argmin(d[win]))
        if self.i >= len(self.path) - 1 and d[-1] < 0.08:
            return 0.0, 0.0, True
        ahead = np.nonzero(d[self.i:] >= self.lookahead)[0]
        j = self.i + int(ahead[0]) if len(ahead) else len(self.path) - 1
        if j == len(self.path) - 1 and d[-1] < 0.08:
            return 0.0, 0.0, True
        gx, gy = self.path[j]
        c, s = math.cos(yaw), math.sin(yaw)
        xr, yr = c * (gx - x) + s * (gy - y), -s * (gx - x) + c * (gy - y)     # target in the car frame
        if xr > 0.05:                                 # the target is in front: the path is behind us already
            return 0.0, 0.0, True
        d2 = max(xr * xr + yr * yr, 1e-6)
        k = max(-1.0 / self.min_radius, min(1.0 / self.min_radius, 2.0 * yr / d2))
        v = -self.speed
        return v, v * k, False
