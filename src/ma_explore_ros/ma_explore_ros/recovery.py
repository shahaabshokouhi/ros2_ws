# Copied from ma_explore (15f02ee) by ma_explore.robot_package: edit it there, not here.
"""Back-tracking after a SLAM tracking loss, and the way out of a wedge (pure geometry, no ROS).

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

Wedge escape (WedgeEscape): a car whose outline, on its own occupancy grid,
lies closer to an obstacle than Nav2's planner allows ("start in lethal")
cannot be planned out of the spot, and Nav2's back-up refuses to move into the
inflation. Among short arcs the car can drive (forward and in reverse, straight
and at two curvatures each side), the escape picks the shortest one after which
the outline is `target` m from every occupied cell. Along the arc the outline
gets at most `slack` m closer to an obstacle than it is at the start (or than
`target`, if it is already that far; turning swings the short rear overhang
outwards), and it covers only cells known to be free or covered by the car
before (it drove over them); unknown cells are not entered. If no arc reaches
`target`, the arc that frees the most of the outline is taken (fewer of its
points within `target` of an obstacle, or `gain` m more clearance), and the
escape is planned again from where it ends: backing out of a pocket first,
then turning away from the wall beside it.
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

    def __init__(self, path_map, slam_at_loss, wheel_at_loss, speed=0.1, lookahead=0.25, min_radius=0.3,
                 forward=False):
        T = se2(*wheel_at_loss) @ np.linalg.inv(se2(*slam_at_loss))
        P = np.array([[p[0], p[1], 1.0] for p in path_map]) @ T.T
        self.path = P[::-1, :2]                       # newest (the loss) first
        self.length = float(np.linalg.norm(np.diff(self.path, axis=0), axis=1).sum()) if len(self.path) > 1 else 0.0
        self.speed, self.lookahead, self.min_radius = speed, lookahead, min_radius
        self.sign = 1.0 if forward else -1.0          # forward: retrace a path that was driven in reverse
        self.i = 0                                    # progress along the path (never goes back)
        self.start = None
        self.driven = 0.0

    def target_in_front(self, wheel_pose):
        """Is the first look-ahead point in front of the car (the path was driven in reverse)?"""
        x, y, yaw = wheel_pose
        d = np.linalg.norm(self.path - np.array([x, y]), axis=1)
        ahead = np.nonzero(d[self.i:] >= self.lookahead)[0]
        j = self.i + int(ahead[0]) if len(ahead) else len(self.path) - 1
        gx, gy = self.path[j]
        return math.cos(yaw) * (gx - x) + math.sin(yaw) * (gy - y) > 0.05

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
        if self.sign * xr < -0.05:                    # the target is on the wrong side: the path is done
            return 0.0, 0.0, True
        d2 = max(xr * xr + yr * yr, 1e-6)
        k = max(-1.0 / self.min_radius, min(1.0 / self.min_radius, 2.0 * yr / d2))
        v = self.sign * self.speed
        return v, v * k, False


# The car's outline around the rear axle (x forward, y left), as Nav2's footprint (nav2_jetracer.yaml):
# 0.24 x 0.17 m, from 0.03 m behind to 0.21 m ahead of the axle.
FOOTPRINT = ((0.21, 0.085), (0.21, -0.085), (-0.03, -0.085), (-0.03, 0.085))


def footprint_points(fp=FOOTPRINT, step=0.02):
    """Points covering the outline and the inside of the footprint, in the car frame, (n, 2)."""
    xs, ys = [p[0] for p in fp], [p[1] for p in fp]
    gx = np.linspace(min(xs), max(xs), max(2, int(round((max(xs) - min(xs)) / step)) + 1))
    gy = np.linspace(min(ys), max(ys), max(2, int(round((max(ys) - min(ys)) / step)) + 1))
    X, Y = np.meshgrid(gx, gy)
    return np.stack([X.ravel(), Y.ravel()], 1)


def arc_pose(pose, sign, k, s):
    """The rear-axle pose after driving s m (sign +1 forward, -1 reverse) at curvature k (1/m, left > 0)."""
    x, y, th = pose
    d = sign * s
    if abs(k) < 1e-9:
        return x + d * math.cos(th), y + d * math.sin(th), th
    th2 = th + d * k
    return x + (math.sin(th2) - math.sin(th)) / k, y - (math.cos(th2) - math.cos(th)) / k, th2


class WedgeEscape:
    """The escape for a car at `pose` (x, y, yaw of the rear axle) on an occupancy grid (data (h, w): -1
    unknown, 0..100; occupied >= 50), resolution and origin (m) in the pose's frame. `driven`: poses the
    car occupied before (its path), whose footprint cells count as passable even if unknown."""

    def __init__(self, data, resolution, origin_x, origin_y, pose, driven=(), target=0.10, gain=0.02, slack=0.01,
                 max_len=0.5, radius=0.25, step=0.02, window=0.9, fp=FOOTPRINT):
        self.res, self.ox, self.oy = resolution, origin_x, origin_y
        self.h, self.w = data.shape
        self.pose, self.target, self.gain, self.max_len, self.step = tuple(pose), target, gain, max_len, step
        self.slack = slack
        self.curvatures = (0.0, 0.5 / radius, -0.5 / radius, 1.0 / radius, -1.0 / radius)
        self.pts = footprint_points(fp)
        occ = data >= 50
        # occupied cell centres near the car (the clearance is measured to these, minus half a cell)
        iy, ix = np.nonzero(occ)
        cx, cy = origin_x + (ix + 0.5) * resolution, origin_y + (iy + 0.5) * resolution
        near = (np.abs(cx - pose[0]) < window) & (np.abs(cy - pose[1]) < window)
        self.obst = np.stack([cx[near], cy[near]], 1)
        passable = (data >= 0) & ~occ
        for p in [pose, *driven]:                      # cells the car has stood on
            for i, j in self.cells(self.place(p)):
                passable[i, j] = passable[i, j] or not occ[i, j]
        self.passable = passable

    def place(self, pose):
        """The footprint points at pose, in the grid frame, (n, 2)."""
        x, y, th = pose
        c, s = math.cos(th), math.sin(th)
        P = self.pts
        return np.stack([x + c * P[:, 0] - s * P[:, 1], y + s * P[:, 0] + c * P[:, 1]], 1)

    def cells(self, xy):
        j = np.floor((xy[:, 0] - self.ox) / self.res).astype(int)
        i = np.floor((xy[:, 1] - self.oy) / self.res).astype(int)
        ok = (i >= 0) & (i < self.h) & (j >= 0) & (j < self.w)
        return list(zip(i[ok], j[ok]))

    def clearance(self, pose):
        """Distance (m) from the footprint at pose to the nearest occupied cell (its edge; < 0: overlapping)."""
        return self.measure(pose)[0]

    def measure(self, pose):
        """(clearance, number of footprint points closer than target to an occupied cell) at pose."""
        if not len(self.obst):
            return math.inf, 0
        xy = self.place(pose)
        d = np.sqrt(((xy[:, None, :] - self.obst[None, :, :]) ** 2).sum(-1)).min(1) - 0.5 * self.res
        return float(d.min()), int((d < self.target).sum())

    def admissible(self, pose):
        xy = self.place(pose)
        j = np.floor((xy[:, 0] - self.ox) / self.res).astype(int)
        i = np.floor((xy[:, 1] - self.oy) / self.res).astype(int)
        if ((i < 0) | (i >= self.h) | (j < 0) | (j >= self.w)).any():
            return False
        return bool(self.passable[i, j].all())

    def wedged(self):
        return self.clearance(self.pose) < self.target

    def plan(self):
        """(sign, curvature, length, clearance at the start, clearance at the end) of the escape arc, or None."""
        c0, n0 = self.measure(self.pose)
        floor = min(c0, self.target) - self.slack - 1e-6
        best, partial = None, None
        for sign in (-1, 1):                           # reverse first: the camera keeps the view it tracks
            for k in self.curvatures:
                s, reached, end = 0.0, None, (c0, n0)
                while s + self.step <= self.max_len + 1e-9:
                    q = arc_pose(self.pose, sign, k, s + self.step)
                    c, n = self.measure(q)
                    if c < floor or not self.admissible(q):
                        break
                    s += self.step
                    end = (c, n)
                    if c >= self.target:
                        reached = s
                        break
                if reached is not None:
                    cost = reached + (0.05 if sign > 0 else 0.0) + 0.005 * abs(k)
                    if best is None or cost < best[0]:
                        best = (cost, (sign, k, reached, c0, end[0]))
                elif s > 0:
                    freed = n0 - end[1]
                    if freed >= max(3, 0.2 * n0) or end[0] - c0 >= self.gain:
                        key = (freed, end[0], -s)
                        if partial is None or key > partial[0]:
                            partial = (key, (sign, k, s, c0, end[0]))
        if best is not None:
            return best[1]
        return partial[1] if partial is not None else None
