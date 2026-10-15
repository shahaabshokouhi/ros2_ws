# Copied from ma_explore (fbdfb24) by scripts/sync_robot_package.sh: edit it there, not here.
"""Decisions from live maps: the bridge between a running robot and the policy.

No ROS here, so it can be tested on its own. A robot's planner (ros_planner.py)
collects what the robot knows -- its own occupancy grid, its teammates' grids
and poses once their maps are merged with its own -- puts it on the training
grid (96 x 96 cells of 0.1 m around what is known; with a policy trained on
ego observations, a grid as large as what is known, so any world fits) and asks
the policy for the next goal.

The observation is built by the training environment itself (ExploreEnv.observe
on a one-environment copy whose state is set from the live maps), so a robot
sees exactly what it saw in training: same planes, same candidate goals, same
features. Robot 0 of that copy is the deciding robot; the others are its
teammates, merged or not.
"""
import math
import os
import time
from dataclasses import dataclass, field

import numpy as np
import torch

from . import baselines, geometry as G
from .env import EnvConfig, ExploreEnv


# ── occupancy grids on the training grid ───────────────────────────────────
@dataclass
class Grid:
    """A nav_msgs/OccupancyGrid as arrays: data (h, w) int8 (-1 unknown,
    0..100), cell size and origin (m) in the grid's frame."""
    data: np.ndarray
    resolution: float
    origin_x: float
    origin_y: float

    def cells(self, occ_threshold=50):
        """Centres (x, y) of the free and the occupied cells, in the grid's frame."""
        d = self.data
        out = []
        for mask in ((d >= 0) & (d < occ_threshold), d >= occ_threshold):
            iy, ix = np.nonzero(mask)
            out.append(np.stack([self.origin_x + (ix + 0.5) * self.resolution,
                                 self.origin_y + (iy + 0.5) * self.resolution], 1))
        return out


def se2(x, y, yaw):
    """3 x 3 homogeneous transform."""
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s, x], [s, c, y], [0, 0, 1.0]])


def apply(T, pts):
    return pts @ T[:2, :2].T + T[:2, 2]


@dataclass
class Window:
    """The H x W training grid placed in the deciding robot's map frame: cell
    (row, col) covers x0 + col*cell .. +cell, y0 + row*cell .. +cell."""
    x0: float
    y0: float
    H: int = 96
    W: int = 96
    cell: float = 0.1

    def to_cell(self, pts):
        """(n, 2) metres -> (n, 2) long (row, col); and which are inside."""
        c = np.floor((pts[:, 0] - self.x0) / self.cell).astype(np.int64)
        r = np.floor((pts[:, 1] - self.y0) / self.cell).astype(np.int64)
        ok = (r >= 0) & (r < self.H) & (c >= 0) & (c < self.W)
        return np.stack([r, c], 1), ok

    def to_xy(self, rc):
        rc = np.asarray(rc, float)
        return np.stack([self.x0 + (rc[..., 1] + 0.5) * self.cell, self.y0 + (rc[..., 0] + 0.5) * self.cell], -1)

    @staticmethod
    def covering(pts, cell=0.1, margin=2.0, step=16, least=96):
        """A window over all of `pts` plus `margin` m on every side, its sides whole
        multiples of `step` cells and at least `least` (ego observations: any world)."""
        if len(pts) == 0:
            return Window.around(pts, least, least, cell)
        lo, hi = pts.min(0) - margin, pts.max(0) + margin
        W = max(least, int(math.ceil((hi[0] - lo[0]) / cell / step)) * step)
        H = max(least, int(math.ceil((hi[1] - lo[1]) / cell / step)) * step)
        return Window.around(pts, H, W, cell)

    @staticmethod
    def around(known_pts, H=96, W=96, cell=0.1):
        """A window centred on the known area (snapped to whole cells)."""
        if len(known_pts) == 0:
            cx = cy = 0.0
        else:
            lo, hi = known_pts.min(0), known_pts.max(0)
            cx, cy = (lo + hi) / 2
        x0 = math.floor((cx - W * cell / 2) / cell) * cell
        y0 = math.floor((cy - H * cell / 2) / cell) * cell
        return Window(x0, y0, H, W, cell)


def rasterize(win, free_pts, occ_pts):
    """Free and occupied (H, W) bool on the window. A cell is occupied if any
    occupied point falls in it, free if a free one does and no occupied one."""
    free = np.zeros((win.H, win.W), bool)
    occ = np.zeros((win.H, win.W), bool)
    for pts, out in ((free_pts, free), (occ_pts, occ)):
        if len(pts):
            rc, ok = win.to_cell(pts)
            out[rc[ok, 0], rc[ok, 1]] = True
    return free & ~occ, occ


# ── the robots as the deciding robot knows them ────────────────────────────
@dataclass
class Robot:
    """One robot in the deciding robot's map frame. Teammates count only when
    `known` (merged: their map and pose can be placed in ours)."""
    free_pts: np.ndarray = field(default_factory=lambda: np.zeros((0, 2)))
    occ_pts: np.ndarray = field(default_factory=lambda: np.zeros((0, 2)))
    pose: tuple = (0.0, 0.0, 0.0)              # x, y, yaw
    goal: tuple = None                          # (x, y) or None
    known: bool = True
    # stage 2 (SLAM-aware policies), from the robot's SLAM node; teammates' via their planners
    trail_pts: np.ndarray = field(default_factory=lambda: np.zeros((0, 2)))   # where it drove
    backlog: float = 0.0                        # s of SLAM work waiting
    lost: bool = False                          # tracking lost
    since_corr: float = 0.0                     # m driven since the last map correction
    links: int = 0                              # graph links with the deciding robot
    # pose options (pose_revisit, graph_model): the path as poses (x, y, yaw, tick)
    # and, for a teammate, the deciding robot's own graph edges with it:
    # [(my node, its node)] (the merge match, then each link the deciding robot found)
    trail_pose: np.ndarray = field(default_factory=lambda: np.zeros((0, 4)))
    edges: list = field(default_factory=list)


def cells_driven(trail_pose, ticks, cell=0.1):
    """A robot's distance driven (cells) up to `ticks` (array), from its path (rows x, y, yaw, tick)."""
    tp = np.asarray(trail_pose, np.float64).reshape(-1, 4)
    ticks = np.asarray(ticks, np.float64)
    if len(tp) < 2:
        return np.zeros(ticks.shape)
    tp = tp[np.argsort(tp[:, 3], kind='stable')]
    cum = np.concatenate([[0.0], np.cumsum(np.hypot(np.diff(tp[:, 0]), np.diff(tp[:, 1])))]) / cell
    return np.interp(ticks, tp[:, 3], cum)


def node_index(trail_pose, ticks, node_ticks, cell=0.1):
    """Pose-graph node of a robot at `ticks` (array) with nodes by its own motion (env node_by
    'move'): its distance driven up to then, in cells, per node_ticks. (Training counts turning on
    the spot too; live the path length stands for it.)"""
    return (cells_driven(trail_pose, ticks, cell) // node_ticks).astype(np.int64)


def load_policy(name, device='cpu', env_config=''):
    """'rl:<checkpoint>', a .pt file (checkpoint or robot_policy.pt) or a baseline name. Returns (policy(obs) -> actions,
    EnvConfig, map size). A baseline normally sees the stage-1 setup; with
    env_config (a run's config.yaml, or a config like configs/slam_aware.yaml)
    it decides inside that setup instead (stage-2 slots, revisit poses, graph)."""
    if name.startswith('rl:') or name.endswith('.pt'):
        from .policy_io import load_policy_file          # the actor only: no training code needed
        pol, env_cfg, H, _ = load_policy_file(name[3:] if name.startswith('rl:') else name, device)
        return pol, env_cfg, H
    if env_config:
        import yaml
        cfg = yaml.safe_load(open(os.path.expanduser(env_config)))
        return baselines.BASELINES[name], EnvConfig(**cfg['env']), int(cfg['train']['map_size'])
    return baselines.BASELINES[name], EnvConfig(), 96


class Decider:
    """Builds the training observation from live maps and asks the policy."""

    def __init__(self, policy, env_cfg, n_agents, H=96, device='cpu', clear_cells=2, goal_clearance=0, escape_cells=4):
        self.policy, self.H, self.dev = policy, H, torch.device(device)
        self.clear_cells = clear_cells        # cells around each robot that cannot be obstacles (0.2 m)
        self.goal_clearance = goal_clearance  # cells: new goals at least this far from known obstacles (0: off)
        self.escape_cells = escape_cells      # cells around the robot passable unless occupied (a tight spot)
        cfg = {**env_cfg.__dict__, 'n_agents': n_agents}
        if env_cfg.ego_obs:
            # a mission can be longer than any training episode (the 22 x 16 m lab: 1200 s); the per-episode
            # tables (graph nodes, visit ticks) are sized for 600 decisions (about 54 min)
            cfg['max_steps_cap'] = max(env_cfg.max_steps_cap, 600)
        self.cfg = EnvConfig(**cfg)
        self.ego = env_cfg.ego_obs            # the grid follows what is known (any world), else H x H
        self.env = self._make_env(H, H)
        self.N = n_agents

    def _make_env(self, H, W):
        e = ExploreEnv(np.zeros((1, H, W), np.uint8), 1, self.cfg, self.dev)
        e.escape_cells = self.escape_cells
        e.keep_full = True                    # ego observations: also the whole-map planes (goal clearance, views)
        e.max_t[0] = self.cfg.max_steps       # the time input: the mission's share gone (ros_planner mission_t)
        return e

    @torch.no_grad()
    def decide(self, robots, trail_pts, t, blocked=(), block_radius=0.3, tick=None):
        """robots[0] is the deciding robot. Returns a dict: action, its kind
        ('keep', 'wait', 'revisit', 'frontier'), the chosen goal (x, y) in the
        robot's map frame (None for keep and wait) and, for a revisit pose, its
        heading goal_yaw and mate (the index in robots of the teammate whose pose
        it is; else None), candidates, the window
        used, and the time taken. Candidate goals within block_radius m of a
        `blocked` point (goals Nav2 could not reach) are masked out before the
        policy chooses. tick: time since the start in training ticks (default
        t x ticks per decision); pose-graph nodes and visit times use it."""
        t0 = time.perf_counter()
        me = robots[0]
        known = [r for r in robots if r.known]
        pts = np.concatenate([np.concatenate([r.free_pts, r.occ_pts]) for r in known] + [np.zeros((0, 2))])
        if self.ego:
            extra = [np.array([r.pose[:2]]) for r in known] + [np.asarray(trail_pts, float).reshape(-1, 2)]
            win = Window.covering(np.concatenate([pts] + extra))
            if (win.H, win.W) != (self.env.H, self.env.W):
                self.env = self._make_env(win.H, win.W)
        else:
            win = Window.around(pts, self.H, self.H)
        e, d = self.env, self.dev
        H, W = e.H, e.W
        merged = np.eye(self.N, dtype=bool)
        for j, r in enumerate(robots):
            f, o = rasterize(win, r.free_pts, r.occ_pts) if r.known else (np.zeros((H, W), bool),) * 2
            e.seen_free[0, j] = torch.as_tensor(f, device=d)
            e.seen_occ[0, j] = torch.as_tensor(o, device=d)
            merged[0, j] = merged[j, 0] = r.known
            rc, ok = win.to_cell(np.array([r.pose[:2]]))
            e.pos[0, j] = torch.as_tensor(np.clip(rc[0], 0, [H - 1, W - 1]), device=d)
            e.heading[0, j] = r.pose[2]
            if r.goal is not None and r.known:
                g, _ = win.to_cell(np.array([r.goal]))
                e.goal[0, j] = torch.as_tensor(np.clip(g[0], 0, [H - 1, W - 1]), device=d)
                e.has_goal[0, j] = True
            else:
                e.goal[0, j] = e.pos[0, j]
                e.has_goal[0, j] = False
        e.merged[0] = torch.as_tensor(merged, device=d)
        # A robot stands where it is: no obstacle within clear_cells of a known robot. Merged maps
        # are a few cm off, so a teammate's obstacle cells can land on a robot; boxed in by them
        # (after inflation) it reaches no goal and could only wait (Isaac, 12 October: 225 s).
        c = self.clear_cells
        for j, r in enumerate(robots):
            if r.known:
                y, x = (int(v) for v in e.pos[0, j])
                e.seen_occ[0, :, max(y - c, 0):y + c + 1, max(x - c, 0):x + c + 1] = False
        e.trail[0] = False
        trails = [trail_pts] + [r.trail_pts if r.known else [] for r in robots[1:]]
        for j, tp in enumerate(trails if self.cfg.slam_model else trails[:1]):
            if len(tp):
                rc, ok = win.to_cell(np.asarray(tp))
                tr = torch.zeros(H, W, dtype=torch.bool, device=d)
                tr[torch.as_tensor(rc[ok, 0], device=d), torch.as_tensor(rc[ok, 1], device=d)] = True
                e.trail[0, j] = tr
        if self.cfg.slam_model:
            for j, r in enumerate(robots):
                e.backlog[0, j] = r.backlog
                e.lost[0, j] = self.cfg.lost_ticks // 2 if r.lost else 0
                e.since_corr[0, j] = r.since_corr
                e.links[0, 0, j] = e.links[0, j, 0] = float(r.links) if j else 0.0
        if self.cfg.slam_model:
            e.tick[0] = int(t) * self.cfg.ticks if tick is None else int(tick)
        if e.poses:
            tt = np.full(H * W, -1, np.int32)
            th = np.zeros(H * W, np.float32)
            for j, r in enumerate(robots):
                tp = np.asarray(r.trail_pose, np.float64).reshape(-1, 4) if (r.known or j == 0) else np.zeros((0, 4))
                tt_j, th_j = tt.copy(), th.copy()
                if len(tp):
                    tp = tp[np.argsort(tp[:, 3], kind='stable')]
                    rc, ok = win.to_cell(tp[:, :2])
                    flat = rc[ok, 0] * W + rc[ok, 1]
                    last = len(flat) - 1 - np.unique(flat[::-1], return_index=True)[1]      # the last visit per cell
                    tt_j[flat[last]] = tp[ok][last, 3].astype(np.int32)
                    th_j[flat[last]] = tp[ok][last, 2]
                e.trail_tick[0, j] = torch.as_tensor(tt_j.reshape(H, W), device=d)
                e.trail_head[0, j] = torch.as_tensor(th_j.reshape(H, W), device=d)
        if e.graph and e.node_at is not None:
            # nodes by each robot's own motion: its node at every tick, from the path it drove
            now = int(e.tick[0])
            ticks = np.arange(e.T)
            for j, r in enumerate(robots):
                tp = r.trail_pose if (r.known or j == 0) else np.zeros((0, 4))
                na = np.minimum(node_index(tp, np.minimum(ticks, now), self.cfg.node_ticks), e.M - 1)
                e.node_at[0, j] = torch.as_tensor(na.astype(np.int16), device=d)
                e.mticks[0, j] = int(cells_driven(tp, [now])[0])
        if e.graph:
            M = e.M
            e.gedge[0] = 0
            for j, r in enumerate(robots):     # robot 0's edges: its own loop closures (own nodes)
                for mine, theirs in (r.edges if (r.known or j == 0) else []):
                    a, b = min(int(mine), M - 1), j * M + min(int(theirs), M - 1)
                    e.gedge[0, 0, a, b] += 1.0
                    e.gedge[0, 0, b, a] += 1.0
        e.t[0] = min(int(t), self.cfg.max_steps - 1)
        e.ask[0] = True                       # the planner asks at its events (event decisions): a free choice
        obs = e.observe()
        if self.goal_clearance > 0:
            # Nav2 cannot plan to a goal inside its obstacle inflation (0.25 m for the car): new goals
            # only where no known obstacle is within goal_clearance cells (keep and wait stay possible)
            near = G.inflate(obs.get('planes_full', obs['planes'])[0, 0, 1][None].bool(), self.goal_clearance)[0]
            cc = obs['cand_cells'][0, 0]
            bad = near[cc[:, 0], cc[:, 1]]
            bad[:e.n_fixed()] = False
            obs['cand_mask'][0, 0] &= ~bad
        if len(blocked):           # (x, y) or (x, y, radius): goals Nav2 reached or could not reach, for a while
            xy = torch.as_tensor(win.to_xy(obs['cand_cells'][0, 0].cpu().numpy()), device=d, dtype=torch.float32)
            bl = np.array([(b[0], b[1], b[2] if len(b) > 2 else block_radius) for b in blocked], np.float32)
            b = torch.as_tensor(bl, device=d)
            near = (torch.cdist(xy, b[:, :2]) < b[None, :, 2]).any(1)
            near[:e.n_fixed()] = False                        # keep and WAIT stay possible
            obs['cand_mask'][0, 0] &= ~near
        a = int(self.policy(obs)[0, 0])
        cand = obs['cand_cells'][0, 0].cpu().numpy()
        mask = obs['cand_mask'][0, 0].cpu().numpy()
        keep = a == 0
        wait = a == 1 and e.n_fixed() == 2
        yaw = float(e.cand_yaw[0, 0, a])
        n_rev = self.cfg.n_revisit if self.cfg.slam_model else 0
        n_own = self.cfg.n_self_revisit if e.graph else 0
        nf0 = e.n_fixed()
        kind = ('keep' if keep else 'wait' if wait else 'revisit' if nf0 <= a < nf0 + n_rev
                else 'loop' if nf0 + n_rev <= a < nf0 + n_rev + n_own else 'frontier')
        mate = int(e.cand_mate[0, 0, a])
        nf = e.n_fixed()
        slot_kind = lambda k: ('revisit' if nf <= k < nf + n_rev else 'loop' if k < nf + n_rev + n_own  # noqa: E731
                               else 'frontier')
        xy_all = win.to_xy(cand)
        offered = [(float(xy_all[k][0]), float(xy_all[k][1]), slot_kind(k)) for k in range(nf, len(cand)) if mask[k]]
        return {'action': a, 'keep': keep, 'wait': wait, 'kind': kind,
                'mate': mate if (kind == 'revisit' and mate > 0) else None,
                'goal': None if (keep or wait) else tuple(win.to_xy(cand[a])),
                'goal_yaw': None if (keep or wait or math.isnan(yaw)) else yaw,
                # the heading it arrives with along its shortest path (for a frontier goal's Nav2 heading)
                'arrive_yaw': float(e.cand_arrive[0, 0, a]),
                # the frontier cell a frontier goal is for, and the share of the sensor disk unknown around it
                # (the planner's event "frontier used up": unknown_near)
                'front': tuple(win.to_xy(e.cand_front[0, 0, a].cpu().numpy())),
                'front_gain': float(e.cand_gain[0, 0, a]),
                'candidates': win.to_xy(cand[mask]), 'n_candidates': int(mask[1:].sum()), 'offered': offered,
                'window': win, 'obs': obs, 'ms': 1e3 * (time.perf_counter() - t0)}

    def unknown_near(self, robots, xy):
        """The share of the sensor disk around xy (m) that is unknown in the deciding
        robot's integrated map (its own map and merged teammates'), as training
        measures a frontier goal (ExploreEnv.cand_gain). None if xy is off the map window."""
        known = [r for r in robots if r.known]
        pts = np.concatenate([np.concatenate([r.free_pts, r.occ_pts]) for r in known] + [np.zeros((0, 2))])
        win = Window.around(pts, self.H, self.H)
        seen = np.zeros((self.H, self.H), bool)
        for r in known:
            f, o = rasterize(win, r.free_pts, r.occ_pts)
            seen |= f | o
        rc, ok = win.to_cell(np.array([xy], float))
        if not ok[0]:
            return None
        k = self.env.ig_kernel.cpu().numpy() > 0
        rad = k.shape[0] // 2
        unk = np.pad(~seen, rad)                      # off the window counts as known, as in training
        y, x = rc[0]
        return float((unk[y:y + 2 * rad + 1, x:x + 2 * rad + 1] & k).sum() / self.env.ig_norm)
