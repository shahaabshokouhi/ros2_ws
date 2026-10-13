# Copied from ma_explore (55b8544) by scripts/sync_robot_package.sh: edit it there, not here.
"""Batched multi-robot exploration environment on the GPU.

B environments x N robots step together. Each robot has its OWN map (what it
has seen). Two robots' maps merge (for good) once the areas they have seen
overlap by `merge_cells` cells (and `merge_delay_s` has passed since), standing
in for SLAM map merging; with
`merge_transitive` merging also passes through a common teammate (off: only
direct merges, as ORB-SLAM3's `new` method). A robot's integrated map is its own
plus those of the robots it is merged with, and it knows a teammate's position,
goal and path only once merged.

A decision step: every robot picks one of K candidate goals from its
integrated map (index 0 = keep the current goal, or stay if there is none;
1.. = frontier cells with the most unknown space in sensor range, spread out),
then the robots drive `ticks` cells along shortest paths on the true map (the
stand-in for Nav2), sensing after every cell. Robots that reach their goal wait
for the next decision.

Reward (shared by the team) per step:
    w_team * gain in team coverage (union of all robots' own maps)
  + w_know * gain in mean integrated-map coverage (knowledge spread by merging)
  - time_cost, + success_bonus when team coverage reaches `success` (episode ends).
Coverage = fraction of the map's free cells seen.

Localization is perfect in this version (all maps share one frame).

Stage 2 (`slam_model: true`, configs/slam_aware.yaml) adds what a real robot's
SLAM and computer go through, as hidden per-robot state:
  * backlog (s of SLAM work waiting): grows with driving, with new ground in
    view (new keyframes), with a teammate's map arriving after a merge and with
    the place-recognition burst of a new merge; the computer works it off at
    a fixed rate. Frame lag = lag0 + backlog.
  * tracking loss: while moving, more likely with high lag and sharp turns;
    a lost robot stands still for `lost_ticks` (relocalizing) and its drift jumps.
  * drift (m): grows with distance driven (faster with lag); a loop closure
    (back on ground it saw `loop_age` ticks ago) shrinks it; a link (a match
    between its map and a merged teammate's, see link_model) pulls it towards
    that teammate's drift and connects their pose graphs.
Two more actions: WAIT (slot 1: stay this decision so the backlog drains,
keeping the goal) and REVISIT candidates (places a merged teammate drove
through). The robot observes only what its SLAM node publishes (backlog and
lag, lost or not, distance since the last map correction, links per
teammate); the true drift is for the critic only. Reward adds
- w_drift x drift term - w_lost x lost time + w_link x new links (capped per pair).
The drift term (drift_penalty) is either the mean drift every decision
('level', the first model: driving always adds drift, so standing still pays)
or only the AVOIDABLE drift ('avoidable'): what driving with a SLAM backlog and
tracking losses add on top of plain odometry drift, minus what loop closures
and links remove.

Two options of stage 2 (configs/slam_aware.yaml turns both on):
  * pose_revisit: a revisit target is a merged teammate's past POSE (cell and
    heading): place recognition needs a similar view, so a link forms with a
    chance that falls with the distance to that pose (link_radius) and the
    heading difference (zero at link_fov_deg). On arrival the robot turns to
    the target heading (turn_ticks per 180 deg, standing).
  * graph_model: each robot's pose graph as IT knows it: one node per
    node_ticks ticks for itself and each robot it has merged with (odometry
    chains), plus the edges it found itself: the merge match and its own links
    with that teammate (a teammate's links with others are not shared). For
    its algebraic connectivity lambda2 (2nd smallest Laplacian eigenvalue),
    each revisit pose gets the relative gain lambda2'/lambda2 - 1 of an edge
    from the robot's newest node to the teammate's node there (exact for one
    edge, from the eigendecomposition: _graph_eigs), and each pair a merge
    error that shrinks with its links. Revisit slots are the
    teammate poses with the most gain (connectivity + merge accuracy) for the
    path; reward adds w_conn x connectivity gain + w_merge x merge error removed
    (in units of the error at the merge) by the links that formed. Merge
    error = sqrt(merge_floor^2 + merge_sigma0^2 / (merge_n0 + links)).

Where links come from (link_model):
  * 'pose' (the first guess): near a merged teammate's past pose, as above
    (without pose_revisit: on ground the teammate saw first).
  * 'views' (calibrated from Isaac sessions, ma_explore.calibrate_links):
    ORB-SLAM3 matches keyframes from anywhere in both maps (the matched
    keyframes were a median 37 s old on the robot's side), so a link does not
    need the robot to stand anywhere in particular. A view is a cell seen
    while facing one of link_bins directions; each tick a merged pair gains a
    link at the SLAM's verification pace (link_rate_fast per second until the
    pair has link_strong measurements, then link_rate_slow), as long as the
    views both maps hold exceed link_views per link. The link joins the two
    graphs where both robots first had a random one of those shared views.

Loops (loop_model 'views', with graph_model): a robot that sees at least
loop_views of its own views again (first seen loop_age ticks ago or more) gets,
at loop_rate per second, an edge in its own graph from its newest node to the
node of the oldest such view: ORB-SLAM3 ties new keyframes to old ones that saw
the same points (covisibility); explicit loop closures are rarer. Its pose
uncertainty is the effective resistance between its first and newest node
(+1 per node along a plain path, so it grows with every keyframe; parallel
paths through loops and links lower it). A loop cuts drift by the share of the
uncertainty it removes; reward + w_unc x uncertainty removed. With
n_self_revisit, REVISIT-OWN slots offer the robot's own older poses (cell and
heading), ranked by the uncertainty a loop there would remove (exact for one
edge: r^2 / (1 + R_nc), from the same eigendecomposition).
"""
import math
from dataclasses import dataclass, field

import torch
import torch.nn.functional as F

from . import geometry as G

N_PLANES = 8         # actor map planes (see observe()), stage 1
N_CAND_FEAT = 8      # candidate features
N_MATE_FEAT = 6      # teammate features
N_SCALARS = 4
N_STATE_PLANES = 6   # critic planes: 5 shared by the environment + the robot's own position
N_STATE_SCALARS = 3
STAGE1_DIMS = dict(planes=N_PLANES, cand=N_CAND_FEAT, mate=N_MATE_FEAT, scalars=N_SCALARS,
                   state_planes=N_STATE_PLANES, state_scalars=N_STATE_SCALARS)


def obs_dims(cfg):
    """Observation sizes for the networks (stage 1, or stage 2 with slam_model)."""
    if not cfg.slam_model:
        return dict(STAGE1_DIMS)
    d = dict(planes=N_PLANES + 1, cand=N_CAND_FEAT + 2, mate=N_MATE_FEAT + 2, scalars=N_SCALARS + 5,
             state_planes=N_STATE_PLANES, state_scalars=N_STATE_SCALARS + 4)
    if cfg.pose_revisit:      # heading change at a revisit pose (cos, sin) and the turn it takes
        d['cand'] += 3
    if cfg.graph_model:       # connectivity and merge gain; merge error per teammate; own lambda2
        d['cand'] += 2
        d['mate'] += 1
        d['scalars'] += 1
        d['state_scalars'] += 2
    if cfg.graph_model and cfg.n_self_revisit:   # is an own pose; uncertainty a loop there removes; own uncertainty
        d['cand'] += 2
        d['scalars'] += 1
        d['state_scalars'] += 1
    return d


@dataclass
class EnvConfig:
    n_agents: int = 3
    ticks: int = 8                 # cells driven per decision (0.1 m cells: 0.8 m)
    max_steps: int = 96            # decisions per episode
    sensor_range: float = 15.0     # cells (1.5 m)
    n_rays: int = 160
    fov_deg: float = 360.0         # camera field of view (360: all around)
    inflate: int = 1               # cells kept clear around obstacles for driving
    n_candidates: int = 16         # incl. index 0 (keep goal / stay)
    goal_standoff: int = 0         # cells: a frontier goal is placed this far back along the robot's path to
                                   # it, on known free ground, facing the frontier, not on the edge of the
                                   # unknown where an unseen obstacle can end up right in front of the camera
                                   # (real robot, 13 October). 0: the frontier cell itself (before).
    turn_at_own_frontier: bool = True  # a new goal on the robot's own cell (a frontier because the unknown is
                                       # behind a narrow camera) turns it around to look (False: it stands
                                       # still, and rule-based planners then stay stuck; before 12 October)
    nms_radius: int = 4            # candidate spacing (cells)
    merge_cells: int = 20          # seen-area overlap that merges two robots' maps ...
    merge_delay_s: float = 0.0     # ... this long after it is reached (tick_s per cell driven; Isaac: 35 s)
    share: str = 'overlap'         # overlap | full | none
    start: str = 'mixed'           # cluster | spread | mixed
    cluster_radius: int = 6
    spread_min: int = 20
    success: float = 0.95
    w_team: float = 10.0
    w_know: float = 5.0
    time_cost: float = 0.02
    success_bonus: float = 1.0
    # ── stage 2: SLAM and compute state (off = stage 1, unchanged) ──
    slam_model: bool = False
    tick_s: float = 0.67           # seconds per tick (0.1 m at 0.15 m/s)
    wait_action: bool = True       # slot 1 = wait this decision
    n_revisit: int = 3             # revisit slots (places merged teammates drove through)
    revisit_nms: int = 6
    capacity: float = 0.67         # s of SLAM work done per tick (one core-second per second)
    work_move: float = 0.45        # s of work per tick driven (tracking at 30 fps)
    work_new: float = 0.6          # s of work per tick x share of the sensor disk newly seen
    work_import: float = 1.0       # s of work per sensor disk of teammate map received
    work_merge: float = 3.0        # s of work for a new merge (place recognition burst)
    backlog_max: float = 10.0
    lag0: float = 0.05             # s frame lag with nothing waiting
    lag_crit: float = 1.5          # s lag where tracking loss becomes likely
    lag_scale: float = 0.3
    lost_p0: float = 0.0005        # per moving tick
    lost_lag: float = 0.08         # extra per moving tick at high lag
    lost_turn: float = 0.01        # extra per moving tick x (turn / 180 deg)
    lost_ticks: int = 12           # standing still while relocalizing (~8 s)
    lost_drift: float = 0.05       # m of drift added by a tracking loss
    drift_rate: float = 0.01       # m of drift per m driven ...
    drift_lag: float = 1.0         # ... times (1 + drift_lag x lag)
    loop_age: int = 40             # ticks: ground seen this long ago can close a loop
    p_loop: float = 0.05           # per eligible tick
    loop_gain: float = 0.5         # drift x (1 - loop_gain) on a loop closure
    loop_cooldown: int = 16
    # loops (stage 2): 'random' = p_loop per tick on ground seen loop_age ticks ago (the first model);
    # 'views' (needs link_model views) = when the camera sees at least loop_views of the robot's own
    # views (cell x heading bin) first seen loop_age ticks ago, at loop_rate per second, as ORB-SLAM3's
    # place recognition needs a similar view; with graph_model it joins the robot's newest graph node to
    # the node of that old view and cuts drift as much as it cuts the pose uncertainty (graph resistance)
    loop_model: str = 'random'
    loop_views: int = 10
    loop_rate: float = 0.5
    n_self_revisit: int = 0        # REVISIT-OWN slots: the robot's own older poses (cell + heading), ranked
                                   # by the pose uncertainty a loop there would remove (graph_model)
    w_unc: float = 0.0             # reward per unit of pose uncertainty removed by loop closures
    p_link: float = 0.1            # per tick on ground a merged teammate saw first
    link_gain: float = 0.5         # pull towards the teammate's drift
    link_cooldown: int = 16
    link_cap: int = 3              # rewarded links per pair
    w_drift: float = 2.0           # reward: drift term, see drift_penalty
    drift_penalty: str = 'level'   # 'level': - w_drift x mean drift (m) every decision (the first stage-2
                                   # model; it pays to stand still); 'avoidable': - w_drift x (drift added
                                   # beyond driving with an idle SLAM: backlog, losses - drift removed by
                                   # loop closures and links), mean per robot
    w_lost: float = 0.5            # reward: - w_lost x share of robot-time lost
    w_link: float = 0.05           # reward: + per new link (up to link_cap per pair)
    merge_transitive: bool = True  # False: only direct merges (ORB-SLAM3 new); stage 1 keeps True
    # revisit poses (stage 2 option; module docstring)
    pose_revisit: bool = False
    link_radius: int = 2           # cells from the teammate's pose where a link can form
    link_fov_deg: float = 60.0     # heading difference where the link chance reaches zero
    p_link_pose: float = 0.3       # per tick at the exact pose and heading
    # where links come from (stage 2): 'pose' = near a merged teammate's past pose (link_radius,
    # link_fov_deg, p_link_pose with pose_revisit, else p_link; the first guess); 'views' = from the
    # views both maps hold, at the SLAM's verification pace (calibrated in Isaac: ma_explore.calibrate_links)
    link_model: str = 'pose'
    link_bins: int = 4             # 'views': a view = a cell seen while facing one of link_bins directions
    link_views: float = 20.0       # 'views': shared views per link (a pair's links stay below shared / link_views)
    link_rate_fast: float = 0.15   # 'views': links per second while the pair has < link_strong measurements
    link_rate_slow: float = 0.094  # 'views': links per second after that (ORB-SLAM3 then only spot-checks)
    link_strong: int = 20          # 'views': measurements where that switch happens (MA_NEW_ALIGN_STRONG)
    turn_ticks: float = 6.0        # ticks to turn 180 deg at a revisit pose (a car: a multi-point turn)
    revisit_yaw_err_deg: float = 0.0   # arrival heading error, uniform +-this (Nav2 accepts 34 deg)
    # each robot's own pose graph (stage 2 option)
    graph_model: bool = False
    node_ticks: int = 32           # ticks per pose-graph node (about 3 m driven)
    w_conn: float = 0.5            # reward per unit of relative connectivity gain (dlambda2 / lambda2)
    conn_cap: float = 2.0          # largest relative gain counted for one link
    merge_sigma0: float = 0.05     # m: spread of one measurement; merge error =
                                   # sqrt(merge_floor^2 + merge_sigma0^2 / (merge_n0 + links))
    merge_n0: int = 1              # measurements in the merge match itself (Isaac: 4)
    merge_floor: float = 0.0       # m: merge error that more links do not remove (Isaac: 0.02)
    w_merge: float = 1.0           # reward per sigma0 of merge error removed


class ExploreEnv:
    def __init__(self, map_pool, num_envs, cfg=None, device='cuda', seed=0):
        self.cfg = cfg or EnvConfig()
        c = self.cfg
        self.B, self.N, self.dev = num_envs, c.n_agents, torch.device(device)
        self.pool = torch.as_tensor(map_pool, device=self.dev).bool()            # (P, H, W)
        P, self.H, self.W = self.pool.shape
        self.pool_stand = ~G.inflate(self.pool, c.inflate) & ~self.pool           # where a robot may be
        self.gen = torch.Generator(device=self.dev).manual_seed(seed)
        self.rays, self.ray_ang = G.ray_table(c.sensor_range, c.n_rays, self.dev)
        self.ig_kernel = G.disk_kernel(c.sensor_range, self.dev)
        self.ig_norm = float(self.ig_kernel.sum())
        self.fov = math.radians(c.fov_deg)
        B, N, H, W, d = self.B, self.N, self.H, self.W, self.dev
        self.map_idx = torch.zeros(B, dtype=torch.long, device=d)
        self.occ = torch.zeros(B, H, W, dtype=torch.bool, device=d)
        self.stand = torch.zeros(B, H, W, dtype=torch.bool, device=d)
        self.target = torch.zeros(B, H, W, dtype=torch.bool, device=d)
        self.n_target = torch.ones(B, device=d)
        self.seen_free = torch.zeros(B, N, H, W, dtype=torch.bool, device=d)
        self.seen_occ = torch.zeros_like(self.seen_free)
        self.trail = torch.zeros_like(self.seen_free)
        self.merged = torch.zeros(B, N, N, dtype=torch.bool, device=d)
        self.overlap_since = torch.full((B, N, N), -1, dtype=torch.long, device=d)   # decision the overlap was reached
        self.merge_delay = round(c.merge_delay_s / (c.ticks * c.tick_s))              # decisions
        self.pos = torch.zeros(B, N, 2, dtype=torch.long, device=d)
        self.goal = torch.zeros(B, N, 2, dtype=torch.long, device=d)
        self.has_goal = torch.zeros(B, N, dtype=torch.bool, device=d)
        self.goal_field = torch.full((B, N, H, W), G.INF, device=d)
        self.heading = torch.zeros(B, N, device=d)
        self.t = torch.zeros(B, dtype=torch.long, device=d)
        self.dist = torch.zeros(B, N, device=d)
        self.t90 = torch.full((B,), -1, dtype=torch.long, device=d)
        self.cand = torch.zeros(B, N, c.n_candidates, 2, dtype=torch.long, device=d)
        self.cand_mask = torch.zeros(B, N, c.n_candidates, dtype=torch.bool, device=d)
        self.team_cov = torch.zeros(B, device=d)
        self.know_cov = torch.zeros(B, device=d)
        self.waiting = torch.zeros(B, N, dtype=torch.bool, device=d)
        if self.cfg.slam_model:
            self.backlog = torch.zeros(B, N, device=d)
            self.drift = torch.zeros(B, N, device=d)
            self.since_corr = torch.zeros(B, N, device=d)       # m driven since the last loop / link
            self.lost = torch.zeros(B, N, dtype=torch.long, device=d)
            self.links = torch.zeros(B, N, N, device=d)
            self.last_link = torch.full((B, N, N), -10 ** 6, dtype=torch.long, device=d)
            self.last_loop = torch.full((B, N), -10 ** 6, dtype=torch.long, device=d)
            self.first_seen = torch.full((B, N, H, W), -1, dtype=torch.int32, device=d)
            self.tick = torch.zeros(B, dtype=torch.long, device=d)
            self.step_lost = torch.zeros(B, device=d)
            self.step_links = torch.zeros(B, device=d)
            self.step_drift = torch.zeros(B, device=d)      # avoidable drift added - drift corrected
        assert not (c.graph_model and not c.pose_revisit), 'graph_model needs pose_revisit (links at poses)'
        assert c.link_model in ('pose', 'views'), c.link_model
        assert c.loop_model in ('random', 'views'), c.loop_model
        assert not (c.loop_model == 'views' and c.link_model != 'views'), "loop_model views needs link_model views"
        assert not (c.n_self_revisit and not (c.graph_model and c.pose_revisit)), 'n_self_revisit needs graph_model'
        self.views = c.slam_model and c.link_model == 'views'
        if self.views:
            assert c.max_steps * c.ticks < 2 ** 15, 'first_view keeps ticks as int16'
            # tick each robot first had each view (cell, heading bin); -1 = not yet
            self.first_view = torch.full((B, N, H, W, c.link_bins), -1, dtype=torch.int16, device=d)
        self.poses = c.slam_model and (c.pose_revisit or c.graph_model)
        self.graph = c.slam_model and c.graph_model
        K = c.n_candidates
        self.cand_yaw = torch.full((B, N, K), float('nan'), device=d)       # target heading (revisit poses)
        self.cand_mate = torch.full((B, N, K), -1, dtype=torch.long, device=d)  # whose pose (revisit poses)
        self.view_yaw = torch.full((B, N), float('nan'), device=d)       # heading to take at a viewpoint goal
        if self.poses:
            self.trail_tick = torch.full((B, N, H, W), -1, dtype=torch.int32, device=d)   # last visit
            self.trail_head = torch.zeros(B, N, H, W, device=d)                           # heading then
            self.goal_yaw = torch.full((B, N), float('nan'), device=d)
            self.turning = torch.zeros(B, N, dtype=torch.long, device=d)
            self.turn_rate = torch.zeros(B, N, device=d)
        if self.graph:
            self.M = math.ceil(c.max_steps * c.ticks / c.node_ticks) + 1                # nodes per robot
            NM = N * self.M
            # each robot's own edges (merge matches, its links), between team node indices j * M + k
            self.gedge = torch.zeros(B, N, NM, NM, device=d)
            r = torch.arange(NM, device=d)
            chain = torch.zeros(NM, NM, device=d)
            nxt = r[:-1][(r[:-1] + 1) % self.M != 0]
            chain[nxt, nxt + 1] = chain[nxt + 1, nxt] = 1.0                                 # odometry
            self.chain = chain
            self.lam2 = torch.zeros(B, N, device=d)
            self.node_gain = torch.zeros(B, N, NM, device=d)
            self.obs_node = torch.zeros(B, dtype=torch.long, device=d)
            self.step_conn = torch.zeros(B, device=d)
            self.step_merge = torch.zeros(B, device=d)
            self.step_unc = torch.zeros(B, device=d)                       # pose uncertainty removed by loops
            self.unc = torch.zeros(B, N, device=d)                         # pose uncertainty (graph resistance)
            self.loop_node_gain = torch.zeros(B, N, self.M, device=d)           # what a loop to each own node removes
        if self.views:
            self.old_views = torch.zeros(B, N, device=d)                   # own old views in sight now
            self.old_view_tick = torch.zeros(B, N, dtype=torch.long, device=d)

    # ── episodes ─────────────────────────────────────────────────────────
    def reset(self):
        self._reset(torch.ones(self.B, dtype=torch.bool, device=self.dev))
        return self.observe()

    def _rand(self, *shape):
        return torch.rand(*shape, generator=self.gen, device=self.dev)

    def _reset(self, m):
        idx = m.nonzero().squeeze(1)
        if idx.numel() == 0:
            return
        c, N, H, W = self.cfg, self.N, self.H, self.W
        k = idx.numel()
        mi = torch.randint(0, self.pool.shape[0], (k,), generator=self.gen, device=self.dev)
        self.map_idx[idx] = mi
        self.occ[idx] = self.pool[mi]
        self.stand[idx] = self.pool_stand[mi]
        self.target[idx] = ~self.pool[mi]
        self.n_target[idx] = self.target[idx].flatten(1).sum(1).float()
        # start poses
        stand = self.stand[idx].flatten(1).float()                              # (k, HW)
        first = torch.multinomial(stand, 1, generator=self.gen).squeeze(1)
        pos = torch.zeros(k, N, 2, dtype=torch.long, device=self.dev)
        pos[:, 0] = torch.stack([first // W, first % W], -1)
        yy = torch.arange(H, device=self.dev)[None, :, None]
        xx = torch.arange(W, device=self.dev)[None, None, :]
        mode = c.start
        cluster = (self._rand(k) < 0.5) if mode == 'mixed' else torch.full((k,), mode == 'cluster', device=self.dev)
        for a in range(1, N):
            near = ((yy - pos[:, 0, 0, None, None]) ** 2 + (xx - pos[:, 0, 1, None, None]) ** 2) <= c.cluster_radius ** 2
            far = torch.ones_like(near)
            for b2 in range(a):
                far &= ((yy - pos[:, b2, 0, None, None]) ** 2 + (xx - pos[:, b2, 1, None, None]) ** 2) >= c.spread_min ** 2
            w = torch.where(cluster[:, None, None], near, far).flatten(1).float() * stand
            w = torch.where(w.sum(1, keepdim=True) > 0, w, stand)                # fall back: anywhere
            pick = torch.multinomial(w, 1, generator=self.gen).squeeze(1)
            pos[:, a] = torch.stack([pick // W, pick % W], -1)
        self.pos[idx] = pos
        self.goal[idx] = pos
        self.has_goal[idx] = False
        self.goal_field[idx] = G.INF
        self.heading[idx] = self._rand(k, N) * 2 * math.pi
        self.seen_free[idx] = False
        self.seen_occ[idx] = False
        self.trail[idx] = False
        self.merged[idx] = torch.eye(N, dtype=torch.bool, device=self.dev)
        self.overlap_since[idx] = -1
        self.t[idx] = 0
        self.dist[idx] = 0
        self.t90[idx] = -1
        self.waiting[idx] = False
        if c.slam_model:
            for name in ('backlog', 'drift', 'since_corr', 'links', 'step_lost', 'step_links', 'step_drift'):
                getattr(self, name)[idx] = 0
            self.lost[idx] = 0
            self.last_link[idx] = -10 ** 6
            self.last_loop[idx] = -10 ** 6
            self.first_seen[idx] = -1
            self.tick[idx] = 0
        if self.views:
            self.first_view[idx] = -1
        if self.poses:
            self.trail_tick[idx] = -1
            self.trail_head[idx] = 0
            self.goal_yaw[idx] = float('nan')
            self.turning[idx] = 0
        if self.graph:
            self.gedge[idx] = 0
            self.lam2[idx] = 0
            self.node_gain[idx] = 0
            self.step_conn[idx] = 0
            self.step_merge[idx] = 0
            self.step_unc[idx] = 0
            self.unc[idx] = 0
            self.loop_node_gain[idx] = 0
        self._sense(idx)
        self._merge()
        tc, kc = self._coverage()
        self.team_cov[idx], self.know_cov[idx] = tc[idx], kc[idx]

    # ── simulation ───────────────────────────────────────────────────────
    def _sense(self, envs=None):
        """Every robot (in `envs`, default all) adds what it sees from where it is."""
        if envs is None:
            envs = torch.arange(self.B, device=self.dev)
        k, N, H, W = envs.numel(), self.N, self.H, self.W
        occ = self.occ[envs][:, None].expand(k, N, H, W).reshape(k * N, H, W)
        pos = self.pos[envs].reshape(k * N, 2)
        free_i, occ_i = G.sense(occ, pos, self.rays, self.ray_ang, self.heading[envs].reshape(-1), self.fov)
        sf = self.seen_free[envs].reshape(-1)
        so = self.seen_occ[envs].reshape(-1)
        new_count = None
        if self.cfg.slam_model:
            seen_i = torch.unique(torch.cat([free_i, occ_i]))
            fresh = seen_i[~(sf[seen_i] | so[seen_i])]
            fs = self.first_seen[envs].reshape(-1)
            fs[fresh] = self.tick[envs][fresh // (N * H * W)].int()
            self.first_seen[envs] = fs.view(k, N, H, W)
            new_count = torch.bincount(fresh // (H * W), minlength=k * N).view(k, N).float()
            if self.views:   # the views: cells seen, each with the robot's heading bin
                nb = self.cfg.link_bins
                hb = torch.floor(torch.remainder(self.heading[envs].reshape(-1) + math.pi / nb, 2 * math.pi)
                                 / (2 * math.pi / nb)).long().clamp(max=nb - 1)
                vi = seen_i * nb + hb[seen_i // (H * W)]
                fv = self.first_view[envs].reshape(-1)
                if self.cfg.loop_model == 'views':     # own views first seen long ago, in sight now (loops)
                    first = fv[vi].long()
                    ri = vi // (H * W * nb)                                              # robot (env-major)
                    now_t = self.tick[envs][vi // (N * H * W * nb)]
                    old = (first >= 0) & (now_t - first >= self.cfg.loop_age)
                    cnt = torch.bincount(ri[old], minlength=k * N).float()
                    oldest = torch.full((k * N,), 10 ** 9, dtype=torch.long, device=self.dev)
                    oldest.scatter_reduce_(0, ri[old], first[old], reduce='amin')
                    self.old_views[envs] = cnt.view(k, N)
                    self.old_view_tick[envs] = oldest.view(k, N)
                vi = vi[fv[vi] < 0]
                fv[vi] = self.tick[envs][vi // (N * H * W * nb)].to(torch.int16)
                self.first_view[envs] = fv.view(k, N, H, W, nb)
        sf[free_i] = True
        so[occ_i] = True
        self.seen_free[envs] = sf.view(k, N, H, W)
        self.seen_occ[envs] = so.view(k, N, H, W)
        tr = self.trail[envs].reshape(k * N, H * W)
        tr[torch.arange(k * N, device=self.dev), pos[:, 0] * W + pos[:, 1]] = True
        self.trail[envs] = tr.view(k, N, H, W)
        if self.poses:
            r, cell = torch.arange(k * N, device=self.dev), pos[:, 0] * W + pos[:, 1]
            tt = self.trail_tick[envs].reshape(k * N, H * W)
            th = self.trail_head[envs].reshape(k * N, H * W)
            tt[r, cell] = self.tick[envs].repeat_interleave(N).int()
            th[r, cell] = self.heading[envs].reshape(-1)
            self.trail_tick[envs] = tt.view(k, N, H, W)
            self.trail_head[envs] = th.view(k, N, H, W)
        return new_count

    def _merge(self):
        N = self.N
        if self.cfg.share == 'full':
            self.merged[:] = True
            return
        if self.cfg.share == 'none':
            return
        seen = (self.seen_free | self.seen_occ).flatten(2).float()                 # (B, N, HW)
        overlap = torch.einsum('bnk,bmk->bnm', seen, seen)
        reached = overlap >= self.cfg.merge_cells
        self.overlap_since = torch.where(reached & (self.overlap_since < 0), self.t[:, None, None], self.overlap_since)
        m = self.merged | (reached & (self.t[:, None, None] - self.overlap_since >= self.merge_delay))
        if self.cfg.merge_transitive:
            for _ in range(N - 1):                                                # transitive closure
                m = m | (torch.einsum('bnk,bkm->bnm', m.float(), m.float()) > 0)
        new = m & ~self.merged
        self.merged = m
        if self.graph and new.any():
            self._merge_edges(new)

    def _merge_edges(self, new):
        """The merge match is the first edge between two robots' graphs: from my
        current node to the teammate's node where its path came closest to me."""
        B, N, H, W, M, d = self.B, self.N, self.H, self.W, self.M, self.dev
        yy = torch.arange(H, device=d).view(1, 1, 1, H, 1)
        xx = torch.arange(W, device=d).view(1, 1, 1, 1, W)
        py, px = self.pos[..., 0].view(B, N, 1, 1, 1), self.pos[..., 1].view(B, N, 1, 1, 1)
        tt = self.trail_tick[:, None]                                            # (B, 1, j, H, W)
        d2 = torch.where(tt >= 0, ((yy - py) ** 2 + (xx - px) ** 2).float(), torch.full((), 1e9, device=d))
        near = d2.flatten(3).argmin(3)                                           # (B, i, j)
        their_tick = tt.expand(B, N, N, H, W).flatten(3).gather(3, near[..., None]).squeeze(3)
        b, i, j = new.nonzero(as_tuple=True)
        mine = i * M + (self.tick[b] // self.cfg.node_ticks).clamp(max=M - 1)
        theirs = j * M + (their_tick[b, i, j].long() // self.cfg.node_ticks).clamp(0, M - 1)
        self.gedge[b, i, mine, theirs] += 1.0
        self.gedge[b, i, theirs, mine] += 1.0

    def integrated(self):
        """Each robot's integrated map: (free, occupied), (B, N, H, W) bool."""
        m = self.merged.float()
        f = torch.einsum('bnm,bmk->bnk', m, self.seen_free.flatten(2).float()) > 0
        o = torch.einsum('bnm,bmk->bnk', m, self.seen_occ.flatten(2).float()) > 0
        sh = (self.B, self.N, self.H, self.W)
        return f.view(sh), o.view(sh)

    def _coverage(self):
        team = self.seen_free.any(1) & self.target
        tc = team.flatten(1).sum(1) / self.n_target
        f, _ = self.integrated()
        kc = (f & self.target[:, None]).flatten(2).sum(2).float().mean(1) / self.n_target
        return tc, kc

    def _drive(self):
        """One cell along each robot's path to its goal (8-connected)."""
        B, N, H, W = self.B, self.N, self.H, self.W
        Fd = self.goal_field.view(B * N, H, W)
        P = F.pad(Fd, (1, 1, 1, 1), value=G.INF)
        y, x = self.pos[..., 0].reshape(-1), self.pos[..., 1].reshape(-1)
        r = torch.arange(B * N, device=self.dev)
        here = Fd[r, y, x]
        best, by, bx = here, y, x
        for dy, dx, _ in G._SHIFTS:
            v = P[r, y + 1 + dy, x + 1 + dx]
            better = v < best
            best = torch.where(better, v, best)
            by = torch.where(better, y + dy, by)
            bx = torch.where(better, x + dx, bx)
        frozen = self.waiting.view(-1) | ~self.has_goal.view(-1)      # no goal: stand (a stale path field must not move it)
        if self.cfg.slam_model:
            frozen = frozen | (self.lost.view(-1) > 0)
        if self.poses:
            turning = (self.turning.view(-1) > 0) & (self.lost.view(-1) == 0)
            frozen = frozen | turning
        by, bx = torch.where(frozen, y, by), torch.where(frozen, x, bx)
        moving = (by != y) | (bx != x)
        step = torch.where((by != y) & (bx != x), G.SQRT2, 1.0) * moving
        new_heading = torch.atan2((by - y).float(), (bx - x).float())
        turn = torch.remainder(new_heading - self.heading.view(-1) + math.pi, 2 * math.pi) - math.pi
        turn = torch.where(moving, turn.abs(), torch.zeros_like(turn))
        self.heading.view(-1)[moving] = new_heading[moving]
        self.pos[..., 0] = by.view(B, N)
        self.pos[..., 1] = bx.view(B, N)
        self.dist += step.view(B, N)
        reached = (self.pos == self.goal).all(-1)
        if self.poses:
            # turning on the spot counts as camera motion (tracking work, loss risk) without distance
            turn = torch.where(turning, self.turn_rate.view(-1), turn)
            self.turning.view(-1)[turning] -= 1
            moving = moving | turning
            self._start_turn(reached & self.has_goal)
        else:                                    # without turn modelling: face the frontier at a stand-off goal
            arr = reached & self.has_goal & ~torch.isnan(self.view_yaw)
            self.heading[arr] = self.view_yaw[arr]
        self.has_goal &= ~reached
        return moving.view(B, N), step.view(B, N), turn.view(B, N)

    def _start_turn(self, arrived):
        """Robots arriving at a revisit pose turn to its heading (the view the teammate had)."""
        m = arrived & ~torch.isnan(self.goal_yaw)
        if not m.any():
            return
        dpsi = torch.remainder(self.goal_yaw - self.heading + math.pi, 2 * math.pi) - math.pi
        n = torch.ceil(self.cfg.turn_ticks * dpsi.abs() / math.pi).long()
        self.turning = torch.where(m, n, self.turning)
        self.turn_rate = torch.where(m, dpsi.abs() / n.clamp(min=1), self.turn_rate)
        err = (self._rand(*m.shape) * 2 - 1) * math.radians(self.cfg.revisit_yaw_err_deg)
        self.heading = torch.where(m, self.goal_yaw + err, self.heading)
        self.goal_yaw = torch.where(m, torch.full_like(self.goal_yaw, float('nan')), self.goal_yaw)

    def step(self, actions):
        """actions: (B, N) long candidate indices. Returns obs, reward (B,), done (B,), info."""
        c, B, N, H, W = self.cfg, self.B, self.N, self.H, self.W
        a = actions.clamp(0, c.n_candidates - 1)
        valid = torch.gather(self.cand_mask, 2, a[..., None]).squeeze(-1)
        a = torch.where(valid, a, torch.zeros_like(a))
        new = torch.gather(self.cand, 2, a[..., None, None].expand(B, N, 1, 2)).squeeze(2)
        n_fixed = self.n_fixed()                 # keep (+ wait): slots that set no new goal
        go = a >= n_fixed
        self.waiting = (a == 1) if n_fixed == 2 else torch.zeros_like(go)
        change = go | (~self.has_goal & ~self.waiting)
        self.goal = torch.where(change[..., None], new, self.goal)
        self.has_goal = self.has_goal | go
        here = go & (self.goal == self.pos).all(-1)                        # a new goal where the robot stands
        if self.poses:
            yaw = torch.gather(self.cand_yaw, 2, a[..., None]).squeeze(-1)
            self.goal_yaw = torch.where(go, yaw, torch.where(change, torch.full_like(yaw, float('nan')), self.goal_yaw))
            if c.turn_at_own_frontier:                                     # a frontier here: turn around to look
                back = here & torch.isnan(self.goal_yaw)
                self.goal_yaw = torch.where(back, self.heading + math.pi, self.goal_yaw)
            self._start_turn(here)                                         # already there: just turn
        else:
            yaw = torch.gather(self.cand_yaw, 2, a[..., None]).squeeze(-1)
            self.view_yaw = torch.where(go, yaw, torch.where(change, torch.full_like(yaw, float('nan')), self.view_yaw))
            face = here & ~torch.isnan(self.view_yaw)                     # a viewpoint here: face the unknown
            self.heading = torch.where(face, self.view_yaw, self.heading)
            if c.turn_at_own_frontier:
                flip = here & ~face
                self.heading = torch.where(flip, torch.remainder(self.heading + 2 * math.pi, 2 * math.pi) - math.pi,
                                           self.heading)
        self.has_goal &= ~(self.goal == self.pos).all(-1)
        ch = (change & self.has_goal).view(-1).nonzero().squeeze(1)
        if ch.numel():
            src = torch.zeros(ch.numel(), H, W, dtype=torch.bool, device=self.dev)
            g = self.goal.view(-1, 2)[ch]
            src[torch.arange(ch.numel(), device=self.dev), g[:, 0], g[:, 1]] = True
            stand = self.stand[:, None].expand(B, N, H, W).reshape(-1, H, W)[ch]
            self.goal_field.view(-1, H, W)[ch] = G.distance_field(stand, src)
        old_tc, old_kc = self.team_cov.clone(), self.know_cov.clone()
        if c.slam_model:
            old_merged = self.merged.clone()
            old_gap = self._known_count(integrated=True) - self._known_count(integrated=False)
            self.step_lost.zero_()
            self.step_links.zero_()
            self.step_drift.zero_()
        if self.graph:
            self.step_conn.zero_()
            self.step_merge.zero_()
            self.step_unc.zero_()
        for _ in range(c.ticks):
            moving, steplen, turn = self._drive()
            new_cells = self._sense()
            if c.slam_model:
                self._slam_tick(moving, steplen, turn, new_cells)
        self._merge()
        if c.slam_model:
            # map data arriving from teammates, and the place-recognition burst of a new merge
            gap = self._known_count(integrated=True) - self._known_count(integrated=False)
            imported = (gap - old_gap).clamp(min=0) / self.ig_norm
            new_merges = (self.merged & ~old_merged).sum(2).float()
            self.backlog = (self.backlog + c.work_import * imported + c.work_merge * new_merges).clamp(max=c.backlog_max)
        self.team_cov, self.know_cov = self._coverage()
        self.t += 1
        hit90 = (self.t90 < 0) & (self.team_cov >= 0.9)
        self.t90 = torch.where(hit90, self.t, self.t90)
        success = self.team_cov >= c.success
        done = success | (self.t >= c.max_steps)
        reward = (c.w_team * (self.team_cov - old_tc) + c.w_know * (self.know_cov - old_kc)
                  - c.time_cost + c.success_bonus * success.float())
        if c.slam_model:
            drift_term = self.drift.mean(1) if c.drift_penalty == 'level' else self.step_drift / N
            reward = (reward - c.w_drift * drift_term - c.w_lost * self.step_lost / (N * c.ticks)
                      + c.w_link * self.step_links)
        if self.graph:
            reward = reward + c.w_conn * self.step_conn + c.w_merge * self.step_merge + c.w_unc * self.step_unc
        info = {'done_team_cov': self.team_cov[done], 'done_know_cov': self.know_cov[done],
                'done_steps': self.t[done], 'done_t90': self.t90[done], 'done_success': success[done],
                'done_dist': self.dist[done].mean(1), 'done_merged': self.merged[done].float().mean((1, 2)),
                'done_map': self.map_idx[done]}
        if c.slam_model:
            info.update({'done_drift': self.drift[done].mean(1), 'done_links': self.links[done].sum((1, 2)),
                         'done_backlog': self.backlog[done].mean(1)})
        if self.graph:
            mate = self.merged[done] & ~torch.eye(N, dtype=torch.bool, device=self.dev)
            err = self.merge_err()[done]
            info.update({'done_lam2': self.lam2[done].mean(1),
                         'done_merge_err': (err * mate).sum((1, 2)) / mate.sum((1, 2)).clamp(min=1)})
        self._reset(done)
        return self.observe(), reward, done, info

    def n_fixed(self):
        """Candidate slots that set no new goal: keep (0) and, in stage 2, wait (1)."""
        return 2 if (self.cfg.slam_model and self.cfg.wait_action) else 1

    def _known_count(self, integrated):
        if integrated:
            f, o = self.integrated()
            return (f | o).flatten(2).sum(2).float()
        return (self.seen_free | self.seen_occ).flatten(2).sum(2).float()

    def _slam_tick(self, moving, steplen, turn, new_cells):
        """One tick of the SLAM and compute model (stage 2), after driving and sensing."""
        c, B, N, d = self.cfg, self.B, self.N, self.dev
        mv = moving.float()
        lost_now = self.lost > 0
        # work in, work done
        work = c.work_move * mv + c.work_new * new_cells / self.ig_norm
        self.backlog = (self.backlog + work - c.capacity).clamp(0, c.backlog_max)
        lag = c.lag0 + self.backlog
        # tracking loss while moving
        p = c.lost_p0 + c.lost_lag * torch.sigmoid((lag - c.lag_crit) / c.lag_scale) + c.lost_turn * turn / math.pi
        newly = moving & ~lost_now & (self._rand(B, N) < p)
        self.lost = torch.where(newly, torch.full_like(self.lost, c.lost_ticks), (self.lost - 1).clamp(min=0))
        self.step_lost += (self.lost > 0).float().sum(1)
        # drift
        metres = 0.1 * steplen
        self.drift = self.drift + c.drift_rate * metres * (1 + c.drift_lag * lag) + c.lost_drift * newly.float()
        drift_before = self.drift
        avoidable = c.drift_rate * metres * c.drift_lag * self.backlog + c.lost_drift * newly.float()
        self.since_corr = self.since_corr + metres
        # loop closure: back on ground this robot saw long ago
        bi = torch.arange(B, device=d)[:, None].expand(B, N)
        ni = torch.arange(N, device=d)[None].expand(B, N)
        py, px = self.pos[..., 0], self.pos[..., 1]
        fs_here = self.first_seen[bi, ni, py, px]                                  # (B, N) my first look at my cell
        tick = self.tick[:, None]
        ok = ~(self.lost > 0)
        if c.loop_model == 'views':
            # seeing enough of its own old views again, from a similar direction: ORB-SLAM3 place recognition
            loop = (ok & moving & (self.old_views >= c.loop_views) & (tick - self.last_loop >= c.loop_cooldown)
                    & (self._rand(B, N) < c.loop_rate * c.tick_s))
        else:
            loop = (ok & moving & (fs_here >= 0) & (tick - fs_here >= c.loop_age)
                    & (tick - self.last_loop >= c.loop_cooldown) & (self._rand(B, N) < c.p_loop))
        cut = torch.full_like(self.drift, c.loop_gain)
        if self.graph and c.loop_model == 'views' and loop.any():
            cut = self._loop_edges(loop)                                   # share of the uncertainty removed
        self.drift = torch.where(loop, self.drift * (1 - cut), self.drift)
        self.last_loop = torch.where(loop, tick.expand(B, N), self.last_loop)
        mate = self.merged & ~torch.eye(N, dtype=torch.bool, device=d)
        their_tick = mine_tick = None
        if self.views:
            # links: at the SLAM's verification pace, while the views both maps hold allow more
            seen_v = (self.first_view >= 0).view(B, N, -1).float()
            shared = torch.bmm(seen_v, seen_v.transpose(1, 2))                      # (B, i, j)
            rate = torch.where(c.merge_n0 + self.links < c.link_strong, c.link_rate_fast, c.link_rate_slow)
            link = (mate & ok[..., None] & (shared >= c.link_views * (self.links + 1))
                    & (self._rand(B, N, N) < rate * c.tick_s))
            if self.graph and link.any():
                mine_tick, their_tick = self._shared_view_ticks(link)
        elif c.pose_revisit:
            # links: near a merged teammate's past pose, looking the way it looked
            q, their_tick = self._view_match()
            link = (mate & ok[..., None] & moving[..., None] & (tick[..., None] - self.last_link >= c.link_cooldown)
                    & (self._rand(B, N, N) < c.p_link_pose * q))
        else:
            # links: on ground a merged teammate saw first
            fs_mate = self.first_seen[bi[..., None].expand(B, N, N), torch.arange(N, device=d)[None, None].expand(B, N, N),
                                      py[..., None].expand(B, N, N), px[..., None].expand(B, N, N)]   # (B, i, j)
            link = (mate & ok[..., None] & (fs_mate >= 0) & ((fs_here[..., None] < 0) | (fs_mate < fs_here[..., None]))
                    & (tick[..., None] - self.last_link >= c.link_cooldown) & (self._rand(B, N, N) < c.p_link))
        if self.graph and link.any():
            self._link_edges(link, their_tick, mine_tick)
        any_link = link.any(2)
        best = torch.where(link, self.drift[:, None, :], torch.full((), 1e9, device=d)).amin(2)
        self.drift = torch.where(any_link, self.drift - c.link_gain * (self.drift - best).clamp(min=0), self.drift)
        self.step_links += (link & (self.links < c.link_cap)).float().sum((1, 2))
        self.links = self.links + link.float()
        self.last_link = torch.where(link, tick[..., None].expand(B, N, N), self.last_link)
        self.since_corr = torch.where(loop | any_link, torch.zeros_like(self.since_corr), self.since_corr)
        self.step_drift += (avoidable - (drift_before - self.drift)).sum(1)
        self.tick += 1

    def _view_match(self):
        """How well each robot's current view matches each teammate's past views
        nearby: (B, i, j) in 0..1 (distance and heading falloff, best visit within
        link_radius cells) and the tick of that visit."""
        c, B, N, H, W, d = self.cfg, self.B, self.N, self.H, self.W, self.dev
        r, fov = c.link_radius, math.radians(c.link_fov_deg)
        bi = torch.arange(B, device=d)[:, None, None].expand(B, N, N)
        ji = torch.arange(N, device=d)[None, None, :].expand(B, N, N)
        py, px = self.pos[..., 0][..., None].expand(B, N, N), self.pos[..., 1][..., None].expand(B, N, N)
        best_q = torch.zeros(B, N, N, device=d)
        best_t = torch.full((B, N, N), -1, dtype=torch.int32, device=d)
        for dy in range(-r, r + 1):
            for dx in range(-r, r + 1):
                dist = math.hypot(dy, dx)
                if dist > r:
                    continue
                y, x = py + dy, px + dx
                inb = (y >= 0) & (y < H) & (x >= 0) & (x < W)
                y, x = y.clamp(0, H - 1), x.clamp(0, W - 1)
                tt = self.trail_tick[bi, ji, y, x]
                dpsi = torch.remainder(self.heading[..., None] - self.trail_head[bi, ji, y, x] + math.pi,
                                       2 * math.pi) - math.pi
                q = (1 - dist / (r + 1)) * (1 - dpsi.abs() / fov).clamp(min=0) * (inb & (tt >= 0)).float()
                better = q > best_q
                best_q = torch.where(better, q, best_q)
                best_t = torch.where(better, tt, best_t)
        return best_q, best_t

    def _merr(self, links):
        """Merge error (m) of an alignment with `links` links beyond the merge match."""
        c = self.cfg
        return torch.sqrt(c.merge_floor ** 2 + c.merge_sigma0 ** 2 / (c.merge_n0 + links))

    def merge_err0(self):
        """Merge error (m) right after the merge: the unit of the merge features and reward."""
        c = self.cfg
        return math.sqrt(c.merge_floor ** 2 + c.merge_sigma0 ** 2 / c.merge_n0)

    def merge_err(self):
        """(B, i, j) merge error (m) of robot i's alignment with j: the merge match plus i's links with j."""
        return self._merr(self.links)

    def merge_gain(self):
        """(B, i, j) merge error one more link would remove, in units of merge_err0()."""
        return (self._merr(self.links) - self._merr(self.links + 1)) / self.merge_err0()

    def _loop_edges(self, loop):
        """Loop closures go into each robot's own graph, from its newest node (as of the
        last decision) to the node of the oldest own view in sight; the uncertainty they
        remove is rewarded. Returns (B, N): the share of the uncertainty removed."""
        c, M, N = self.cfg, self.M, self.N
        b, i = loop.nonzero(as_tuple=True)
        seen = self.obs_node[b]
        now = torch.minimum((self.tick[b] // c.node_ticks).clamp(max=M - 1), seen)
        then = torch.minimum((self.old_view_tick[b, i] // c.node_ticks).clamp(0, M - 1), seen)
        du = torch.where(then < now, self.loop_node_gain[b, i, then], torch.zeros_like(now, dtype=torch.float))
        self.step_unc.index_add_(0, b, du)
        share = torch.zeros(self.B, N, device=self.dev)
        share[b, i] = (du / self.unc[b, i].clamp(min=1e-6)).clamp(0, 0.9)
        mine, theirs = i * M + now, i * M + then
        real = then < now
        b, mine, theirs, i = b[real], mine[real], theirs[real], i[real]
        self.gedge[b, i, mine, theirs] += 1.0
        self.gedge[b, i, theirs, mine] += 1.0
        return share

    def _shared_view_ticks(self, link):
        """For each new link (B, i, j): a random view both robots hold, and the
        ticks each first had it. Returns (mine, theirs), (B, N, N) int (0 elsewhere)."""
        B, N, d = self.B, self.N, self.dev
        b, i, j = link.nonzero(as_tuple=True)
        fi, fj = self.first_view[b, i].view(b.numel(), -1), self.first_view[b, j].view(b.numel(), -1)
        pick = torch.multinomial(((fi >= 0) & (fj >= 0)).float(), 1, generator=self.gen)
        mine = torch.zeros(B, N, N, dtype=torch.long, device=d)
        theirs = torch.zeros_like(mine)
        mine[b, i, j] = fi.gather(1, pick).squeeze(1).long()
        theirs[b, i, j] = fj.gather(1, pick).squeeze(1).long()
        return mine, theirs

    def _link_edges(self, link, their_tick, mine_tick=None):
        """New links go into the finder's graph (from its node at mine_tick, default
        now, to the teammate's node at their_tick); reward the connectivity gain of
        that edge (exact, in the graph as of this decision: newer nodes count as
        the newest one then) and the merge error removed."""
        c, M = self.cfg, self.M
        b, i, j = link.nonzero(as_tuple=True)
        seen = self.obs_node[b]                          # nodes newer than the last observe() are not in the gains
        now = self.tick[b] if mine_tick is None else mine_tick[b, i, j]
        now = torch.minimum((now // c.node_ticks).clamp(max=M - 1), seen)
        then = torch.minimum((their_tick[b, i, j].long() // c.node_ticks).clamp(0, M - 1), seen)
        mine, theirs = i * M + now, j * M + then
        self.step_conn.index_add_(0, b, self._edge_gain(b, i, mine, theirs))
        self.step_merge.index_add_(0, b, self.merge_gain()[b, i, j])
        self.gedge[b, i, mine, theirs] += 1.0
        self.gedge[b, i, theirs, mine] += 1.0

    def _graph_eigs(self):
        """lambda2 of each robot's own pose graph (B, N), and the relative gain
        lambda2'/lambda2 - 1 of one more edge from its newest node to each team
        node (B, N, N*M). Nodes it does not know (unmerged robots, future nodes)
        are cut off with a large diagonal, so the graph's own eigenvalues come
        first. The gain is exact for one edge: adding w (e_a - e_b)(e_a - e_b)^T
        moves lambda2 to the root in (lambda2, lambda3) of the secular equation
        1 + w sum_k z_k^2 / (lambda_k - mu) = 0, z = V^T (e_a - e_b) (bisection)."""
        B, N, M, d = self.B, self.N, self.M, self.dev
        NM = N * M
        cur = (self.tick // self.cfg.node_ticks).clamp(max=M - 1)
        self.obs_node = cur
        node_ok = torch.arange(M, device=d)[None] <= cur[:, None]                       # (B, M)
        allowed = (self.merged[..., None] & node_ok[:, None, None]).reshape(B, N, NM)
        al = allowed.float()
        A = self.chain * al[..., :, None] * al[..., None, :] + self.gedge.clamp(max=3.0)
        L = torch.diag_embed(A.sum(-1) + 50.0 * (1 - al)) - A
        ev, V = torch.linalg.eigh(L.view(B * N, NM, NM))
        two = allowed.sum(-1).view(-1) >= 2
        lam2 = torch.where(two, ev[:, 1], torch.zeros_like(ev[:, 1])).clamp(min=0)
        self.lam2 = lam2.view(B, N)
        a = (torch.arange(N, device=d)[None] * M + cur[:, None]).view(-1)            # my newest node
        Z = V[torch.arange(B * N, device=d), a][:, None, :] - V                       # (BN, b, k): z for edge a-b
        ok = two & (lam2 > 1e-7) & (ev[:, 2] - ev[:, 1] > 1e-7)
        gain = self._secular_gain(ev[:, None, :], Z ** 2, lam2[:, None])
        self.node_gain = torch.where(ok[:, None] & allowed.view(B * N, NM), gain, torch.zeros_like(gain)).view(B, N, NM)
        if self.views:      # kept for the gain of links from older nodes (_edge_gain)
            self._eig = (ev, V, lam2, ok, allowed.view(B * N, NM))
        # pose uncertainty: effective resistance between my first node (the anchor) and my newest node,
        # R = (e_0 - e_n)^T L+ (e_0 - e_n): one per node along a plain path (it grows with every keyframe),
        # less when loops and links give parallel paths; plus the part of the newest node driven so far.
        # A loop edge newest-c (weight 1) removes r^2 / (1 + R_nc), r = (e_0 - e_n)^T L+ (e_n - e_c).
        r_ = torch.arange(B * N, device=d)
        inv = torch.where((ev > 1e-6) & (ev < 25.0), 1.0 / ev.clamp(min=1e-6), torch.zeros_like(ev))   # L+ spectrum
        anchor = (torch.arange(N, device=d)[None] * M).expand(B, N).reshape(-1)
        za = V[r_, anchor] - V[r_, a]                                                   # (BN, K)
        frac = ((self.tick % self.cfg.node_ticks).float() / self.cfg.node_ticks)[:, None].expand(B, N)
        self.unc = (za ** 2 * inv).sum(-1).view(B, N) + frac
        if self.cfg.n_self_revisit:
            own = (torch.arange(NM, device=d) // M)[None] == torch.arange(N, device=d).repeat(B)[:, None]   # (BN, NM)
            rr = (Z * (za * inv)[:, None, :]).sum(-1)                                    # (BN, NM)
            Rnc = (Z ** 2 * inv[:, None, :]).sum(-1)
            gain = rr ** 2 / (1.0 + Rnc)
            gain = torch.where(own & allowed.view(B * N, NM), gain, torch.zeros_like(gain))
            self.loop_node_gain = gain.view(B, N, N, M)[torch.arange(B, device=d)[:, None],
                                                        torch.arange(N, device=d)[None], torch.arange(N, device=d)[None]]

    def _secular_gain(self, ev, Z2, lam2):
        """Relative lambda2 gain of one edge: ev (.., NM) eigenvalues, Z2 (.., NM)
        squared z = V^T (e_a - e_b), lam2 (..); bisection on (lambda2, lambda3)."""
        lo = ev[..., 1].expand(Z2.shape[:-1]).clone()
        hi = ev[..., 2].expand(Z2.shape[:-1]).clone()
        for _ in range(30):
            mu = (lo + hi) / 2
            f = 1 + (Z2 / (ev - mu[..., None])).sum(-1)
            neg = f < 0                       # f rises from -inf (just above lambda2) to +inf (below lambda3)
            lo = torch.where(neg, mu, lo)
            hi = torch.where(neg, hi, mu)
        return ((lo + hi) / 2 / lam2.clamp(min=1e-9) - 1).clamp(0, self.cfg.conn_cap)

    def _edge_gain(self, b, i, na, nb):
        """Relative lambda2 gain of an edge between team nodes na and nb in robot
        i's graph of env b (all (L,)), as of the last observe(). Without the
        views link model only edges from the newest node occur: node_gain."""
        if not self.views:
            return self.node_gain[b, i, nb]
        ev, V, lam2, ok, allowed = self._eig
        r = b * self.N + i
        z2 = (V[r, na] - V[r, nb]) ** 2                                              # (L, NM)
        g = self._secular_gain(ev[r], z2, lam2[r])
        return torch.where(ok[r] & allowed[r, na] & allowed[r, nb], g, torch.zeros_like(g))

    def _own_revisit_poses(self, stand_known, Dr, s0):
        """REVISIT-OWN slots s0..: the robot's own poses driven loop_age ticks ago or earlier,
        ranked by the pose uncertainty a loop there would remove, for the path. Fills
        cand_yaw (the heading it had there). Returns cells (B, N, n, 2), valid, gain (B, N, n)."""
        c, B, N, H, W, d, M = self.cfg, self.B, self.N, self.H, self.W, self.dev, self.M
        n, HW = c.n_self_revisit, H * W
        tt = self.trail_tick.view(B, N, HW).long()                                  # my own last visit per cell
        old = (tt >= 0) & (self.tick[:, None, None] - tt >= c.loop_age)
        node = (tt // c.node_ticks).clamp(0, M - 1)
        gain = self.loop_node_gain.gather(2, node)                                 # (B, N, HW)
        ok = (old & (stand_known & (Dr < G.INF)).view(B, N, HW))
        score = torch.where(ok, (0.02 + gain) / (1 + Dr.view(B, N, HW) / 20.0), torch.zeros((), device=d))
        cells, cok = G.top_candidates(score.view(-1, H, W), n, c.revisit_nms)
        cells, cok = cells.view(B, N, n, 2), cok.view(B, N, n)
        flat = cells[..., 0] * W + cells[..., 1]
        yaw = self.trail_head.view(B, N, HW).gather(2, flat)
        self.cand_yaw[..., s0:s0 + n] = torch.where(cok, yaw, torch.full((), float('nan'), device=d))
        return cells, cok, gain.gather(2, flat) * cok

    @staticmethod
    def _lam2_feat(lam2):
        return ((torch.log10(lam2 + 1e-6) + 6) / 6).clamp(0, 1.5)

    def _revisit_poses(self, mate, stand_known, Dr):
        """Revisit slots: merged teammates' past poses (cell + heading). With the
        graph model, ranked by gain (connectivity + merge accuracy) for the path;
        else nearest first. Fills self.cand_yaw for these slots. Returns cells
        (B, N, n, 2), valid (B, N, n), connectivity and merge gain (B, N, n) or None."""
        c, B, N, H, W, d = self.cfg, self.B, self.N, self.H, self.W, self.dev
        n_rev, n0, HW = c.n_revisit, self.n_fixed(), H * W
        tt = self.trail_tick.view(B, 1, N, HW)
        visited = (tt >= 0) & mate[..., None]                                      # (B, i, j, HW)
        conn = mg = None
        if self.graph:
            M = self.M
            node = (tt.long() // c.node_ticks).clamp(0, M - 1) + (torch.arange(N, device=d) * M).view(1, 1, N, 1)
            conn = self.node_gain[:, :, None, :].expand(B, N, N, N * M).gather(3, node.expand(B, N, N, HW))
            mg = self.merge_gain()                                                    # (B, i, j)
            val = conn / c.conn_cap + mg[..., None]
        else:
            val = torch.ones(B, N, N, HW, device=d)
        val = torch.where(visited, val, torch.full((), -1.0, device=d))
        best, jstar = val.max(2)                                                   # (B, i, HW)
        ok = (best >= 0).view(B, N, H, W) & stand_known & (Dr < G.INF)
        if self.graph:
            score = torch.where(ok, (0.05 + best.view(B, N, H, W)) / (1 + Dr / 20.0), torch.zeros((), device=d))
        else:
            score = torch.where(ok, 2.0 * (H + W) - Dr, torch.zeros((), device=d))
        cells, rok = G.top_candidates(score.view(-1, H, W), n_rev, c.revisit_nms)
        cells, rok = cells.view(B, N, n_rev, 2), rok.view(B, N, n_rev)
        flat = cells[..., 0] * W + cells[..., 1]                                    # (B, i, n)
        j = jstar.gather(2, flat)
        head = self.trail_head.view(B, N, HW)
        yaw = head[torch.arange(B, device=d)[:, None, None], j, flat]
        self.cand_yaw[..., n0:n0 + n_rev] = torch.where(rok, yaw, torch.full((), float('nan'), device=d))
        self.cand_mate[..., n0:n0 + n_rev] = torch.where(rok, j, torch.full((), -1, dtype=torch.long, device=d))
        if not self.graph:
            return cells, rok, None, None
        rc = conn.gather(3, flat[:, :, None, :].expand(B, N, N, n_rev)).gather(2, j[:, :, None]).squeeze(2)
        rm = mg.gather(2, j)
        return cells, rok, rc * rok, rm * rok

    # ── observations ─────────────────────────────────────────────────────
    def observe(self):
        """Per-robot observations (only what the robot knows) and the critic's
        global state. Also fills self.cand / self.cand_mask for step()."""
        c, B, N, H, W, d = self.cfg, self.B, self.N, self.H, self.W, self.dev
        if self.graph:
            self._graph_eigs()
        free, occ = self.integrated()
        unknown = ~(free | occ)
        stand_known = ~G.inflate(occ.view(B * N, H, W), c.inflate).view(B, N, H, W) & ~occ
        fr = G.frontiers(free.view(-1, H, W), unknown.view(-1, H, W), stand_known.view(-1, H, W)).view(B, N, H, W)
        ig = G.info_gain(unknown.view(-1, H, W), self.ig_kernel).view(B, N, H, W)
        # optimistic path distance from each robot: unknown counts as passable
        src = torch.zeros(B * N, H, W, dtype=torch.bool, device=d)
        p = self.pos.view(-1, 2)
        src[torch.arange(B * N, device=d), p[:, 0], p[:, 1]] = True
        Dr = G.distance_field((stand_known | unknown).view(-1, H, W), src).view(B, N, H, W)
        mate = self.merged & ~torch.eye(N, dtype=torch.bool, device=d)              # (B, N, N) i knows j
        n_fixed = self.n_fixed()
        n_rev = c.n_revisit if c.slam_model else 0
        n_own = c.n_self_revisit if self.graph else 0
        score = torch.where(fr & (Dr < G.INF), ig, torch.zeros((), device=d))
        cells, ok = G.top_candidates(score.view(-1, H, W), c.n_candidates - n_fixed - n_rev - n_own, c.nms_radius)
        cells, ok = cells.view(B, N, -1, 2), ok.view(B, N, -1)
        vyaw = None
        if c.goal_standoff > 0:
            # stand off: walk back from the frontier towards the robot (down its path distance), each step to the
            # neighbour farthest from unknown space, until goal_standoff cells from any unknown cell, on known
            # free ground: the goal is there, on the way to the frontier, so the camera looks at the edge of
            # the unknown instead of driving into it. No heading is imposed (the robot arrives as it drives:
            # a car told to face a set heading at the goal manoeuvres on the spot; Isaac, 13 October). If no
            # such cell is near the path (or the robot is already that close), the frontier stays the goal.
            Du = G.distance_field(free.view(-1, H, W), unknown.view(-1, H, W)).view(B, N, H, W)
            Pd = F.pad(Dr.view(B * N, H, W), (1, 1, 1, 1), value=G.INF).view(B, N, H + 2, W + 2)
            bi_ = torch.arange(B, device=d)[:, None, None].expand_as(cells[..., 0])
            ni_ = torch.arange(N, device=d)[None, :, None].expand_as(cells[..., 0])
            frontier, v = cells.clone(), cells.clone()
            done_ = torch.zeros_like(ok)
            for _ in range(2 * c.goal_standoff):
                here_d = Dr[bi_, ni_, v[..., 0], v[..., 1]]
                best = torch.full_like(here_d, -1.0)
                nxt = v.clone()
                for dy, dx, _ in G._SHIFTS:
                    yy, xx = v[..., 0] + dy, v[..., 1] + dx
                    dv = Pd[bi_, ni_, yy + 1, xx + 1]
                    yy, xx = yy.clamp(0, H - 1), xx.clamp(0, W - 1)
                    du = Du[bi_, ni_, yy, xx]
                    sc = du.clamp(max=c.goal_standoff) * 1000.0 - dv
                    good = (dv < here_d) & free[bi_, ni_, yy, xx] & stand_known[bi_, ni_, yy, xx] & (sc > best)
                    best = torch.where(good, sc, best)
                    nxt = torch.where(good[..., None], torch.stack([yy, xx], -1), nxt)
                v = torch.where(done_[..., None], v, nxt)
                done_ = done_ | (Du[bi_, ni_, v[..., 0], v[..., 1]] >= c.goal_standoff)
            moved = done_ & (v != frontier).any(-1) & ok
            cells = torch.where(moved[..., None], v, frontier)
            vyaw = (moved, frontier)
        keep = torch.where(self.has_goal[..., None], self.goal, self.pos)[:, :, None]
        fixed = [keep] + ([self.pos[:, :, None]] if n_fixed == 2 else [])                # keep, wait (stay here)
        rev_conn = rev_merge = None
        self.cand_yaw.fill_(float('nan'))
        self.cand_mate.fill_(-1)
        if n_rev and c.pose_revisit:
            rcells, rok, rev_conn, rev_merge = self._revisit_poses(mate, stand_known, Dr)
            fixed.append(rcells)
        elif n_rev:
            # revisit: places merged teammates drove through (where links form), nearest first
            mates_trail = torch.einsum('bnm,bmk->bnk', mate.float(), self.trail.flatten(2).float()).view(B, N, H, W) > 0
            rev_ok = mates_trail & ~self.trail & stand_known & (Dr < G.INF)
            rscore = torch.where(rev_ok, 2.0 * (H + W) - Dr, torch.zeros((), device=d))
            rcells, rok = G.top_candidates(rscore.view(-1, H, W), n_rev, c.revisit_nms)
            fixed.append(rcells.view(B, N, -1, 2))
            rok = rok.view(B, N, -1)
        own_gain = None
        if n_own:
            ocells, ook, own_gain = self._own_revisit_poses(stand_known, Dr, n_fixed + n_rev)
            fixed.append(ocells)
        self.cand = torch.cat(fixed + [cells], 2)
        nfr = cells.shape[2]                     # which frontier slots stand off, and their frontier cells
        self.cand_stood = torch.zeros(B, N, c.n_candidates, dtype=torch.bool, device=d)
        self.cand_front = self.cand.clone()
        if vyaw is not None:
            self.cand_stood[..., -nfr:], self.cand_front[..., -nfr:, :] = vyaw
        self.cand_mask = torch.cat([torch.ones(B, N, n_fixed, dtype=torch.bool, device=d)]
                                   + ([rok] if n_rev else []) + ([ook] if n_own else []) + [ok], 2)
        K = c.n_candidates
        # the heading a robot arrives with: the direction of the last step of its shortest path to the goal
        # (the live planner gives it to Nav2, so the car does not turn on the spot at a goal)
        P = F.pad(Dr.view(B * N, H, W), (1, 1, 1, 1), value=G.INF).view(B, N, H + 2, W + 2)
        cy0, cx0 = self.cand[..., 0], self.cand[..., 1]
        bi0 = torch.arange(B, device=d)[:, None, None].expand(B, N, K)
        ni0 = torch.arange(N, device=d)[None, :, None].expand(B, N, K)
        best = torch.full((B, N, K), float(G.INF), device=d)
        prev = torch.zeros(B, N, K, 2, device=d)
        for dy, dx, _ in G._SHIFTS:
            v = P[bi0, ni0, cy0 + 1 + dy, cx0 + 1 + dx]
            better = v < best
            best = torch.where(better, v, best)
            prev = torch.where(better[..., None], torch.tensor([float(dy), float(dx)], device=d), prev)
        self.cand_arrive = torch.atan2(-prev[..., 0], -prev[..., 1])                 # from the previous cell to the goal
        # candidate features
        rel = (self.cand - self.pos[:, :, None]).float()
        cy, cx = self.cand[..., 0], self.cand[..., 1]
        bi = torch.arange(B, device=d)[:, None, None].expand(B, N, K)
        ni = torch.arange(N, device=d)[None, :, None].expand(B, N, K)
        path = Dr[bi, ni, cy, cx].clamp(max=2 * (H + W))
        gain = ig[bi, ni, cy, cx] / self.ig_norm
        gpos = self.goal.float()
        ppos = self.pos.float()
        dgoal = torch.cdist(self.cand.float().view(B * N, K, 2),
                            gpos[:, None].expand(B, N, N, 2).reshape(B * N, N, 2)).view(B, N, K, N)
        dpos = torch.cdist(self.cand.float().view(B * N, K, 2),
                           ppos[:, None].expand(B, N, N, 2).reshape(B * N, N, 2)).view(B, N, K, N)
        far = float(H + W)
        dgoal = torch.where(mate[:, :, None] & self.has_goal[:, None, None], dgoal, torch.full((), far, device=d)).amin(-1)
        dpos = torch.where(mate[:, :, None], dpos, torch.full((), far, device=d)).amin(-1)
        is_keep = torch.zeros(B, N, K, device=d)
        is_keep[..., 0] = self.has_goal.float()
        feats = [rel[..., 0] / H, rel[..., 1] / W, path / far, rel.norm(dim=-1) / far, gain, is_keep, dgoal / far, dpos / far]
        if c.slam_model:
            is_wait = torch.zeros(B, N, K, device=d)
            is_rev = torch.zeros(B, N, K, device=d)
            if n_fixed == 2:
                is_wait[..., 1] = 1
            is_rev[..., n_fixed:n_fixed + n_rev] = 1
            feats += [is_wait, is_rev]
        if self.poses and c.pose_revisit:
            # heading change at the target against the way the robot will arrive (straight there), and the turn
            travel = torch.atan2(rel[..., 0], rel[..., 1])
            dpsi = torch.remainder(self.cand_yaw - travel + math.pi, 2 * math.pi) - math.pi
            has = ~torch.isnan(dpsi)
            dpsi = torch.where(has, dpsi, torch.zeros_like(dpsi))
            feats += [torch.cos(dpsi) * has, torch.sin(dpsi) * has, dpsi.abs() / math.pi]
        if self.graph:
            cg = torch.zeros(B, N, K, device=d)
            mg = torch.zeros(B, N, K, device=d)
            if rev_conn is not None:
                cg[..., n_fixed:n_fixed + n_rev] = rev_conn / c.conn_cap
                mg[..., n_fixed:n_fixed + n_rev] = rev_merge
            feats += [cg, mg]
        if n_own:
            is_own = torch.zeros(B, N, K, device=d)
            ug = torch.zeros(B, N, K, device=d)
            is_own[..., n_fixed + n_rev:n_fixed + n_rev + n_own] = 1
            ug[..., n_fixed + n_rev:n_fixed + n_rev + n_own] = (own_gain / 2.0).clamp(max=2)
            feats += [is_own, ug]
        cand_feat = torch.stack(feats, -1)
        # teammate features (known only once merged)
        rel_p = (ppos[:, None] - ppos[:, :, None])                                 # (B, N, N, 2): j relative to i
        rel_g = (gpos[:, None] - ppos[:, :, None])
        mf = mate.float()[..., None]
        mparts = [mate.float()[..., None], rel_p[..., :1] / H * mf, rel_p[..., 1:] / W * mf,
                  rel_g[..., :1] / H * mf, rel_g[..., 1:] / W * mf,
                  (rel_p.norm(dim=-1, keepdim=True) / far) * mf + (1 - mf)]
        if c.slam_model:     # graph links with that teammate; its distance since a map correction
            mparts += [(self.links / c.link_cap).clamp(max=1)[..., None] * mf,
                       (self.since_corr[:, None, :, None].expand(B, N, N, 1) / 5.0).clamp(max=2) * mf]
        if self.graph:       # how well my map is aligned with that teammate's (from my links with it)
            mparts.append((self.merge_err() / self.merge_err0())[..., None] * mf)
        mates = torch.cat(mparts, -1)
        off = ~torch.eye(N, dtype=torch.bool, device=d)
        mates = mates[off[None].expand(B, N, N)].view(B, N, N - 1, mates.shape[-1])
        # map planes
        pos_plane = torch.zeros(B, N, H, W, dtype=torch.bool, device=d)
        pos_plane.view(B * N, H * W)[torch.arange(B * N, device=d), p[:, 0] * W + p[:, 1]] = True
        goal_plane = torch.zeros_like(pos_plane)
        g = self.goal.view(-1, 2)
        goal_plane.view(B * N, H * W)[torch.arange(B * N, device=d), g[:, 0] * W + g[:, 1]] = self.has_goal.view(-1)
        mates_pos = torch.einsum('bnm,bmk->bnk', mate.float(), pos_plane.flatten(2).float()).view(B, N, H, W) > 0
        mates_goal = torch.einsum('bnm,bmk->bnk', mate.float(), goal_plane.flatten(2).float()).view(B, N, H, W) > 0
        own_seen = self.seen_free | self.seen_occ
        plist = [free, occ, own_seen, fr, pos_plane, mates_pos, mates_goal, self.trail]
        if c.slam_model:
            plist.append(torch.einsum('bnm,bmk->bnk', mate.float(), self.trail.flatten(2).float()).view(B, N, H, W) > 0)
        planes = torch.stack(plist, 2)
        slist = [(self.t.float() / c.max_steps)[:, None].expand(B, N),
                 ((free | occ).flatten(2).sum(2).float() / (H * W)),
                 mate.float().sum(2) / max(1, N - 1), self.has_goal.float()]
        if c.slam_model:     # what the robot's SLAM node publishes about itself
            slist += [(self.backlog / 3.0).clamp(max=3), (self.lost > 0).float(), self.lost.float() / c.lost_ticks,
                      (self.since_corr / 5.0).clamp(max=2),
                      (self.links.sum(2) / (c.link_cap * max(1, N - 1))).clamp(max=1)]
        if self.graph:       # algebraic connectivity of my own pose graph (log scale)
            slist.append(self._lam2_feat(self.lam2))
        if n_own:            # my pose uncertainty (graph resistance, in path nodes of node_ticks)
            slist.append((self.unc / 10.0).clamp(max=3))
        scalars = torch.stack(slist, -1)
        # critic: global state, once per environment (5 planes); the critic
        # adds each robot's own position plane (planes[:, :, 4]) itself.
        state = torch.stack([self.occ, self.seen_free.any(1), self.seen_occ.any(1),
                             pos_plane.any(1), goal_plane.any(1)], 1)
        sslist = [self.t.float() / c.max_steps, self.team_cov, self.merged.float().mean((1, 2))]
        if c.slam_model:     # the truth: drift is hidden from the robots
            sslist += [(self.drift.mean(1) / 0.5).clamp(max=4), (self.backlog.amax(1) / 3.0).clamp(max=3),
                       (self.lost > 0).float().mean(1), (self.links.clamp(max=c.link_cap).sum((1, 2))
                                                         / (c.link_cap * N * max(1, N - 1)))]
        if self.graph:
            pair = (self.merged & ~torch.eye(N, dtype=torch.bool, device=d)).float()
            sslist += [self._lam2_feat(self.lam2).mean(1),
                       (self.merge_err() / self.merge_err0() * pair).sum((1, 2)) / pair.sum((1, 2)).clamp(min=1)]
        if n_own:
            sslist.append((self.unc.mean(1) / 10.0).clamp(max=3))
        state_scalars = torch.stack(sslist, -1)
        return {'planes': planes.to(torch.uint8), 'cand': cand_feat, 'cand_mask': self.cand_mask,
                'mates': mates, 'scalars': scalars, 'cand_cells': self.cand,
                'state': state.to(torch.uint8), 'state_scalars': state_scalars}
