# Copied from ma_explore (54d3ca2) by scripts/sync_robot_package.sh: edit it there, not here.
"""ROS 2 planner for one robot: picks its next exploration goal and sends it to Nav2.

    python -m ma_explore.ros_planner --ros-args -p agent:=sim_1 -p agents:="sim_1 sim_2 sim_3" \\
        -p policy:=rl:/path/best.pt [-p device:=cpu] [-p out:=DIR]

Decentralized, like on the real robots: a robot uses only its own map and
pose, plus what it can place in its own map frame -- a teammate's map, pose and
goal once the robot's SLAM has merged with that teammate (the merge transform
on /<agent>/orb_slam3/merge_transform). The observation is built exactly as
in training (live.Decider).

Inputs (per robot a; this robot is <agent>):
  /<a>/map                         nav_msgs/OccupancyGrid in <a>/map_nav (SLAM grid)
  /<a>/orb_slam3/odom              nav_msgs/Odometry, base_footprint in <a>/map_nav
  /ma_explore/<a>/goal             geometry_msgs/PointStamped, the goal each planner publishes
                                   (z = 1: has a goal, z = 0: none)
  /<agent>/orb_slam3/merge_transform  T(<agent> map <- other's map), every 2 s while merged
  /<agent>/orb_slam3/slam_metrics  tracking state (index 1; 2 = OK)
  TF <a>/map -> <a>/map_nav        (static, from the floor fit)
Output: NavigateToPose goals on /<agent>/navigate_to_pose, the goal on
/ma_explore/<agent>/goal, each decision on /ma_explore/<agent>/action (JSON)
and the map it decided on on /ma_explore/<agent>/known_map (for the views).

Frames are '<a>/map_nav' etc. (several robots on one /tf: Isaac, or real robots
with run_slam3.sh --ma-frames); -p frame_prefix:=none uses plain 'map_nav' (one
real robot with run_slam3.sh --nav). The policy's team size (the training
config's n_agents) is kept: robots that are not there count as never merged, so
one robot alone explores as a robot does before its first merge.

When it decides (like a training step = 0.8 m of driving):
  * no goal (start, reached, failed), or
  * driven `decide_dist` m since the last decision, or
  * `decide_period` s since the last decision.
Action 0 keeps the current goal (or stays when there is none, then asks again
after `stay_wait` s). A goal Nav2 rejects, aborts or reaches is blocked for
`block_time` s (candidates within 0.3 m of it are masked): a frontier the car
stands on can stay a frontier under a forward camera, and choosing it again
would loop. While tracking is
lost it waits (the robot's safety gate has stopped the car). Each decision is
a row of <out>/<agent>_decisions.csv (kind: keep, stay, wait, revisit, frontier;
mate: whose pose a revisit goes to) and a JSON message on
/ma_explore/<agent>/action (latched) for the live view. No SLAM pose for
pose_timeout s also counts as lost (after a loss ORB-SLAM3 tracks in a
temporary map: tracking 'OK', but no pose until it is back in the main map);
each loss and recovery is a row of <out>/<agent>_slam_events.csv (where, the
heading, the goal and its bearing, how long). /ma_explore/<agent>/enable
(std_msgs/Bool) switches exploring off (the goal is cancelled) and on.
-p policy:= takes 'rl:<file>', a .pt file (checkpoint or robot_policy.pt) or a baseline name.

SLAM-aware policies (stage 2, slam_model) also get what the SLAM node reports
about itself (/<agent>/orb_slam3/slam_metrics 7-13 and .../ma_links): backlog
(frame lag above this robot's usual lag, plus 0.1 s per keyframe waiting for
LocalMapping), tracking lost or not, metres since the last map correction,
graph links per teammate; teammates' distance-since-correction, backlog and
driven paths come from their planners (/ma_explore/<a>/status, odometry).
Their extra action WAIT stops the car where it is (the goal is kept) for one
decision, about `wait_s` seconds.

With pose_revisit, a revisit goal is a merged teammate's past pose: Nav2 gets
the heading the teammate had there (the same view, for place recognition).
With graph_model, the planner keeps its own pose-graph edges with each
teammate as training does: the merge match, then one per new link in ma_links
(minus the count at the merge: the merge match's own measurements), each
between nodes (training ticks since the start: tick_s) where both robots had a
similar view (link_model 'views'), else from its newest node to the teammate's
node where that teammate's path came closest. Unmerged teammates'
paths are kept but not used until the merge transform arrives.
"""
import csv
import json
import math
import os
import time
from collections import deque

import numpy as np
import rclpy
import torch
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PointStamped, TransformStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.action import ActionClient
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Float64MultiArray, String
from tf2_ros import Buffer, TransformListener

from .live import Decider, Grid, Robot, load_policy
from .recovery import BackTrack

TRACKING_OK = 2          # ORB-SLAM3 Tracking::OK


def quat_to_R(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def T_of(tr):
    """geometry_msgs Transform -> 4 x 4."""
    T = np.eye(4)
    T[:3, :3] = quat_to_R(tr.rotation)
    T[:3, 3] = [tr.translation.x, tr.translation.y, tr.translation.z]
    return T


def apply3(T, xy):
    """Points on the floor (z = 0) of one frame -> (x, y) in another (4 x 4)."""
    p = np.c_[xy, np.zeros(len(xy)), np.ones(len(xy))]
    return (p @ T.T)[:, :2]


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class Planner(Node):
    def __init__(self):
        super().__init__('ma_explore_planner')
        dyn = ParameterDescriptor(dynamic_typing=True)          # 300 and 300.0 both fine
        p = lambda n, v: type(v)(self.declare_parameter(n, v, dyn).value)   # noqa: E731
        self.me = p('agent', 'sim_1')
        self.agents = p('agents', 'sim_1 sim_2 sim_3').split()
        policy = p('policy', 'coordinated')
        device = p('device', 'cpu')
        self.decide_dist = p('decide_dist', 0.8)
        self.decide_period = p('decide_period', 8.0)
        self.stay_wait = p('stay_wait', 2.0)
        self.block_time = p('block_time', 60.0)
        self.merge_fresh = p('merge_fresh', 6.0)
        self.wait_s = p('wait_s', 5.4)            # one training decision: 8 ticks x 0.67 s
        # no SLAM pose for this long = tracking lost (after a loss ORB-SLAM3 tracks in a temporary
        # map: its tracking state says OK but no pose comes until it is back in the main map)
        self.pose_timeout = p('pose_timeout', 1.0)
        self.enabled = p('start_enabled', True)   # /ma_explore/<agent>/enable (Bool) switches it
        # recovery after a tracking loss: wait, then back up along the path driven before the loss
        # (wheel odometry, reverse, slowly) so the camera sees again what it saw; recovery.py
        self.rec_on = p('recovery', True)
        self.rec_wait = p('recovery_wait', 2.0)          # s to wait first (SLAM may relocalize by itself)
        self.rec_back = p('recovery_back_m', 0.8)        # m backed up per attempt
        self.rec_speed = p('recovery_speed', 0.1)        # m/s
        self.rec_tries = p('recovery_attempts', 2)
        self.rec_test_at = p('recovery_test_at', -1.0)   # testing: pretend a loss this many s after the start
        out = os.path.expanduser(p('out', '.'))
        self.start_file = os.path.expanduser(p('start_file', ''))
        assert self.me in self.agents
        self.others = [a for a in self.agents if a != self.me]
        # TF frames: '<agent>/map_nav' (several robots on one /tf: Isaac, or real robots started
        # with run_slam3.sh --ma-frames), or plain 'map_nav' (frame_prefix:=none: one real robot)
        self.prefixed = p('frame_prefix', 'agent') != 'none'
        torch.set_num_threads(2)
        pol, env_cfg, H = load_policy(policy, device, p('env_config', ''))   # env_config: baselines only
        if p('prefer_revisit', False):
            pol = self.revisit_first(pol, env_cfg)
        # the policy's team size: robots that are not there count as never merged (as before a merge)
        self.team = max(len(self.agents), env_cfg.n_agents)
        # new goals kept clear of known obstacles (m; Nav2's inflation is 0.25 m for the car): baselines
        # need it on the full stack (else they pick frontier cells by walls that Nav2 cannot reach);
        # -1 = auto: 0.25 m for baselines, off for a trained policy (as in training; 0 failed goals in Isaac)
        gc = p('goal_clearance', -1.0)
        gc = (0.0 if policy.startswith('rl:') else 0.25) if gc < 0 else gc
        self.decider = Decider(pol, env_cfg, self.team, H, device, goal_clearance=math.ceil(round(gc / 0.1, 6)))
        self.policy_name = policy
        # warm up (the first call compiles the path-distance kernel)
        t0 = time.perf_counter()
        ring = np.array([[math.cos(a), math.sin(a)] for a in np.linspace(0, 2 * math.pi, 60)])
        self.decider.decide([Robot(ring * 0.5, ring * 1.5, (0, 0, 0))] + [Robot(known=False)] * (self.team - 1), [], 0)
        self.get_logger().info(f'{self.me}: policy ready ({time.perf_counter() - t0:.1f} s warm-up)')

        self.grid, self.pose, self.goal_of, self.merge = {}, {}, {}, {}
        self.trails = {a: [] for a in self.agents}       # each robot's path, in its own map_nav
        self.tposes = {a: [] for a in self.agents}       # the same as poses (x, y, yaw, tick)
        self.edges = {a: [] for a in self.others}        # my pose-graph edges with each teammate
        self.links_n0 = {}                                # ma_links count at the merge (the merge match)
        self.rng = np.random.default_rng(0)
        self.t_start = None                               # common start (start file): tick 0
        self.status = {}                                  # teammates' (since_corr, backlog)
        self.links = {}                                   # graph links per teammate (ma_links)
        self.lags, self.lag_base = [], None
        self.metrics = [0.0] * 14
        self.paused_until = None
        self.tracking = TRACKING_OK
        self.pose_t = {}                                  # when each robot's SLAM pose last came
        self.lost_since, self.last_kind = None, ''
        self.slam_hist = deque(maxlen=400)                # my recent SLAM poses (wall time, x, y, yaw)
        self.own_edges, self.reobs_s, self.last_loop_t = [], 0.0, -1e9   # own loop edges (re-observation)
        self.corrections = None
        self.last_new_merge_t = -1e9
        self.wheel_hist = deque(maxlen=400)               # my wheel odometry (wall time, x, y, yaw)
        self.rec = None                                   # recovery state while lost
        self.test_lost = False
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        for a in self.agents:
            self.create_subscription(OccupancyGrid, f'/{a}/map', lambda m, a=a: self.on_grid(a, m), latched)
            self.create_subscription(Odometry, f'/{a}/orb_slam3/odom', lambda m, a=a: self.on_odom(a, m), 10)
            if a != self.me:
                self.create_subscription(PointStamped, f'/ma_explore/{a}/goal',
                                         lambda m, a=a: self.on_goal(a, m), latched)
        for a in self.others:
            self.create_subscription(Float64MultiArray, f'/ma_explore/{a}/status',
                                     lambda m, a=a: self.status.__setitem__(a, tuple(m.data[:2])), 10)
        self.create_subscription(String, f'/{self.me}/orb_slam3/ma_links', self.on_links, 10)
        self.status_pub = self.create_publisher(Float64MultiArray, f'/ma_explore/{self.me}/status', 10)
        self.create_subscription(TransformStamped, f'/{self.me}/orb_slam3/merge_transform', self.on_merge, 10)
        self.create_subscription(Float64MultiArray, f'/{self.me}/orb_slam3/slam_metrics', self.on_metrics, 10)
        self.goal_pub = self.create_publisher(PointStamped, f'/ma_explore/{self.me}/goal', latched)
        self.action_pub = self.create_publisher(String, f'/ma_explore/{self.me}/action', latched)
        self.known_pub = self.create_publisher(OccupancyGrid, f'/ma_explore/{self.me}/known_map', latched)
        self.create_subscription(Bool, f'/ma_explore/{self.me}/enable', self.on_enable, latched)
        self.create_subscription(Odometry, f'/{self.me}/odom', self.on_wheel, 20)     # the base driver's wheel odometry
        self.cmd_pub = self.create_publisher(Twist, f'/{self.me}/cmd_vel', 10)      # direct, as the keyboard teleop
        self.tf = Buffer()
        self.tfl = TransformListener(self.tf, self)
        self.nav = ActionClient(self, NavigateToPose, f'/{self.me}/navigate_to_pose')

        self.goal = None              # (x, y) in my map_nav, None = no goal
        self.goal_yaw = None          # heading at a revisit pose, else None
        self.goal_handle = None
        self.goal_seq = 0
        self.blocked = []             # [(x, y, until)]
        self.trail = []
        self.t_decisions = 0
        self.last_xy, self.last_t, self.next_ok = None, 0.0, 0.0
        os.makedirs(out, exist_ok=True)
        self.out = out
        self.ready = False
        self.log_f = open(os.path.join(out, f'{self.me}_decisions.csv'), 'w', newline='')
        # every tracking loss: where, heading, the goal it was driving to (bearing from the heading), how long
        self.ev_f = open(os.path.join(out, f'{self.me}_slam_events.csv'), 'w', newline='')
        self.ev = csv.writer(self.ev_f)
        self.ev.writerow(['wall', 'event', 'duration_s', 'x', 'y', 'yaw', 'goal_x', 'goal_y', 'goal_bearing_deg',
                          'goal_dist_m', 'last_kind', 'decisions'])
        self.log = csv.writer(self.log_f)
        self.log.writerow(['wall', 'decision', 'policy', 'action', 'keep', 'goal_x', 'goal_y', 'x', 'y', 'yaw',
                           'n_candidates', 'merged_with', 'decide_ms', 'reason', 'goal_yaw', 'kind', 'mate'])
        self.create_timer(0.5, self.tick)
        self.create_timer(0.1, self.recovery_tick)
        self.get_logger().info(f'{self.me}: planner "{policy}" on {device}, team {self.agents}')

    @staticmethod
    def revisit_first(pol, cfg):
        """Testing (prefer_revisit): take the first revisit pose whenever one is
        offered, else ask the policy -- to check revisit goals and graph edges on
        the full stack, or to drive calibration runs."""
        n0 = 2 if (cfg.slam_model and cfg.wait_action) else 1
        n_rev = cfg.n_revisit if cfg.slam_model else 0

        def f(obs):
            a = pol(obs)
            if n_rev:
                rv = obs['cand_mask'][..., n0:n0 + n_rev]
                has_goal = obs['cand'][..., 0, 5] > 0.5          # keep a goal until Nav2 ends it
                a = torch.where(has_goal, torch.zeros_like(a), torch.where(rv.any(-1), n0 + rv.float().argmax(-1), a))
            return a
        return f

    # ── inputs ───────────────────────────────────────────────────────────
    def on_grid(self, a, m):
        self.grid[a] = Grid(np.asarray(m.data, np.int8).reshape(m.info.height, m.info.width),
                            m.info.resolution, m.info.origin.position.x, m.info.origin.position.y)

    def on_odom(self, a, m):
        self.pose_t[a] = time.monotonic()
        q = m.pose.pose.orientation
        if a == self.me:
            px, py = m.pose.pose.position.x, m.pose.pose.position.y
            h = self.slam_hist
            if not h or math.hypot(px - h[-1][1], py - h[-1][2]) >= 0.02:
                h.append((time.monotonic(), px, py, yaw_of(q)))
        self.pose[a] = (m.pose.pose.position.x, m.pose.pose.position.y, yaw_of(q))
        xy = self.pose[a][:2]
        tr = self.trails[a]
        if not tr or math.dist(tr[-1], xy) >= 0.1:
            tr.append(xy)
            self.tposes[a].append((xy[0], xy[1], self.pose[a][2], self.tick_now()))
        if a == self.me:
            self.trail = tr

    def on_goal(self, a, m):
        self.goal_of[a] = (m.point.x, m.point.y) if m.point.z > 0.5 else None

    def on_merge(self, m):
        if m.child_frame_id in self.others:
            if m.child_frame_id not in self.merge:
                self.last_new_merge_t = time.monotonic()     # a map correction now is the merge, not a loop
            self.merge[m.child_frame_id] = (T_of(m.transform), time.monotonic())

    def on_metrics(self, m):
        if len(m.data) > 1:
            self.tracking = int(m.data[1])
        if len(m.data) >= 14:
            self.metrics = list(m.data)
            if self.lag_base is None:              # this robot's usual frame lag (transport, not backlog)
                self.lags.append(m.data[7] / 1000)
                if len(self.lags) >= 150:
                    self.lag_base = float(np.median(self.lags))

    def own_loops(self, dt):
        """My loop edges as training forms them (env loop_model 'views'): while I see my own old
        views again -- here: an own pose at least loop_age ticks old within 1 m, heading within
        45 deg -- an edge from my newest node to that pose's node at loop_rate per second (at most
        one per loop_cooldown ticks). ORB-SLAM3's explicit loop closures (slam_metrics corrections
        without a new merge) add one too."""
        c = self.decider.cfg
        if not (self.decider.env.graph and c.loop_model == 'views') or not self.tposes[self.me]:
            return
        now_tick = self.tick_now()
        x, y, yaw = self.pose[self.me]
        old = [p for p in self.tposes[self.me] if now_tick - p[3] >= c.loop_age and math.hypot(p[0] - x, p[1] - y) < 1.0
               and abs((p[2] - yaw + math.pi) % (2 * math.pi) - math.pi) < math.pi / 4]
        corr = self.metrics[11]
        explicit = (self.corrections is not None and corr > self.corrections
                    and time.monotonic() - self.last_new_merge_t > 10.0)
        self.corrections = corr
        self.reobs_s = self.reobs_s + dt if old else 0.0
        cool = (time.monotonic() - self.last_loop_t) >= c.loop_cooldown * c.tick_s
        if old and cool and (explicit or self.reobs_s * c.loop_rate >= 1.0):
            then = min(p[3] for p in old)
            self.own_edges.append((now_tick // c.node_ticks, then // c.node_ticks))
            self.reobs_s, self.last_loop_t = 0.0, time.monotonic()

    def on_wheel(self, m):
        q = m.pose.pose.orientation
        self.wheel_hist.append((time.monotonic(), m.pose.pose.position.x, m.pose.pose.position.y, yaw_of(q)))

    def on_links(self, m):
        """Links per teammate beyond the merge match: ORB-SLAM3 counts the merge's
        own measurements too (3-7 in Isaac; training keeps them in merge_n0)."""
        for k, v in (t.split(':') for t in m.data.split()):
            n0 = self.links_n0.setdefault(k, int(v))
            self.links[k] = max(0, int(v) - n0)

    def backlog(self):
        """Seconds of SLAM work waiting, as the training model counts it."""
        if self.lag_base is None:
            return 0.0
        return max(0.0, self.metrics[7] / 1000 - self.lag_base) + 0.1 * max(0.0, self.metrics[8])

    def tick_now(self):
        """Time since the common start in training ticks (pose-graph nodes, visit times)."""
        if self.t_start is None:
            return 0
        return int((time.monotonic() - self.t_start) / self.decider.cfg.tick_s)

    def graph_edges(self, a, T):
        """My edges with teammate a, as the training model keeps them: the merge
        match, then one per link my SLAM found (ma_links). With link_model
        'views' a link joins both graphs where both robots had a shared view:
        here a random pair of poses (mine, a's) under 1 m apart and facing
        within 45 deg (Isaac: linked keyframes a median 0.6 m and 11 deg apart).
        Otherwise (or with no such pair yet) from my newest node to a's node
        where its path came closest to me."""
        e = self.edges[a]
        want = 1 + self.links.get(a, 0)
        if len(e) < want and self.tposes[a]:
            tp = np.array(self.tposes[a])
            xy = apply3(T, tp[:, :2])
            nt = self.decider.cfg.node_ticks
            d = np.linalg.norm(xy - np.array(self.pose[self.me][:2]), axis=1)
            nearest = (self.tick_now() // nt, int(tp[int(np.argmin(d)), 3]) // nt)
            pairs = ([], [])
            if self.decider.env.views and self.tposes[self.me]:
                mp = np.array(self.tposes[self.me])
                dd = np.linalg.norm(mp[:, None, :2] - xy[None], axis=2)
                yaw = tp[:, 2] + math.atan2(T[1, 0], T[0, 0])
                dy = np.abs((mp[:, None, 2] - yaw[None] + math.pi) % (2 * math.pi) - math.pi)
                pairs = np.nonzero((dd < 1.0) & (dy < math.pi / 4))
            while len(e) < want:
                if len(e) >= 1 and len(pairs[0]):
                    k = int(self.rng.integers(len(pairs[0])))
                    e.append((int(mp[pairs[0][k], 3]) // nt, int(tp[pairs[1][k], 3]) // nt))
                else:
                    e.append(nearest)
        return list(e)

    # ── frames ───────────────────────────────────────────────────────────
    def map_from_nav(self, a):
        """T(<a>/map <- <a>/map_nav), static; None until TF has it."""
        try:
            return T_of(self.tf.lookup_transform(self.frame(a, 'map'), self.frame(a, 'map_nav'), rclpy.time.Time()).transform)
        except Exception:
            return None

    def T_me_from(self, a):
        """T(my map_nav <- a's map_nav) or None if not merged (or frames missing)."""
        if a == self.me:
            return np.eye(4)
        mg = self.merge.get(a)
        if mg is None or time.monotonic() - mg[1] > self.merge_fresh:
            return None
        Tm, Ta = self.map_from_nav(self.me), self.map_from_nav(a)
        if Tm is None or Ta is None:
            return None
        return np.linalg.inv(Tm) @ mg[0] @ Ta

    def robot(self, a):
        T = self.T_me_from(a)
        g, pose = self.grid.get(a), self.pose.get(a)
        if T is None or g is None or pose is None:
            return Robot(known=False)
        free, occ = g.cells()
        xy = apply3(T, np.array([pose[:2]]))[0]
        yaw = pose[2] + math.atan2(T[1, 0], T[0, 0])
        goal = self.goal if a == self.me else self.goal_of.get(a)
        if goal is not None and a != self.me:
            goal = tuple(apply3(T, np.array([goal]))[0])
        tr = np.array(self.trails[a]) if self.trails[a] else np.zeros((0, 2))
        tp = np.array(self.tposes[a], np.float64).reshape(-1, 4)
        if len(tp):
            tp = np.c_[apply3(T, tp[:, :2]), tp[:, 2] + math.atan2(T[1, 0], T[0, 0]), tp[:, 3]]
        if a == self.me:
            since, backlog, lost = self.metrics[10], self.backlog(), self.tracking != TRACKING_OK
            edges = list(self.own_edges)                  # my loop edges (own nodes), as training keeps them
        else:
            since, backlog = self.status.get(a, (0.0, 0.0))
            lost = False
            edges = self.graph_edges(a, T) if self.decider.env.graph else []
        return Robot(apply3(T, free), apply3(T, occ), (xy[0], xy[1], yaw), goal, True,
                     trail_pts=apply3(T, tr) if len(tr) else tr, backlog=backlog, lost=lost,
                     since_corr=since, links=self.links.get(a, 0), trail_pose=tp, edges=edges)

    # ── deciding ─────────────────────────────────────────────────────────
    def tick(self):
        now = time.monotonic()
        if self.me not in self.grid or self.me not in self.pose:
            return
        if not self.nav.server_is_ready():
            return
        if not self.ready:            # tell the session we are ready, then wait for the common start
            self.ready = True
            open(os.path.join(self.out, f'{self.me}.ready'), 'w').close()
        if self.start_file and not os.path.exists(self.start_file):
            return
        if self.t_start is None:
            self.t_start = time.monotonic()
        self.own_loops(0.5)
        st = Float64MultiArray()
        st.data = [float(self.metrics[10]), float(self.backlog())]
        self.status_pub.publish(st)
        if (self.rec_test_at > 0 and not self.test_lost and self.rec is None and self.t_start is not None
                and now - self.t_start > self.rec_test_at):
            self.rec_test_at = -1.0                       # once
            self.test_lost = True
            if self.goal_handle is not None:              # a real loss: the safety gate cancels the goal
                self.goal_seq += 1
                self.goal_handle.cancel_goal_async()
            self.get_logger().warn(f'{self.me}: recovery test: acting as if tracking were lost')
        if self.is_lost(now):
            if self.lost_since is None:           # the safety gate has stopped the car and cancelled the goal
                self.lost_since = now
                self.slam_event('lost', 0.0)
                self.get_logger().warn(f'{self.me}: no SLAM pose (tracking lost): not deciding until it is back')
            return
        if self.lost_since is not None:
            self.slam_event('back', now - self.lost_since)
            self.get_logger().info(f'{self.me}: SLAM pose back after {now - self.lost_since:.1f} s')
            self.lost_since = None
            self.goal, self.goal_yaw, self.paused_until, self.next_ok = None, None, None, now
            self.publish_goal()
        if not self.enabled:
            return
        xy = self.pose[self.me][:2]
        if self.paused_until is not None:
            if now >= self.paused_until:
                self.decide('waited')
            return
        if self.goal is None:
            reason = 'no goal' if now >= self.next_ok else None
        elif self.last_xy is not None and math.dist(xy, self.last_xy) >= self.decide_dist:
            reason = 'drove'
        elif now - self.last_t >= self.decide_period:
            reason = 'period'
        else:
            reason = None
        if reason:
            self.decide(reason)

    def decide(self, reason):
        now = time.monotonic()
        self.blocked = [b for b in self.blocked if b[2] > now]
        robots = [self.robot(self.me)] + [self.robot(a) for a in self.others]
        robots += [Robot(known=False)] * (self.team - len(robots))
        res = self.decider.decide(robots, self.trail, self.t_decisions,
                                  blocked=[b[:2] for b in self.blocked], tick=self.tick_now())
        self.t_decisions += 1
        self.last_xy, self.last_t = self.pose[self.me][:2], now
        n_goals = res['n_candidates'] - (1 if self.decider.env.n_fixed() == 2 else 0)
        if n_goals <= 0:
            self.get_logger().info(f'{self.me}: decision {self.t_decisions}: no reachable goal on the map')
        x, y, yaw = self.pose[self.me]
        merged = [a for a, r in zip(self.others, robots[1:]) if r.known]
        self.publish_known(res)
        paused, self.paused_until = self.paused_until is not None, None
        if res['goal'] is not None:
            gyaw = res.get('goal_yaw')
            if gyaw is None and math.dist(res['goal'], (x, y)) < 0.15:
                # a frontier on its own cell (the unknown is behind the camera): turn around, as in training
                gyaw = (yaw + 2 * math.pi) % (2 * math.pi) - math.pi
            self.send(res['goal'], gyaw)
        elif res.get('wait'):
            # stay here for one decision; the goal is kept
            if self.goal_handle is not None and not paused:
                self.goal_seq += 1              # the cancel below must not look like a failure
                self.goal_handle.cancel_goal_async()
            self.paused_until = now + self.wait_s
        elif paused and self.goal is not None:
            self.send(self.goal)                # "keep" after waiting: drive on
        elif self.goal is None:
            self.next_ok = now + self.stay_wait
        g = res['goal'] or self.goal or (float('nan'), float('nan'))
        kind = res['kind'] if (res['kind'] != 'keep' or self.goal is not None) else 'stay'
        self.last_kind = kind
        mate = self.others[res['mate'] - 1] if res.get('mate') else ''
        self.log.writerow([f'{time.time():.3f}', self.t_decisions, self.policy_name, res['action'], int(res['keep']),
                           f'{g[0]:.3f}', f'{g[1]:.3f}', f'{x:.3f}', f'{y:.3f}', f'{yaw:.3f}',
                           res['n_candidates'], ' '.join(merged), f"{res['ms']:.1f}", reason,
                           '' if res.get('goal_yaw') is None else f"{res['goal_yaw']:.3f}", kind, mate])
        self.log_f.flush()
        # for the live view (isaac_view.py): what was decided, and the robot's SLAM state then
        self.action_pub.publish(String(data=json.dumps({
            'decision': self.t_decisions, 'kind': kind, 'mate': mate, 'reason': reason,
            'goal': None if self.goal is None else [float(self.goal[0]), float(self.goal[1])],
            'goal_yaw': self.goal_yaw, 'wait_s': self.wait_s if kind == 'wait' else 0.0,
            'links': {a: self.links.get(a, 0) for a in merged}, 'backlog': round(float(self.backlog()), 2),
            'since_corr': round(float(self.metrics[10]), 2), 'pose': [float(x), float(y), float(yaw)],
            'mates': {a: [float(v) for v in r.pose] for a, r in zip(self.others, robots[1:]) if r.known},
            'offered': res.get('offered', [])})))

    def is_lost(self, now):
        return (self.test_lost or self.tracking != TRACKING_OK
                or now - self.pose_t.get(self.me, -1e9) > self.pose_timeout)

    # ── recovery after a tracking loss ───────────────────────────────────
    def recovery_tick(self):
        now = time.monotonic()
        if self.lost_since is None:
            if self.rec is not None:                    # tracking is back
                if self.rec['stage'] in ('drive', 'pause'):
                    self.stop_car()
                    self.slam_event('recovery_ok', now - self.rec['t_lost'])
                    self.get_logger().info(f"{self.me}: recovered after backing up {self.rec['driven']:.2f} m")
                self.rec = None
            return
        if not self.rec_on:
            return
        r = self.rec
        if r is None:
            r = self.rec = {'stage': 'wait', 't': now, 't_lost': self.lost_since, 'tries': 0, 'driven': 0.0,
                            'loss': self.slam_hist[-1] if self.slam_hist else None}
        if r['stage'] in ('failed', 'none'):
            return
        if r['stage'] in ('wait', 'pause') and now - r['t'] >= self.rec_wait:
            if r['tries'] >= self.rec_tries:
                r['stage'] = 'failed'
                self.slam_event('recovery_failed', now - r['t_lost'])
                self.get_logger().warn(f'{self.me}: tracking still lost after {r["tries"]} back-ups: '
                                       'needs help (drive it back with the keyboard)')
                return
            bt = self.backtrack(r, (r['tries'] + 1) * self.rec_back)
            if bt is None:
                r['stage'] = 'none'
                self.get_logger().warn(f'{self.me}: cannot back up (no wheel odometry on /{self.me}/odom, '
                                       'or no path driven): waiting for tracking')
                return
            r.update(stage='drive', bt=bt, tries=r['tries'] + 1, t=now, check=(now, self.wheel_hist[-1][1:3]))
            self.slam_event('recovery_start', now - r['t_lost'])
            self.get_logger().info(f'{self.me}: backing up along the last {bt.length:.2f} m driven '
                                   f'(attempt {r["tries"]})')
        if r['stage'] == 'drive':
            w = self.wheel_hist[-1]
            if now - w[0] > 0.5:                          # no wheel odometry: do not drive blind
                self.stop_car()
                r['stage'] = 'none'
                return
            v, wz, done = r['bt'].command(w[1:])
            if done:
                self.stop_car()
                r['driven'] += r['bt'].driven
                r.update(stage='pause', t=now)
                if self.test_lost:                        # test: the back-up itself is what is tested
                    self.test_lost = False
                return
            t0, xy0 = r['check']
            if now - t0 > 3.0:                            # not moving although commanded: blocked
                if math.dist(xy0, w[1:3]) < 0.03:
                    self.stop_car()
                    r['driven'] += r['bt'].driven
                    r.update(stage='pause', t=now, tries=self.rec_tries)
                    self.get_logger().warn(f'{self.me}: backing up is blocked; stopping')
                    return
                r['check'] = (now, w[1:3])
            cmd = Twist()
            cmd.linear.x, cmd.angular.z = float(v), float(wz)
            self.cmd_pub.publish(cmd)

    def backtrack(self, r, back_m):
        """A BackTrack over the last back_m metres driven before the loss, or None."""
        if r['loss'] is None or not self.wheel_hist or len(self.slam_hist) < 3:
            return None
        hist = [h for h in self.slam_hist if h[0] <= r['loss'][0]]
        path, dist = [hist[-1]], 0.0
        for h in reversed(hist[:-1]):
            dist += math.hypot(h[1] - path[-1][1], h[2] - path[-1][2])
            path.append(h)
            if dist >= back_m:
                break
        path = [(h[1], h[2], h[3]) for h in reversed(path)]
        t_loss = r['loss'][0]
        wl = min(self.wheel_hist, key=lambda w: abs(w[0] - t_loss))          # wheel pose at the loss
        if abs(wl[0] - t_loss) > 1.0 or len(path) < 3:
            return None
        bt = BackTrack(path, r['loss'][1:], wl[1:], speed=self.rec_speed)
        bt.sync(self.wheel_hist[-1][1:])
        return bt

    def stop_car(self):
        for _ in range(3):
            self.cmd_pub.publish(Twist())

    def slam_event(self, kind, duration):
        x, y, yaw = self.pose[self.me]
        g = self.goal
        bearing = dist = ''
        if g is not None:
            bearing = f'{math.degrees((math.atan2(g[1] - y, g[0] - x) - yaw + math.pi) % (2 * math.pi) - math.pi):.0f}'
            dist = f'{math.dist((x, y), g):.2f}'
        self.ev.writerow([f'{time.time():.3f}', kind, f'{duration:.1f}', f'{x:.3f}', f'{y:.3f}', f'{yaw:.3f}',
                          '' if g is None else f'{g[0]:.3f}', '' if g is None else f'{g[1]:.3f}', bearing, dist,
                          self.last_kind, self.t_decisions])
        self.ev_f.flush()

    def on_enable(self, m):
        if bool(m.data) == self.enabled:
            return
        self.enabled = bool(m.data)
        self.get_logger().info(f'{self.me}: exploring {"on" if self.enabled else "off (goal cancelled)"}')
        if not self.enabled:
            if self.goal_handle is not None:
                self.goal_seq += 1
                self.goal_handle.cancel_goal_async()
            self.goal, self.goal_yaw, self.paused_until = None, None, None
            self.publish_goal()

    def frame(self, a, name):
        return f'{a}/{name}' if self.prefixed else name

    def publish_known(self, res):
        """The map the policy decided on (own + merged teammates' grids, the
        training window) as an OccupancyGrid in my map_nav: for the live view."""
        planes, win = res['obs']['planes'][0, 0], res['window']
        free, occ = planes[0].cpu().numpy().astype(bool), planes[1].cpu().numpy().astype(bool)
        g = np.full(free.shape, -1, np.int8)
        g[free], g[occ] = 0, 100
        m = OccupancyGrid()
        m.header.frame_id = self.frame(self.me, 'map_nav')
        m.header.stamp = self.get_clock().now().to_msg()
        m.info.resolution = float(win.cell)
        m.info.width, m.info.height = int(win.W), int(win.H)
        m.info.origin.position.x, m.info.origin.position.y = float(win.x0), float(win.y0)
        m.info.origin.orientation.w = 1.0
        m.data = g.reshape(-1).tolist()
        self.known_pub.publish(m)

    def send(self, goal, goal_yaw=None):
        x, y, _ = self.pose[self.me]
        if goal_yaw is None and goal == self.goal:
            goal_yaw = self.goal_yaw                    # resending a revisit pose keeps its heading
        self.goal, self.goal_yaw = goal, goal_yaw
        self.goal_seq += 1
        seq = self.goal_seq
        msg = NavigateToPose.Goal()
        msg.pose.header.frame_id = self.frame(self.me, 'map_nav')
        msg.pose.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x, msg.pose.pose.position.y = float(goal[0]), float(goal[1])
        # a revisit pose: the heading the teammate had there (the same view); else
        # arrive facing the way it drove
        yaw = goal_yaw if goal_yaw is not None else math.atan2(goal[1] - y, goal[0] - x)
        msg.pose.pose.orientation.z, msg.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        self.nav.send_goal_async(msg).add_done_callback(lambda f: self.on_accepted(f, seq, goal))
        self.publish_goal()

    def on_accepted(self, fut, seq, goal):
        h = fut.result()
        if seq != self.goal_seq:
            return
        if not h.accepted:
            self.fail(goal, 'rejected')
            return
        self.goal_handle = h
        h.get_result_async().add_done_callback(lambda f: self.on_result(f, seq, goal))

    def on_result(self, fut, seq, goal):
        if seq != self.goal_seq:
            return            # replaced by a newer goal
        st = fut.result().status
        if st == GoalStatus.STATUS_ABORTED:
            self.fail(goal, 'aborted')
        else:             # reached, or cancelled (the safety gate on a tracking loss)
            if st == GoalStatus.STATUS_SUCCEEDED:
                # A reached goal is not offered again for a while: with a forward camera a
                # frontier the car stands on can stay a frontier (training's sensor saw all around).
                self.blocked.append((goal[0], goal[1], time.monotonic() + self.block_time))
                x, y, yaw = self.pose[self.me]
                msg = f'{self.me}: reached ({goal[0]:.2f}, {goal[1]:.2f}) at {math.dist((x, y), goal):.2f} m'
                if self.goal_yaw is not None:
                    err = (yaw - self.goal_yaw + math.pi) % (2 * math.pi) - math.pi
                    msg += f'; revisit pose, heading off by {math.degrees(err):.0f} deg'
                self.get_logger().info(msg)
            self.goal = None
            self.publish_goal()

    def fail(self, goal, why):
        self.get_logger().info(f'{self.me}: goal ({goal[0]:.2f}, {goal[1]:.2f}) {why}; blocked for {self.block_time:.0f} s')
        self.blocked.append((goal[0], goal[1], time.monotonic() + self.block_time))
        self.goal = None
        self.publish_goal()

    def publish_goal(self):
        m = PointStamped()
        m.header.frame_id = self.frame(self.me, 'map_nav')
        m.header.stamp = self.get_clock().now().to_msg()
        if self.goal is not None:
            m.point.x, m.point.y, m.point.z = float(self.goal[0]), float(self.goal[1]), 1.0
        self.goal_pub.publish(m)

    def shutdown(self):
        if self.goal_handle is not None:
            try:
                self.goal_handle.cancel_goal_async()
            except Exception:
                pass
        self.log_f.close()
        self.ev_f.close()


def main():
    rclpy.init()
    node = Planner()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        try:                          # a second Ctrl-C (ros2 launch sends one) must not abort the cleanup
            node.shutdown()
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
