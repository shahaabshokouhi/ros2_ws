#!/usr/bin/env python3
"""Score a multi-robot simulation test and write its report.

    ma_report.py RUN_DIR

RUN_DIR is what run_ma_sim_test.sh produced (raw/: spec, Isaac's ground truth,
each robot's SLAM evaluation files and live poses). Needs numpy and
matplotlib (run it with ~/venvs/env_isaaclab/bin/python).

What it measures:

1. Each robot's own path.
   - Keyframes, fitted: the robot's final keyframe camera positions (after
     SLAM's own optimisation), rigidly fitted to the true camera positions
     (no scale: RGB-D is metric). The RMS distance is the usual ATE: the
     shape of the path, with the best possible placement.
   - Keyframes, anchored: the same keyframes placed in the world through the
     robot's FIRST keyframe and its true pose, without any fitting. This adds
     the drift accumulated since the start, which is what another robot
     relying on the map would see.
   - Live estimate: the pose the robot drove on while running
     (/<agent>/orb_slam3/odom, rear axle, 2D), fitted to the true rear axle.
2. Merging, for every host robot H holding peer P's map:
   - Alignment error: every map-to-map transform H logged for P, compared
     with the TRUE transform between the two maps (each map's origin is its
     first keyframe, whose true camera pose is known in the simulation, so
     the truth does not inherit either robot's drift).
   - Placement error: each of P's keyframes as it sits in H's map (fused
     "seam" keyframes and "copies"), placed in the world through H's map
     origin, against where P's camera truly was at that time.
3. Coverage: each robot's own map (what a single-robot method would have
   built in the same run) against its integrated map (own + everything
   received from the others): map points, occupied 10 cm cubes (3D), occupied
   10 cm floor cells (m^2), and the share of the team's map (all robots' own
   maps together) that the robot knows, alone and with sharing.
4. Figures: route overview, path vs ground truth per robot, the accuracy
   summary, alignment error over time, placement errors, and each robot's
   map in 3D (map points and keyframes coloured by the robot they came from).

Writes RUN_DIR/report.html, metrics.json, figures/*.png and tables/*.csv.
"""
import csv
import datetime
import html
import json
import math
import os
import sys

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.lines import Line2D  # noqa: E402

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
from jetracer_sim import routes  # noqa: E402

REAR = 0.0698                       # rear axle behind Isaac's base_link (m)
HALF = 2.975                        # inner wall faces of the room (m)
# The floor inside the walls (xmin, ymin, xmax, ymax): the room, or a session's world
# (raw/world/free_grid.json, run_isaac_explore.sh --env big); set by load_obstacles.
BOUNDS = [-HALF, -HALF, HALF, HALF]


def xlim(ax, pad):
    ax.set_xlim(BOUNDS[0] - pad, BOUNDS[2] + pad)
    ax.set_ylim(BOUNDS[1] - pad, BOUNDS[3] + pad)


def box_aspect3d(ax):
    ax.set_box_aspect((BOUNDS[2] - BOUNDS[0], BOUNDS[3] - BOUNDS[1], 2.2), zoom=0.95)
# Categorical slots 1-3 of the report palette (validated for 3 series, all pairs).
AGENT_COLORS = ['#2a78d6', '#eb6834', '#1baf7a']
INK, INK2, MUTED, GRID, SURFACE = '#0b0b0b', '#52514e', '#8a8984', '#e4e3df', '#fcfcfb'
TRUTH = '#52514e'

plt.rcParams.update({
    'font.family': 'DejaVu Sans', 'font.size': 9, 'axes.titlesize': 10, 'axes.titleweight': 'bold',
    'axes.labelcolor': INK2, 'axes.edgecolor': GRID, 'axes.linewidth': 0.8,
    'xtick.color': INK2, 'ytick.color': INK2, 'text.color': INK,
    'axes.grid': True, 'grid.color': GRID, 'grid.linewidth': 0.6,
    'axes.facecolor': SURFACE, 'figure.facecolor': 'white', 'savefig.facecolor': 'white',
    'legend.frameon': False, 'legend.fontsize': 8, 'lines.linewidth': 1.6,
    'axes.spines.top': False, 'axes.spines.right': False,
})


# ── geometry ───────────────────────────────────────────────────────────────
def quat_to_R(qx, qy, qz, qw):
    q = np.array([qw, qx, qy, qz], float)
    q /= np.linalg.norm(q)
    w, x, y, z = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def se3(R, t):
    T = np.eye(4)
    T[:3, :3], T[:3, 3] = R, t
    return T


def inv(T):
    R, t = T[:3, :3], T[:3, 3]
    return se3(R.T, -R.T @ t)


def rot_deg(R):
    return float(np.degrees(np.arccos(np.clip((np.trace(R) - 1) / 2, -1, 1))))


def umeyama(src, dst):
    """Rigid R, t minimising sum ||dst - (R src + t)||^2 (any dimension, no scale)."""
    ms, md = src.mean(0), dst.mean(0)
    H = (src - ms).T @ (dst - md)
    U, _, Vt = np.linalg.svd(H)
    D = np.eye(src.shape[1])
    D[-1, -1] = np.sign(np.linalg.det(Vt.T @ U.T))
    R = Vt.T @ D @ U.T
    return R, md - R @ ms


def stats(x):
    x = np.asarray(x, float)
    if x.size == 0:
        return None
    return {'n': int(x.size), 'rmse': float(np.sqrt(np.mean(x ** 2))), 'median': float(np.median(x)),
            'mean': float(x.mean()), 'p95': float(np.percentile(x, 95)), 'max': float(x.max())}


def cm(s, key='rmse'):
    return '–' if not s else f'{s[key] * 100:.1f}'


# ── input ──────────────────────────────────────────────────────────────────
def read_csv(path):
    if not os.path.exists(path):
        return []
    with open(path) as f:
        return list(csv.DictReader(f))


class Truth:
    """One robot's ground truth from Isaac: camera (optical frame) and base."""

    def __init__(self, path):
        a = np.loadtxt(path, delimiter=',', skiprows=1, ndmin=2)
        a = a[np.argsort(a[:, 0])]
        self.t = a[:, 0]
        self.base = a[:, 1:5]           # x, y, z, yaw
        self.cam = a[:, 5:8]
        self.q = a[:, 8:12]

    def _interp(self, t, cols):
        return np.stack([np.interp(t, self.t, cols[:, k]) for k in range(cols.shape[1])], -1)

    def valid(self, t):
        t = np.asarray(t, float)
        return (t >= self.t[0]) & (t <= self.t[-1])

    def cam_pos(self, t):
        return self._interp(np.asarray(t, float), self.cam)

    def cam_pose(self, t):
        i = int(np.clip(np.searchsorted(self.t, t), 0, len(self.t) - 1))
        return se3(quat_to_R(*self.q[i]), self.cam_pos(t))

    def rear(self, t):
        """True rear-axle position (x, y) at time(s) t."""
        t = np.asarray(t, float)
        yaw = np.interp(t, self.t, np.unwrap(self.base[:, 3]))
        x = np.interp(t, self.t, self.base[:, 0]) - REAR * np.cos(yaw)
        y = np.interp(t, self.t, self.base[:, 1]) - REAR * np.sin(yaw)
        return np.stack([x, y], -1)


def has_main(rows):
    return any(r['in_main_map'] == '1' for r in rows)


def keyframes(rows, kind=None, origin=None, main_only=True):
    """Keyframes of the robot's main map (or, before a map has qualified as
    main, of its map: short sessions), optionally by kind and origin robot."""
    out = []
    main_only = main_only and has_main(rows)
    for r in rows:
        if main_only and r['in_main_map'] != '1':
            continue
        if kind and r['kind'] not in kind:
            continue
        if origin and r['origin_agent'] != origin:
            continue
        T = se3(quat_to_R(float(r['qx']), float(r['qy']), float(r['qz']), float(r['qw'])),
                [float(r['tx']), float(r['ty']), float(r['tz'])])
        out.append((float(r['timestamp']), T, r))
    out.sort(key=lambda k: k[0])
    return out


# ── scoring ────────────────────────────────────────────────────────────────
def fit_own(kfs, truth, dt):
    """Rigid fit of own keyframe positions to the truth; (T_world<-map, errors, times)."""
    ts = np.array([k[0] for k in kfs]) + dt
    ok = truth.valid(ts)
    if ok.sum() < 3:
        return None, None, None
    src = np.array([k[1][:3, 3] for k in kfs])[ok]
    dst = truth.cam_pos(ts[ok])
    R, t = umeyama(src, dst)
    err = np.linalg.norm(dst - (src @ R.T + t), axis=1)
    return se3(R, t), err, ts[ok] - dt


def anchor(kfs, truth, dt):
    """T_world<-map from the map's first keyframe and its true camera pose."""
    for t, T, _ in kfs:
        if truth.valid(t + dt):
            return truth.cam_pose(t + dt) @ inv(T)
    return None


def best_time_offset(own, truths):
    """Constant offset between image stamps and the truth log's clock that
    minimises the fitted keyframe error (normally a few ms)."""
    best = (None, 0.0)
    for dt in np.arange(-0.15, 0.1501, 0.005):
        tot = 0.0
        for a, kfs in own.items():
            _, err, _ = fit_own(kfs, truths[a], dt)
            if err is None:
                return 0.0
            tot += float(np.mean(err ** 2))
        if best[0] is None or tot < best[0]:
            best = (tot, float(dt))
    return best[1]


def online_errors(rows, truth, dt):
    """Live SLAM pose (rear axle, 2D, in map_nav) fitted to the true rear axle."""
    if len(rows) < 10:
        return None, None, None, None
    st = np.array([float(r['stamp']) for r in rows]) + dt
    xy = np.array([[float(r['x']), float(r['y'])] for r in rows])
    ok = truth.valid(st)
    if ok.sum() < 10:
        return None, None, None, None
    gt = truth.rear(st[ok])
    R, t = umeyama(xy[ok], gt)
    est = xy[ok] @ R.T + t
    return np.linalg.norm(gt - est, axis=1), st[ok] - dt, est, gt


# ── report ─────────────────────────────────────────────────────────────────
def main():
    run = os.path.abspath(sys.argv[1])
    raw = os.path.join(run, 'raw')
    figdir, tabdir = os.path.join(run, 'figures'), os.path.join(run, 'tables')
    os.makedirs(figdir, exist_ok=True)
    os.makedirs(tabdir, exist_ok=True)
    spec = routes.load_spec(os.path.join(raw, 'spec.json'))
    load_obstacles(raw)
    agents = spec['agents']
    color = {a: AGENT_COLORS[i % len(AGENT_COLORS)] for i, a in enumerate(agents)}
    env = open(os.path.join(raw, 'env.txt')).read() if os.path.exists(os.path.join(raw, 'env.txt')) else ''
    method = next((l.split(': ', 1)[1] for l in env.splitlines() if l.startswith('method: ')), '?')
    t_drive = float(open(os.path.join(raw, 'drive_start.txt')).read()) if os.path.exists(
        os.path.join(raw, 'drive_start.txt')) else None
    t_end = float(open(os.path.join(raw, 'drive_end.txt')).read()) if os.path.exists(
        os.path.join(raw, 'drive_end.txt')) else None

    truths = {a: Truth(os.path.join(raw, 'gt', f'{a}_gt.csv')) for a in agents}
    kf_rows = {a: read_csv(os.path.join(raw, a, 'eval', 'keyframes.csv')) for a in agents}
    own = {a: keyframes(kf_rows[a], kind={'own'}) for a in agents}
    usable = {a: k for a, k in own.items() if len(k) >= 3}
    dt = best_time_offset(usable, truths) if usable else 0.0
    t0 = t_drive or min(tr.t[0] for tr in truths.values())

    metrics = {'run': run, 'method': method, 'spec': spec, 'time_offset_s': dt, 'agents': {}, 'pairs': {}}

    # 1. own paths
    T_fit, T_anchor, per_agent = {}, {}, {}
    for a in agents:
        m = {}
        frames = read_csv(os.path.join(raw, a, 'eval', 'frames.csv'))
        if frames:
            states = [int(float(r['tracking_state'])) for r in frames]
            tm = np.array([float(r['track_ms']) for r in frames])
            m['frames'] = len(frames)
            m['tracking_ok_fraction'] = float(np.mean([s == 2 for s in states]))
            m['track_ms_median'] = float(np.median(tm))
            m['track_ms_p95'] = float(np.percentile(tm, 95))
        summ = {r['key']: r['value'] for r in read_csv(os.path.join(raw, a, 'eval', 'summary.csv'))}
        m['summary'] = summ
        kfs = own[a]
        m['own_keyframes'] = len(kfs)
        T, err, ts = fit_own(kfs, truths[a], dt) if len(kfs) >= 3 else (None, None, None)
        if T is not None:
            T_fit[a] = T
            m['kf_fitted'] = stats(err)
            Ta = anchor(kfs, truths[a], dt)
            T_anchor[a] = Ta
            pos = np.array([k[1][:3, 3] for k in kfs])
            tts = np.array([k[0] for k in kfs]) + dt
            ok = truths[a].valid(tts)
            aerr = np.linalg.norm(truths[a].cam_pos(tts[ok]) - (pos[ok] @ Ta[:3, :3].T + Ta[:3, 3]), axis=1)
            m['kf_anchored'] = stats(aerr)
            with open(os.path.join(tabdir, f'keyframe_errors_{a}.csv'), 'w') as f:
                f.write('time_s,fitted_err_m,anchored_err_m\n')
                for t_, e1, e2 in zip(ts, err, aerr):
                    f.write(f'{t_ - t0:.3f},{e1:.5f},{e2:.5f}\n')
        rows = read_csv(os.path.join(raw, 'online', f'{a}_online.csv'))
        oerr, ots, oest, ogt = online_errors(rows, truths[a], dt)
        if oerr is not None:
            m['online'] = stats(oerr)
            if t_drive and t_end:
                gaps = np.diff(np.concatenate([[t_drive], ots[(ots > t_drive) & (ots < t_end)], [t_end]]))
                m['online_coverage'] = float(1 - gaps[gaps > 0.5].sum() / (t_end - t_drive))
            with open(os.path.join(tabdir, f'online_errors_{a}.csv'), 'w') as f:
                f.write('time_s,err_m,est_x,est_y,true_x,true_y\n')
                for t_, e, p, g in zip(ots, oerr, oest, ogt):
                    f.write(f'{t_ - t0:.3f},{e:.5f},{p[0]:.4f},{p[1]:.4f},{g[0]:.4f},{g[1]:.4f}\n')
        per_agent[a] = (err, ts, oerr, ots, oest)
        metrics['agents'][a] = m

    # 2. merging
    # A received keyframe carries no timestamp in the host's file: look it up
    # by its id in the owner's own records (what it served, and its own map).
    stamp_of = {}
    for a in agents:
        d = {int(r['kf_id']): float(r['timestamp'])
             for r in read_csv(os.path.join(raw, a, 'eval', 'served.csv'))}
        for r in kf_rows[a]:
            if r['kind'] == 'own':
                d.setdefault(int(r['kf_id']), float(r['timestamp']))
        stamp_of[a] = d
    pairs = {}
    for h in agents:
        for p in agents:
            if h == p:
                continue
            key = f'{h}<-{p}'
            pm = {}
            ver = [r for r in read_csv(os.path.join(raw, h, 'eval', 'verify.csv')) if r['peer'] == p]
            pm['verifications'] = len(ver)
            pm['verifications_passed'] = sum(r['ok'] in ('1', 'true') for r in ver)
            al = [r for r in read_csv(os.path.join(raw, h, 'eval', 'alignment.csv')) if r['peer'] == p]
            series = []
            if al and h in T_anchor and p in T_anchor:
                true_anchor = inv(T_anchor[h]) @ T_anchor[p]
                true_fit = inv(T_fit[h]) @ T_fit[p]
                for r in al:
                    T = se3(quat_to_R(float(r['qx']), float(r['qy']), float(r['qz']), float(r['qw'])),
                            [float(r['tx']), float(r['ty']), float(r['tz'])])
                    E, F = inv(true_anchor) @ T, inv(true_fit) @ T
                    series.append((float(r['wall']) - t0, int(float(r['n'])), float(np.linalg.norm(E[:3, 3])),
                                   rot_deg(E[:3, :3]), float(np.linalg.norm(F[:3, 3])), rot_deg(F[:3, :3])))
                with open(os.path.join(tabdir, f'alignment_{h}_from_{p}.csv'), 'w') as f:
                    f.write('time_s,n,trans_err_m,rot_err_deg,trans_err_vs_fit_m,rot_err_vs_fit_deg\n')
                    for s in series:
                        f.write(','.join(f'{v:.5f}' if isinstance(v, float) else str(v) for v in s) + '\n')
            pm['alignments'] = len(al)
            if series:
                pm['first_alignment_s'] = series[0][0]
                pm['final_alignment_trans_err_m'] = series[-1][2]
                pm['final_alignment_rot_err_deg'] = series[-1][3]
                pm['final_alignment_trans_err_vs_fit_m'] = series[-1][4]
            placed = keyframes(kf_rows[h], kind={'seam', 'copy'}, origin=p)
            perr = []
            if placed and h in T_anchor:
                Tw = T_anchor[h]
                with open(os.path.join(tabdir, f'placement_{h}_from_{p}.csv'), 'w') as f:
                    f.write('time_s,kind,err_m\n')
                    for _, T, r in placed:
                        t_ = stamp_of[p].get(int(r['origin_kf_id']))
                        if t_ is None or not truths[p].valid(t_ + dt):
                            continue
                        e = float(np.linalg.norm((Tw @ T)[:3, 3] - truths[p].cam_pos(t_ + dt)))
                        perr.append((r['kind'], e))
                        f.write(f'{t_ - t0:.3f},{r["kind"]},{e:.5f}\n')
            pm['placed_keyframes'] = len(perr)
            pm['placement'] = stats([e for _, e in perr])
            pm['placement_seam'] = stats([e for k, e in perr if k == 'seam'])
            pm['placement_copy'] = stats([e for k, e in perr if k == 'copy'])
            mp = read_csv(os.path.join(raw, h, 'eval', 'mappoints.csv'))
            pm['map_points_from_peer'] = sum(r['origin_agent'] == p and (r['in_main_map'] == '1' or not has_main(mp))
                                             for r in mp)
            pairs[key] = (series, perr)
            metrics['pairs'][key] = pm

    # 3. coverage: own map vs integrated map
    cov_pts, team_vox, team_cells = coverage_points(raw, agents, T_fit)
    for h, d in cov_pts.items():
        own_v, int_v = voxels(d['own']), voxels(d['all'])
        own_c, int_c = cells(d['own']), cells(d['all'])
        c = {'own_points': int(len(d['own'])), 'integrated_points': int(len(d['all'])),
             'own_voxels': len(own_v), 'integrated_voxels': len(int_v),
             'own_area_m2': len(own_c) * CELL ** 2, 'integrated_area_m2': len(int_c) * CELL ** 2,
             'team_voxels': len(team_vox), 'team_area_m2': len(team_cells) * CELL ** 2}
        c['voxel_gain'] = c['integrated_voxels'] / max(1, c['own_voxels']) - 1
        c['area_gain'] = c['integrated_area_m2'] / max(1e-9, c['own_area_m2']) - 1
        c['team_share_own'] = len(own_v & team_vox) / max(1, len(team_vox))
        c['team_share_integrated'] = len(int_v & team_vox) / max(1, len(team_vox))
        metrics['agents'][h]['coverage'] = c
    with open(os.path.join(tabdir, 'coverage.csv'), 'w') as f:
        keys = ['own_points', 'integrated_points', 'own_voxels', 'integrated_voxels', 'voxel_gain', 'own_area_m2',
                'integrated_area_m2', 'area_gain', 'team_share_own', 'team_share_integrated']
        f.write('robot,' + ','.join(keys) + '\n')
        for h in cov_pts:
            c = metrics['agents'][h]['coverage']
            f.write(h + ',' + ','.join(f'{c[k]:.4f}' if isinstance(c[k], float) else str(c[k]) for k in keys) + '\n')

    with open(os.path.join(run, 'metrics.json'), 'w') as f:
        json.dump(metrics, f, indent=2)

    figs = make_figures(run, raw, figdir, spec, agents, color, truths, own, kf_rows, T_fit, T_anchor,
                        per_agent, pairs, metrics, t0, dt)
    figs.update(coverage_figures(figdir, agents, color, cov_pts, metrics))
    write_html(run, spec, agents, color, metrics, figs, env, t0, t_end, dt)
    print(f'report: {os.path.join(run, "report.html")}')


# ── coverage ───────────────────────────────────────────────────────────────
VOXEL = 0.10     # m, 3D coverage cubes
CELL = 0.10      # m, floor-plan coverage cells


def in_room(W):
    return ((W[:, 0] > BOUNDS[0] - 0.5) & (W[:, 0] < BOUNDS[2] + 0.5) & (W[:, 1] > BOUNDS[1] - 0.5)
            & (W[:, 1] < BOUNDS[3] + 0.5) & (W[:, 2] > -0.3) & (W[:, 2] < 2.3))


def coverage_points(raw, agents, T_fit):
    """Per robot, its map points in the world frame (through its fitted
    placement): own ones, all of them, and per origin robot; plus the team's
    map (every robot's own points) as voxels and floor cells."""
    out, team_v, team_c = {}, set(), set()
    for h in agents:
        if h not in T_fit:
            continue
        mp = read_csv(os.path.join(raw, h, 'eval', 'mappoints.csv'))
        if not mp:
            continue
        main = has_main(mp)
        rows = [r for r in mp if r['in_main_map'] == '1' or not main]
        if not rows:
            continue
        P = np.array([[float(r['x']), float(r['y']), float(r['z'])] for r in rows])
        org = np.array([r['origin_agent'] for r in rows])
        T = T_fit[h]
        W = P @ T[:3, :3].T + T[:3, 3]
        keep = in_room(W)
        W, org = W[keep], org[keep]
        out[h] = {'own': W[org == h], 'all': W, 'by_origin': {a: W[org == a] for a in agents}}
        team_v |= voxels(W[org == h])
        team_c |= cells(W[org == h])
    return out, team_v, team_c


def voxels(W):
    return set(map(tuple, np.floor(W / VOXEL).astype(int))) if len(W) else set()


def cells(W):
    return set(map(tuple, np.floor(W[:, :2] / CELL).astype(int))) if len(W) else set()


def coverage_figures(figdir, agents, color, cov_pts, metrics):
    figs = {}
    hosts = [h for h in agents if h in cov_pts]
    if not hosts:
        return figs
    # Summary: share of the team's map each robot knows, alone and with sharing.
    fig, axes = plt.subplots(1, 2, figsize=(10, 3.6))
    x = np.arange(len(hosts))
    for ax, (k_own, k_int, title, scale, unit) in zip(axes, [
            ('team_share_own', 'team_share_integrated', "Share of the team's map this robot knows", 100, '%'),
            ('own_area_m2', 'integrated_area_m2', 'Floor area covered by the map', 1, 'm²')]):
        for i, h in enumerate(hosts):
            c = metrics['agents'][h]['coverage']
            vo, vi = c[k_own] * scale, c[k_int] * scale
            ax.bar(i - 0.2, vo, width=0.38, color=color[h], alpha=0.35, hatch='///', edgecolor=color[h], lw=0)
            ax.bar(i + 0.2, vi, width=0.38, color=color[h])
            for xx, v in ((i - 0.2, vo), (i + 0.2, vi)):
                ax.text(xx, v, f'{v:.0f}{unit}' if unit == '%' else f'{v:.1f}', ha='center', va='bottom', fontsize=7, color=INK2)
        ax.set_xticks(x)
        ax.set_xticklabels(hosts)
        ax.set_title(title)
        ax.set_ylabel(unit if unit != '%' else 'percent of all robots\' own maps together')
        ax.grid(axis='x', visible=False)
    fig.legend(handles=[plt.Rectangle((0, 0), 1, 1, fc=MUTED, alpha=0.35, hatch='///', ec=MUTED, lw=0),
                        plt.Rectangle((0, 0), 1, 1, fc=INK2)],
               labels=['own map only (single robot)', 'integrated map (multi-robot)'],
               loc='lower center', ncol=2, bbox_to_anchor=(0.5, -0.06))
    figs['coverage'] = save(fig, figdir, 'coverage_summary.png')

    # Per robot: own vs integrated point cloud in 3D, and the floor plan of who added what.
    for h in hosts:
        d = cov_pts[h]
        c = metrics['agents'][h]['coverage']
        fig = plt.figure(figsize=(13, 4.6))
        gs = fig.add_gridspec(1, 3, width_ratios=[1, 1, 0.85], wspace=0.3)
        for col, (title, sets) in enumerate([
                (f'{h}: own map only\n{c["own_points"]} points, {c["own_voxels"]} cubes of 10 cm', [(h, d['own'])]),
                (f'{h}: integrated map\n{c["integrated_points"]} points, {c["integrated_voxels"]} cubes '
                 f'(+{100 * c["voxel_gain"]:.0f} %)', [(a, d['by_origin'][a]) for a in agents])]):
            ax = fig.add_subplot(gs[col], projection='3d')
            for a, W in sets:
                if len(W):
                    ax.scatter(W[:, 0], W[:, 1], W[:, 2], s=0.5 if a == h else 0.9, color=color[a],
                               alpha=0.5 if a == h else 0.8, lw=0, depthshade=False)
            xlim(ax, 0)
            ax.set_zlim(-0.1, 2.1)
            ax.set_zticks([0, 1, 2])
            box_aspect3d(ax)
            ax.view_init(elev=28, azim=-60)
            ax.tick_params(labelsize=6, pad=0)
            ax.set_title(title, fontsize=9)
        ax2 = fig.add_subplot(gs[2])
        room(ax2)
        own_c = cells(d['own'])
        x_lo, y_lo = BOUNDS[0] - 0.5, BOUNDS[1] - 0.5
        img = np.full((int(round((BOUNDS[3] - BOUNDS[1] + 1) / CELL)), int(round((BOUNDS[2] - BOUNDS[0] + 1) / CELL)), 4), 0.0)
        offx, offy = -int(math.floor(x_lo / CELL)), -int(math.floor(y_lo / CELL))

        def paint(cs, rgba):
            for cx, cy in cs:
                i, j = cy + offy, cx + offx
                if 0 <= i < img.shape[0] and 0 <= j < img.shape[1] and img[i, j, 3] == 0:
                    img[i, j] = rgba
        paint(own_c, matplotlib.colors.to_rgba(color[h], 0.9))
        for a in agents:
            if a != h:
                paint(cells(d['by_origin'][a]) - own_c, matplotlib.colors.to_rgba(color[a], 0.9))
        ax2.imshow(img, origin='lower', extent=(-offx * CELL, (img.shape[1] - offx) * CELL,
                                                -offy * CELL, (img.shape[0] - offy) * CELL),
                   interpolation='nearest', zorder=1)
        xlim(ax2, 0.15)
        ax2.set_title(f'{h}: floor plan of the map\n{c["own_area_m2"]:.1f} m² own, {c["integrated_area_m2"]:.1f} m² '
                      f'integrated (+{100 * c["area_gain"]:.0f} %)', fontsize=9)
        handles = [Line2D([], [], ls='', marker='s', ms=7, color=color[h], label=f'{h} (own)')]
        handles += [Line2D([], [], ls='', marker='s', ms=7, color=color[a], label=f'added by {a}') for a in agents if a != h]
        fig.legend(handles=handles, loc='lower center', ncol=len(handles), bbox_to_anchor=(0.5, -0.04))
        figs[f'coverage_{h}'] = save(fig, figdir, f'coverage_{h}.png')
    return figs


# ── figures ────────────────────────────────────────────────────────────────
OBSTACLES = []


def load_obstacles(raw):
    """Furniture and prop footprints and the floor's bounds: a world's (raw/world/, from
    run_isaac_explore.sh), else the room's (raw/room_obstacles.json, else the config's)."""
    wdir = os.path.join(raw, 'world')
    if os.path.exists(os.path.join(wdir, 'free_grid.json')):
        BOUNDS[:] = json.load(open(os.path.join(wdir, 'free_grid.json')))['world_bounds']
        OBSTACLES[:] = json.load(open(os.path.join(wdir, 'obstacles.json')))['objects']
        return
    for path in (os.path.join(raw, 'room_obstacles.json'),
                 os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'config', 'room_obstacles.json')):
        if os.path.exists(path):
            OBSTACLES[:] = json.load(open(path))['objects']
            return


def room(ax):
    ax.add_patch(plt.Rectangle((BOUNDS[0], BOUNDS[1]), BOUNDS[2] - BOUNDS[0], BOUNDS[3] - BOUNDS[1],
                               fill=False, ec=INK2, lw=1.2))
    for o in OBSTACLES:
        if o.get('polygon'):
            ax.add_patch(plt.Polygon(o['polygon'], closed=True, fc='#ecebe7', ec='#d6d5cf', lw=0.5, zorder=0))
        else:
            ax.add_patch(plt.Rectangle((o['x'][0], o['y'][0]), o['x'][1] - o['x'][0], o['y'][1] - o['y'][0],
                                       fc='#ecebe7', ec='#d6d5cf', lw=0.5, zorder=0))
    ax.set_aspect('equal')
    xlim(ax, 0.15)
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')


def save(fig, figdir, name):
    path = os.path.join(figdir, name)
    fig.savefig(path, dpi=150, bbox_inches='tight')
    plt.close(fig)
    return os.path.join('figures', name)


def make_figures(run, raw, figdir, spec, agents, color, truths, own, kf_rows, T_fit, T_anchor,
                 per_agent, pairs, metrics, t0, dt):
    figs = {}
    pts = np.array(routes.route_points(spec['route'])) if 'route' in spec else None

    # Overview: route, true paths, starts.
    fig, ax = plt.subplots(figsize=(5.2, 5.2))
    room(ax)
    if pts is not None:
        ax.plot(np.r_[pts[:, 0], pts[0, 0]], np.r_[pts[:, 1], pts[0, 1]], color=MUTED, lw=1, ls=(0, (4, 3)),
                label='planned route')
    for a, (x, y, yaw) in zip(agents, routes.start_poses(spec)):
        tr = truths[a]
        ax.plot(tr.base[:, 0], tr.base[:, 1], color=color[a], lw=1.6, label=f'{a} (true path)')
        ax.annotate('', xy=(x + 0.3 * math.cos(yaw), y + 0.3 * math.sin(yaw)), xytext=(x, y),
                    arrowprops=dict(arrowstyle='-|>', color=color[a], lw=1.6))
        ax.text(x, y - 0.22, f'{a} start', color=INK2, fontsize=8, ha='center', va='top')
    ax.set_title('Start poses and the true paths driven' if pts is None else 'Test route and the true paths driven')
    ax.text((BOUNDS[0] + BOUNDS[2]) / 2, BOUNDS[1] + 0.08, 'grey: furniture and props 2-30 cm above the floor', ha='center', fontsize=7, color=MUTED)
    ax.legend(loc='upper center', bbox_to_anchor=(0.5, -0.1), ncol=2)
    figs['overview'] = save(fig, figdir, 'overview.png')

    # Path vs ground truth, per robot.
    for a in agents:
        err, ts, oerr, ots, oest = per_agent[a]
        if a not in T_fit and oerr is None:
            continue
        fig, (ax, ax2) = plt.subplots(1, 2, figsize=(10, 4.4), gridspec_kw={'width_ratios': [1, 1.25]})
        room(ax)
        tr = truths[a]
        cams = tr.cam
        ax.plot(cams[:, 0], cams[:, 1], color=TRUTH, lw=2.4, alpha=0.35, label='true camera path')
        if a in T_fit:
            P = np.array([k[1][:3, 3] for k in own[a]])
            W = P @ T_fit[a][:3, :3].T + T_fit[a][:3, 3]
            ax.plot(W[:, 0], W[:, 1], 'o', color=color[a], ms=3, label='keyframes (final, fitted)')
        if oest is not None:
            ax.plot(oest[:, 0], oest[:, 1], color=color[a], lw=0.8, ls=(0, (2, 2)), label='live estimate (rear axle, fitted)')
        ax.set_title(f'{a}: estimated path vs ground truth')
        ax.legend(loc='upper center', bbox_to_anchor=(0.5, -0.12), ncol=1)
        if a in T_fit:
            kfe = np.loadtxt(os.path.join(run, 'tables', f'keyframe_errors_{a}.csv'), delimiter=',', skiprows=1, ndmin=2)
            ax2.plot(kfe[:, 0], kfe[:, 1] * 100, 'o', color=color[a], ms=3, label='keyframes, fitted')
            ax2.plot(kfe[:, 0], kfe[:, 2] * 100, 's', color=INK2, ms=3, label='keyframes, anchored at the first')
        if oerr is not None:
            ax2.plot(ots - t0, oerr * 100, color=color[a], lw=0.8, alpha=0.7, label='live estimate, fitted')
        ax2.set_xlabel('time since the start (s)')
        ax2.set_ylabel('position error (cm)')
        ax2.set_ylim(bottom=0)
        ax2.set_title('Position error over time')
        ax2.legend(loc='upper left')
        figs[f'traj_{a}'] = save(fig, figdir, f'path_{a}.png')

    # Accuracy summary.
    labels = [('kf_fitted', 'keyframes,\nfitted (ATE)'), ('kf_anchored', 'keyframes,\nanchored'),
              ('online', 'live estimate,\nfitted')]
    fig, ax = plt.subplots(figsize=(7, 3.4))
    w = 0.8 / len(agents)
    for i, a in enumerate(agents):
        vals = [metrics['agents'][a].get(k, {}) or {} for k, _ in labels]
        ys = [v.get('rmse', 0) * 100 for v in vals]
        xs = np.arange(len(labels)) + (i - (len(agents) - 1) / 2) * w
        bars = ax.bar(xs, ys, width=w * 0.92, color=color[a], label=a)
        for x_, y_ in zip(xs, ys):
            ax.text(x_, y_ + 0.15, f'{y_:.1f}', ha='center', va='bottom', fontsize=7, color=INK2)
    ax.set_xticks(np.arange(len(labels)))
    ax.set_xticklabels([l for _, l in labels])
    ax.set_ylabel('RMS position error (cm)')
    ax.set_title('Path accuracy per robot')
    ax.grid(axis='x', visible=False)
    ax.legend(ncol=len(agents), loc='upper left')
    figs['accuracy'] = save(fig, figdir, 'accuracy.png')

    # Alignment error over time, one panel per host.
    hosts = [h for h in agents if any(pairs.get(f'{h}<-{p}', ([], []))[0] for p in agents if p != h)]
    if hosts:
        fig, axes = plt.subplots(len(hosts), 2, figsize=(10, 2.6 * len(hosts)), squeeze=False)
        for row, h in enumerate(hosts):
            for p in agents:
                if p == h:
                    continue
                s = pairs[f'{h}<-{p}'][0]
                if not s:
                    continue
                s = np.array(s, float)
                axes[row, 0].plot(s[:, 0], s[:, 2] * 100, 'o-', color=color[p], ms=3, lw=1.2, label=f'map of {p}')
                axes[row, 1].plot(s[:, 0], s[:, 3], 'o-', color=color[p], ms=3, lw=1.2, label=f'map of {p}')
            axes[row, 0].set_ylabel(f'{h}\ntranslation error (cm)')
            axes[row, 1].set_ylabel('rotation error (°)')
            for c in (0, 1):
                axes[row, c].set_ylim(bottom=0)
                axes[row, c].legend(loc='upper right')
        axes[0, 0].set_title('Map alignment error: translation')
        axes[0, 1].set_title('Map alignment error: rotation')
        for c in (0, 1):
            axes[-1, c].set_xlabel('time since the start (s)')
        figs['alignment'] = save(fig, figdir, 'alignment_error.png')

    # Placement of received keyframes.
    rows = [(f'{h} ← {p}', [e for _, e in pairs[f'{h}<-{p}'][1]], color[p])
            for h in agents for p in agents if h != p and pairs[f'{h}<-{p}'][1]]
    if rows:
        fig, ax = plt.subplots(figsize=(7, 0.55 * len(rows) + 1.2))
        for i, (lab, es, c) in enumerate(rows):
            es = np.array(es) * 100
            jitter = (np.random.default_rng(i).random(len(es)) - 0.5) * 0.35
            ax.scatter(es, np.full(len(es), i) + jitter, s=10, color=c, alpha=0.7, lw=0)
            ax.plot([np.median(es)] * 2, [i - 0.3, i + 0.3], color=INK, lw=2)
        ax.set_yticks(range(len(rows)))
        ax.set_yticklabels([r[0] for r in rows])
        ax.set_ylabel('host ← peer')
        ax.invert_yaxis()
        ax.set_xlabel("distance from the peer camera's true position (cm); black bar = median")
        ax.set_title("Where received keyframes land in the host's map")
        ax.set_xlim(left=0)
        ax.grid(axis='y', visible=False)
        figs['placement'] = save(fig, figdir, 'placement_error.png')

    # Each robot's map, in the world frame through its fitted placement.
    for h in agents:
        if h not in T_fit:
            continue
        mp = read_csv(os.path.join(raw, h, 'eval', 'mappoints.csv'))
        if not mp:
            continue
        T = T_fit[h]
        fig = plt.figure(figsize=(12, 5.4))
        gs = fig.add_gridspec(1, 2, width_ratios=[1.25, 1], wspace=0.28)
        ax3 = fig.add_subplot(gs[0], projection='3d')
        ax2 = fig.add_subplot(gs[1])
        room(ax2)
        handles = []
        clipped = 0
        for a in agents:
            main = has_main(mp)
            sel = [r for r in mp if (r['in_main_map'] == '1' or not main) and r['origin_agent'] == a]
            if not sel:
                continue
            P = np.array([[float(r['x']), float(r['y']), float(r['z'])] for r in sel])
            W = P @ T[:3, :3].T + T[:3, 3]
            inside = in_room(W)
            clipped += int((~inside).sum())
            W = W[inside]
            own_pts = a == h
            ax3.scatter(W[:, 0], W[:, 1], W[:, 2], s=0.6 if own_pts else 1.2, color=color[a],
                        alpha=0.45 if own_pts else 0.8, lw=0, depthshade=False)
            ax2.scatter(W[:, 0], W[:, 1], s=0.6 if own_pts else 1.2, color=color[a],
                        alpha=0.45 if own_pts else 0.8, lw=0)
            handles.append(Line2D([], [], ls='', marker='o', ms=5, color=color[a],
                                  label=f'points from {a}' + (' (own)' if own_pts else f' ({len(W)})')))
        for a in agents:
            kfs = keyframes(kf_rows[h], origin=a)
            if not kfs:
                continue
            P = np.array([k[1][:3, 3] for k in kfs]) @ T[:3, :3].T + T[:3, 3]
            own_kf = a == h
            mk = 'o' if own_kf else '^'
            ax3.scatter(P[:, 0], P[:, 1], P[:, 2], s=18, marker=mk, color=color[a], edgecolor=INK, lw=0.5, depthshade=False)
            ax2.scatter(P[:, 0], P[:, 1], s=18, marker=mk, color=color[a], edgecolor=INK, lw=0.5, zorder=3)
            handles.append(Line2D([], [], ls='', marker=mk, ms=6, color=color[a], mec=INK, mew=0.5,
                                  label=f'keyframes from {a} ({len(P)})' + (' (own)' if own_kf else '')))
        xlim(ax3, 0)
        ax3.set_zlim(-0.1, 2.1)
        ax3.set_zticks([0, 1, 2])
        box_aspect3d(ax3)
        ax3.view_init(elev=28, azim=-60)
        ax3.set_xlabel('x (m)', labelpad=2)
        ax3.set_ylabel('y (m)', labelpad=2)
        ax3.set_zlabel('z (m)', labelpad=-2)
        ax3.tick_params(labelsize=7, pad=0)
        ax3.set_title(f"{h}'s map in 3D")
        ax2.set_title(f"{h}'s map from above")
        fig.legend(handles=handles, loc='lower center', ncol=3, bbox_to_anchor=(0.5, -0.06))
        if clipped:
            fig.text(0.99, 0.01, f'{clipped} far-off points outside the room not drawn', ha='right', fontsize=7, color=MUTED)
        figs[f'map_{h}'] = save(fig, figdir, f'map_{h}.png')
    return figs


# ── html ───────────────────────────────────────────────────────────────────
def write_html(run, spec, agents, color, metrics, figs, env, t0, t_end, dt):
    e = html.escape
    mode = next((l.split(': ', 1)[1] for l in env.splitlines() if l.startswith('mode: ')), 'test')
    A = metrics['agents']
    P = metrics['pairs']
    dur = f'{t_end - t0:.0f} s' if t_end else '?'

    def chip(a):
        return f'<span class="chip" style="background:{color[a]}"></span>{e(a)}'

    rows = []
    for a in agents:
        m = A[a]
        s = m.get('summary', {})
        rows.append(
            f'<tr><td>{chip(a)}</td><td>{m.get("own_keyframes", 0)}</td>'
            f'<td>{cm(m.get("kf_fitted"))}</td><td>{cm(m.get("kf_fitted"), "max")}</td>'
            f'<td>{cm(m.get("kf_anchored"))}</td><td>{cm(m.get("online"))}</td>'
            f'<td>{100 * m.get("tracking_ok_fraction", 0):.0f} %</td>'
            f'<td>{m.get("track_ms_median", float("nan")):.0f} / {m.get("track_ms_p95", float("nan")):.0f}</td>'
            f'<td>{e(s.get("foreign_keyframes", "–"))} / {e(s.get("foreign_points", "–"))}</td></tr>')
    own_table = ''.join(rows)

    prow = []
    for h in agents:
        for p in agents:
            if h == p:
                continue
            m = P[f'{h}<-{p}']
            first = f'{m["first_alignment_s"]:.0f} s' if 'first_alignment_s' in m else 'never'
            fin = (f'{100 * m["final_alignment_trans_err_m"]:.1f} cm / {m["final_alignment_rot_err_deg"]:.2f}°'
                   if 'final_alignment_trans_err_m' in m else '–')
            prow.append(
                f'<tr><td>{chip(h)}</td><td>{chip(p)}</td><td>{first}</td><td>{m["alignments"]}</td><td>{fin}</td>'
                f'<td>{m["placed_keyframes"]}</td><td>{cm(m.get("placement"), "median")}</td>'
                f'<td>{cm(m.get("placement"), "p95")}</td><td>{m["verifications_passed"]} / {m["verifications"]}</td>'
                f'<td>{m["map_points_from_peer"]}</td></tr>')
    pair_table = ''.join(prow)

    def img(key, alt):
        return f'<figure><img src="{figs[key]}" alt="{e(alt)}"></figure>' if key in figs else ''

    paths = ''.join(img(f'traj_{a}', f'{a} path vs ground truth') for a in agents)
    goal_rows = []
    for a in agents:
        g = read_csv(os.path.join(run, 'raw', 'online', f'{a}_goals.csv'))
        for r in g:
            goal_rows.append(f'<tr><td>{chip(a)}</td><td>{float(r["wall"]) - t0:.0f} s</td><td>{e(r["frame"])}</td>'
                             f'<td>{float(r["x"]):.2f}</td><td>{float(r["y"]):.2f}</td>'
                             f'<td>{math.degrees(float(r["yaw"])):.0f}°</td></tr>')
    goals = ('<h3>Goals given</h3><table><tr><th>robot</th><th>time</th><th>frame</th><th>x (m)</th>'
             '<th>y (m)</th><th>heading</th></tr>' + ''.join(goal_rows) + '</table>') if goal_rows else ''
    maps = ''.join(img(f'map_{a}', f'{a} map') for a in agents)
    crow = []
    for a in agents:
        c = A[a].get('coverage')
        if not c:
            continue
        crow.append(f'<tr><td>{chip(a)}</td><td>{c["own_points"]}</td><td>{c["integrated_points"]}</td>'
                    f'<td>{c["own_voxels"]}</td><td>{c["integrated_voxels"]}</td><td>+{100 * c["voxel_gain"]:.0f}&nbsp;%</td>'
                    f'<td>{c["own_area_m2"]:.1f}</td><td>{c["integrated_area_m2"]:.1f}</td><td>+{100 * c["area_gain"]:.0f}&nbsp;%</td>'
                    f'<td>{100 * c["team_share_own"]:.0f}&nbsp;% → {100 * c["team_share_integrated"]:.0f}&nbsp;%</td></tr>')
    cov_table = ''.join(crow)
    cov_maps = ''.join(img(f'coverage_{a}', f'{a} own vs integrated map') for a in agents)
    when = datetime.datetime.fromtimestamp(t0).strftime('%Y-%m-%d %H:%M')
    doc = f'''<!doctype html>
<html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width, initial-scale=1">
<title>Multi-Robot Sim Test</title>
<style>
  @page {{ size: A4; margin: 14mm; }}
  :root {{ --ink: {INK}; --ink2: {INK2}; --muted: {MUTED}; --line: {GRID}; --surface: #ffffff; --panel: {SURFACE}; }}
  body {{ font-family: "DejaVu Sans", Helvetica, sans-serif; color: var(--ink); background: var(--surface);
         max-width: 1040px; margin: 0 auto; padding: 24px 16px; font-size: 10pt; line-height: 1.45; }}
  h1 {{ font-size: 20pt; margin: 0 0 4px; }}
  h2 {{ font-size: 13pt; margin: 28px 0 8px; border-bottom: 1.5px solid var(--line); padding-bottom: 3px; break-after: avoid; }}
  .sub {{ color: var(--ink2); margin-bottom: 14px; }}
  table {{ border-collapse: collapse; width: 100%; margin: 8px 0 12px; font-size: 9pt; break-inside: avoid; }}
  th, td {{ border-bottom: 1px solid var(--line); padding: 5px 6px; text-align: right; }}
  th:first-child, td:first-child, th:nth-child(2), td:nth-child(2) {{ text-align: left; }}
  th {{ color: var(--ink2); font-weight: 600; background: var(--panel); }}
  .chip {{ display: inline-block; width: 10px; height: 10px; border-radius: 2px; margin-right: 6px; vertical-align: -1px; }}
  figure {{ margin: 10px 0 18px; break-inside: avoid; }}
  figure img {{ max-width: 100%; }}
  .note {{ color: var(--ink2); font-size: 9pt; }}
  dl {{ display: grid; grid-template-columns: max-content 1fr; gap: 4px 14px; font-size: 9pt; }}
  dt {{ color: var(--ink2); }}
  pre {{ background: var(--panel); border: 1px solid var(--line); padding: 8px; font-size: 8pt; white-space: pre-wrap; }}
  .scroll {{ overflow-x: auto; }}
  @media print {{ .scroll {{ overflow: visible; }} table {{ font-size: 7.5pt; }} th, td {{ padding: 3px 4px; }} }}
</style></head><body>
<h1>{"Multi-robot navigation session" if "nav" in mode else "Multi-robot simulation test"}</h1>
<div class="sub">{e(spec["name"])} · method <b>{e(metrics["method"])}</b> · {len(agents)} robots · {when} · {"session" if "nav" in mode else "drive"} {dur}</div>
<p>{e(spec.get("description", ""))}</p>
{img("overview", "route and true paths")}
{goals}

<h2>1. Each robot's own path</h2>
<p>Errors in centimetres (RMS unless marked). <b>Fitted</b>: the final keyframe trajectory after SLAM's own
optimisation, rigidly fitted to the true camera path (the usual ATE: shape only). <b>Anchored</b>: the same
keyframes placed in the world through the robot's first keyframe, without fitting, so drift since the start
counts. <b>Live</b>: the pose the robot used while driving (rear axle, 2D), fitted.</p>
<div class="scroll"><table><tr><th>robot</th><th>keyframes</th><th>fitted</th><th>fitted max</th><th>anchored</th>
<th>live</th><th>tracking OK</th><th>tracking ms (median / p95)</th><th>received keyframes / points</th></tr>
{own_table}</table></div>
{img("accuracy", "path accuracy")}
{paths}

<h2>2. Map merging</h2>
<p>Each row is one robot (host) holding another robot's (peer's) map. <b>Alignment error</b>: the last
map-to-map transform the host estimated, against the true one (from both maps' first keyframes, whose true
poses are known, so it does not inherit drift). <b>Placement</b>: where the peer's keyframes sit in the host's
map, against where the peer's camera truly was. <b>Checks</b>: geometric match verifications passed / tried.</p>
<div class="scroll"><table><tr><th>host</th><th>peer</th><th>first aligned</th><th>alignments</th><th>final alignment error</th>
<th>placed key&shy;frames</th><th>placement median (cm)</th><th>placement p95 (cm)</th><th>checks passed</th><th>peer points held</th></tr>
{pair_table}</table></div>
{img("alignment", "alignment error over time")}
{img("placement", "placement error")}

<h2>3. Coverage: own map vs integrated map</h2>
<p>What sharing adds to each robot's map. <b>Own</b>: only the map points the robot made itself, which is what a
single-robot method would have built on the same drive. <b>Integrated</b>: its whole map at the end, own points plus those
received from the others. Coverage counts space, not points: <b>cubes</b> are 10 cm cubes of the room that contain at
least one map point, <b>floor area</b> the 10 cm floor cells with a point above them (any height). <b>Team share</b>:
of everything the robots mapped together (all robots' own maps), the part this robot's map covers, alone → with sharing.
All maps are placed in the room through each robot's fitted path; points outside the room are left out.</p>
<div class="scroll"><table><tr><th>robot</th><th>own points</th><th>integrated points</th><th>own cubes</th>
<th>integrated cubes</th><th>gain</th><th>own floor (m²)</th><th>integrated floor (m²)</th><th>gain</th><th>team share</th></tr>
{cov_table}</table></div>
{img("coverage", "coverage summary")}
{cov_maps}

<h2>4. Each robot's map</h2>
<p>Map points (small dots) and keyframes (circles: the robot's own; triangles: received) in each robot's main map at
the end of the run, coloured by the robot they came from, shown in the world frame through the robot's fitted
placement.</p>
{maps}

<h2>5. Data and settings</h2>
<dl><dt>run folder</dt><dd>{e(run)}</dd>
<dt>raw data</dt><dd>raw/: spec.json, gt/&lt;robot&gt;_gt.csv (truth every simulation step), &lt;robot&gt;/eval/*.csv
(keyframes, map points, alignments, verifications, timing), online/&lt;robot&gt;_online.csv (live poses), logs</dd>
<dt>tables</dt><dd>tables/: per-keyframe, live-pose, alignment and placement errors, coverage (CSV)</dd>
<dt>metrics</dt><dd>metrics.json (everything in this report as numbers)</dd>
<dt>clock offset</dt><dd>{dt * 1000:.0f} ms between image stamps and the truth log (fitted)</dd>
<dt>re-score</dt><dd>~/venvs/env_isaaclab/bin/python ~/workspaces/ros2_ws/src/jetracer_sim/eval/ma_report.py &lt;run folder&gt;</dd></dl>
<pre>{e(env)}</pre>
</body></html>'''
    with open(os.path.join(run, 'report.html'), 'w') as f:
        f.write(doc)


if __name__ == '__main__':
    main()
