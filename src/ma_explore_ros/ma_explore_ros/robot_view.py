# Copied from ma_explore (6789422+changes) by ma_explore.robot_package: edit it there, not here.
"""Live view of exploring robots without ground truth (real robots): one panel
per robot, in that robot's own map frame (a window; it only watches).

    python -m ma_explore.robot_view --ros-args -p agents:="robot1 [robot2 ...]" [-p period:=0.5]

Each panel shows what the robot's planner decided on: its known map (its own
SLAM grid plus merged teammates' grids, /ma_explore/<a>/known_map from
ros_planner; before the first decision the robot's own /<a>/map), its live
pose and path (/<a>/orb_slam3/odom), merged teammates where it places them,
the goals it was offered (small circles: frontiers; small diamonds: revisit
poses) and what it chose: a star for a new goal, a big diamond with a heading
arrow for a REVISIT pose, a ring with the seconds left for WAIT. Under the map:
the last action, how often each kind was chosen, links per teammate and SLAM
backlog.
"""
import json
import math
import os
import time

import matplotlib

if not os.environ.get('MPLBACKEND'):
    matplotlib.use('TkAgg')
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import rclpy  # noqa: E402
from nav_msgs.msg import OccupancyGrid, Odometry  # noqa: E402
from rcl_interfaces.msg import ParameterDescriptor  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy  # noqa: E402
from std_msgs.msg import String  # noqa: E402

COLORS = ['#2a78d6', '#e8833a', '#1aa37a', '#c0458a', '#8a6fd1', '#b8a12a']     # as isaac_view.py
KINDS = ('frontier', 'keep', 'wait', 'revisit', 'loop')


def describe(act):
    k = act.get('kind')
    if k is None:
        return 'no decision yet'
    if k == 'revisit':
        return f"REVISIT {act.get('mate', '')}'s pose"
    if k == 'loop':
        return 'LOOP: own old pose'
    return {'wait': 'WAIT', 'keep': 'keep goal', 'stay': 'stay (no goal)', 'frontier': 'new goal'}.get(k, k)


FREE, OCC, UNKNOWN = np.array([0.97, 0.97, 0.97]), np.array([0.25, 0.25, 0.27]), np.array([0.80, 0.80, 0.80])


def grid_image(m):
    d = np.asarray(m.data, np.int8).reshape(m.info.height, m.info.width)
    im = np.empty(d.shape + (3,))
    im[:] = UNKNOWN
    im[(d >= 0) & (d < 50)] = FREE
    im[d >= 50] = OCC
    x0, y0, r = m.info.origin.position.x, m.info.origin.position.y, m.info.resolution
    return im, (x0, x0 + m.info.width * r, y0, y0 + m.info.height * r), d


class RobotView(Node):
    def __init__(self):
        super().__init__('ma_explore_robot_view')
        dyn = ParameterDescriptor(dynamic_typing=True)
        p = lambda n, v: type(v)(self.declare_parameter(n, v, dyn).value)   # noqa: E731
        self.agents = p('agents', 'robot1').split()
        self.period = p('period', 0.5)
        self.own, self.known, self.odom, self.action, self.action_t = {}, {}, {}, {}, {}
        self.trail = {a: [] for a in self.agents}
        self.counts = {a: dict.fromkeys(KINDS, 0) for a in self.agents}
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        for a in self.agents:
            self.create_subscription(OccupancyGrid, f'/{a}/map', lambda m, a=a: self.own.__setitem__(a, m), latched)
            self.create_subscription(OccupancyGrid, f'/ma_explore/{a}/known_map',
                                     lambda m, a=a: self.known.__setitem__(a, m), latched)
            self.create_subscription(Odometry, f'/{a}/orb_slam3/odom', lambda m, a=a: self.on_odom(a, m), 10)
            self.create_subscription(String, f'/ma_explore/{a}/action', lambda m, a=a: self.on_action(a, m), latched)
        self.t0 = time.monotonic()
        self.setup()

    def on_odom(self, a, m):
        q = m.pose.pose.orientation
        x, y = m.pose.pose.position.x, m.pose.pose.position.y
        self.odom[a] = (x, y, 2 * math.atan2(q.z, q.w))
        tr = self.trail[a]
        if not tr or math.hypot(x - tr[-1][0], y - tr[-1][1]) > 0.02:
            tr.append((x, y))

    def on_action(self, a, m):
        d = json.loads(m.data)
        if self.action.get(a, {}).get('decision') != d.get('decision'):
            k = 'keep' if d['kind'] == 'stay' else d['kind']
            self.counts[a][k] = self.counts[a].get(k, 0) + 1
        self.action[a], self.action_t[a] = d, time.monotonic()

    # ── drawing ──────────────────────────────────────────────────────────
    def setup(self):
        n = len(self.agents)
        self.fig, axs = plt.subplots(2, n, figsize=(6.4 * n, 7.6), squeeze=False,
                                     gridspec_kw={'height_ratios': [6.0, 1.0]})
        self.fig.canvas.manager.set_window_title('ma_explore robot view: ' + ' '.join(self.agents))
        self.ax = {a: axs[0, i] for i, a in enumerate(self.agents)}
        self.panel = {}
        for i, a in enumerate(self.agents):
            axs[1, i].axis('off')
            self.panel[a] = axs[1, i].text(0.0, 1.0, '', color=COLORS[i % len(COLORS)], fontsize=9.5,
                                           family='monospace', va='top', transform=axs[1, i].transAxes)
        plt.show(block=False)

    def draw(self, i, a):
        ax, c = self.ax[a], COLORS[i % len(COLORS)]
        ax.clear()
        m = self.known.get(a) or self.own.get(a)
        act = self.action.get(a, {})
        el = time.monotonic() - self.t0
        ax.set_title(f'{a}  ({"what the policy saw" if a in self.known else "own SLAM grid"})   {el:.0f} s',
                     fontsize=10)
        if m is not None:
            im, ext, d = grid_image(m)
            ax.imshow(im, origin='lower', extent=ext, interpolation='nearest')
            known = np.argwhere(d >= 0)
            if len(known):
                r = m.info.resolution
                (r0, c0), (r1, c1) = known.min(0), known.max(0)
                ax.set_xlim(ext[0] + c0 * r - 0.8, ext[0] + (c1 + 1) * r + 0.8)
                ax.set_ylim(ext[2] + r0 * r - 0.8, ext[2] + (r1 + 1) * r + 0.8)
        ax.set_aspect('equal')
        for x, y, k in act.get('offered', []):
            ax.plot([x], [y], 'D' if k in ('revisit', 'loop') else 'o', ms=5, mfc='none', mec=c, mew=1.2, alpha=0.8)
        if self.trail[a]:
            ax.plot(*zip(*self.trail[a]), '-', color=c, lw=2)
        for name, (mx, my, myaw) in act.get('mates', {}).items():
            j = self.agents.index(name) if name in self.agents else len(self.agents)
            mc = COLORS[j % len(COLORS)]
            ax.plot([mx], [my], marker=(3, 0, math.degrees(myaw) - 90), ms=12, color=mc, mec='black')
            ax.text(mx + 0.15, my + 0.15, name, color=mc, fontsize=8)
        pose = self.odom.get(a) or (tuple(act['pose']) if 'pose' in act else None)
        kind = act.get('kind', 'frontier')
        if pose is not None:
            x, y, yaw = pose
            ax.arrow(x, y, 0.25 * math.cos(yaw), 0.25 * math.sin(yaw), width=0.06, head_width=0.18,
                     head_length=0.12, length_includes_head=True, fc=c, ec='black', lw=0.8, zorder=5)
            g = act.get('goal')
            if g is not None:
                revisit = kind in ('revisit', 'loop') or (kind in ('keep', 'wait') and act.get('goal_yaw') is not None)
                ax.plot([x, g[0]], [y, g[1]], ':', color=c, lw=1.5)
                ax.plot([g[0]], [g[1]], 'D' if revisit else '*', ms=11 if revisit else 16, color=c, mec='black',
                        mew=1.0, zorder=6)
                if revisit and act.get('goal_yaw') is not None:
                    gy = act['goal_yaw']
                    ax.annotate('', xy=(g[0] + 0.35 * math.cos(gy), g[1] + 0.35 * math.sin(gy)), xytext=tuple(g),
                                arrowprops=dict(arrowstyle='-|>', color=c, lw=2.2, mutation_scale=14), zorder=7)
            left = act.get('wait_s', 0.0) - (time.monotonic() - self.action_t.get(a, 0.0))
            if kind == 'wait' and left > 0:
                ax.plot([x], [y], 'o', ms=30, mfc='none', mec=c, mew=3, zorder=4)
                ax.text(x + 0.22, y + 0.22, f'WAIT {left:.0f} s', color=c, fontsize=9, weight='bold', zorder=7,
                        bbox=dict(fc='white', ec=c, alpha=0.85, pad=1.5))
        ax.set_xlabel('x (m)')
        n = self.counts[a]
        links = ' '.join(f'{k}:{v}' for k, v in act.get('links', {}).items()) or '-'
        self.panel[a].set_text(
            f"#{act.get('decision', 0):<3d} {describe(act):24s} backlog {act.get('backlog', 0.0):.1f} s\n"
            f"new {n['frontier']}  keep {n['keep']}  wait {n['wait']}  revisit {n['revisit']}  loop {n['loop']}   links {links}\n"
            f"offered: {sum(1 for o in act.get('offered', []) if o[2] == 'frontier')} frontiers, "
            f"{sum(1 for o in act.get('offered', []) if o[2] == 'revisit')} revisit poses, "
            f"{sum(1 for o in act.get('offered', []) if o[2] == 'loop')} own poses (loops)")

    def update(self):
        for i, a in enumerate(self.agents):
            self.draw(i, a)
        self.fig.canvas.draw_idle()


def main():
    rclpy.init()
    node = RobotView()
    try:
        last = 0.0
        while rclpy.ok() and plt.fignum_exists(node.fig.number):
            rclpy.spin_once(node, timeout_sec=0.05)
            if time.monotonic() - last >= node.period:
                last = time.monotonic()
                node.update()
                plt.pause(0.001)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
