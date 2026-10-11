"""Route geometry for multi-robot simulation tests (plain Python, no ROS).

A test spec (config/ma_*.json) names the robots, one closed route they all
drive, where along it each one starts, and how far they drive:

    {"name": "room_loop3",
     "agents": ["sim_1", "sim_2", "sim_3"],
     "route": {"type": "rounded_rect", "xmin": -1.7, "xmax": 1.9,
               "ymin": -1.6, "ymax": 1.9, "radius": 0.6},
     "start_fraction": [0.0, 0.333, 0.667],
     "laps": 1.5, "speed": 0.15}

The route is counter-clockwise in Isaac's world frame (x, y on the floor),
starting at the middle of its bottom edge. Start fractions are measured along
it from there; each robot starts on the route, facing along it, so robots a
third of a lap apart keep that spacing and never meet.
"""
import json
import math


def load_spec(path):
    with open(path) as f:
        spec = json.load(f)
    spec.setdefault('laps', 1.0)
    spec.setdefault('speed', 0.15)
    spec.setdefault('start_fraction', [k / len(spec['agents']) for k in range(len(spec['agents']))])
    return spec


def route_points(route, step=0.02):
    """The route as a closed polyline [(x, y), ...] with about `step` spacing."""
    if route['type'] != 'rounded_rect':
        raise ValueError(f"unknown route type {route['type']}")
    x0, x1, y0, y1, r = route['xmin'], route['xmax'], route['ymin'], route['ymax'], route['radius']
    pts = []

    def line(ax, ay, bx, by):
        n = max(1, int(math.hypot(bx - ax, by - ay) / step))
        pts.extend((ax + (bx - ax) * i / n, ay + (by - ay) * i / n) for i in range(n))

    def arc(cx, cy, a0):
        n = max(1, int(r * math.pi / 2 / step))
        pts.extend((cx + r * math.cos(a0 + math.pi / 2 * i / n),
                    cy + r * math.sin(a0 + math.pi / 2 * i / n)) for i in range(n))

    xm = (x0 + x1) / 2
    line(xm, y0, x1 - r, y0)                  # bottom edge, from its middle, heading +x
    arc(x1 - r, y0 + r, -math.pi / 2)         # bottom-right corner
    line(x1, y0 + r, x1, y1 - r)              # right edge, +y
    arc(x1 - r, y1 - r, 0.0)
    line(x1 - r, y1, x0 + r, y1)              # top edge, -x
    arc(x0 + r, y1 - r, math.pi / 2)
    line(x0, y1 - r, x0, y0 + r)              # left edge, -y
    arc(x0 + r, y0 + r, math.pi)
    line(x0 + r, y0, xm, y0)                  # back to the start
    return pts


def cumulative(pts):
    """Arc length at each point and the total length of the closed route."""
    s = [0.0]
    for (ax, ay), (bx, by) in zip(pts, pts[1:]):
        s.append(s[-1] + math.hypot(bx - ax, by - ay))
    total = s[-1] + math.hypot(pts[0][0] - pts[-1][0], pts[0][1] - pts[-1][1])
    return s, total


def pose_at(pts, s_list, total, s):
    """(x, y, yaw) on the route at arc length s (wraps around)."""
    s %= total
    i = max(0, min(len(pts) - 1, next((k for k, v in enumerate(s_list) if v > s), len(pts)) - 1))
    a, b = pts[i], pts[(i + 1) % len(pts)]
    seg = math.hypot(b[0] - a[0], b[1] - a[1]) or 1e-9
    f = (s - s_list[i]) / seg
    return a[0] + f * (b[0] - a[0]), a[1] + f * (b[1] - a[1]), math.atan2(b[1] - a[1], b[0] - a[0])


def start_poses(spec):
    """[(x, y, yaw_rad)] per agent: the spec's "starts" ([[x, y, yaw_deg], ...])
    if given, else on the route at each start fraction."""
    if 'starts' in spec:
        return [(x, y, math.radians(yaw)) for x, y, yaw in spec['starts']]
    pts = route_points(spec['route'])
    s_list, total = cumulative(pts)
    return [pose_at(pts, s_list, total, f * total) for f in spec['start_fraction']]


if __name__ == '__main__':
    import sys
    sp = load_spec(sys.argv[1])
    # For run_isaac.py --poses: "x,y,yaw_deg;..."
    print(';'.join(f'{x:.4f},{y:.4f},{math.degrees(a):.2f}' for x, y, a in start_poses(sp)))
