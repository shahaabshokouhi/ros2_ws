"""Which trained network a policy file holds, and whether it loads here.

    ros2 run ma_explore_ros policy_info [FILE]     # default: the package's policy/policy.pt
"""
import os
import sys
import time


def main():
    import math

    import numpy as np
    try:
        import torch  # noqa: F401
    except ImportError:
        print('PyTorch is not installed for this Python (see the package README: pip3 install --user torch)')
        sys.exit(1)
    from .live import Decider, Robot
    from .policy_io import load_policy_file
    path = sys.argv[1] if len(sys.argv) > 1 else None
    if path is None:
        from ament_index_python.packages import get_package_share_directory
        share = get_package_share_directory('ma_explore_ros')
        ws = os.path.abspath(os.path.join(share, '..', '..', '..', '..'))
        path = os.path.join(ws, 'src', 'ma_explore_ros', 'policy', 'policy.pt')
    if not os.path.exists(path):
        print(f'no policy file at {path}')
        sys.exit(1)
    os.environ.setdefault('MA_EXPLORE_NO_COMPILE', '1')
    pol, cfg, H, meta = load_policy_file(path, 'cpu')
    print(f'{path}\n  {meta}')
    print(f'  trained for {cfg.n_agents} robots, sensor {cfg.fov_deg:.0f} deg / {cfg.sensor_range / 10:.1f} m, '
          f'stage {"2 (SLAM-aware)" if cfg.slam_model else "1"}')
    ring = np.array([[math.cos(a), math.sin(a)] for a in np.linspace(0, 2 * math.pi, 60)])
    d = Decider(pol, cfg, cfg.n_agents, H, 'cpu')
    robots = [Robot(ring * 0.5, ring * 1.5, (0, 0, 0))] + [Robot(known=False)] * (cfg.n_agents - 1)
    d.decide(robots, [], 0)
    t0 = time.perf_counter()
    for k in range(5):
        r = d.decide(robots, [], k + 1)
    print(f'  test decision: {r["kind"]}, {1000 * (time.perf_counter() - t0) / 5:.0f} ms per decision on this CPU')


if __name__ == '__main__':
    main()
