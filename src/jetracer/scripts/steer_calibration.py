#!/usr/bin/env python3
"""Measure how the car really turns for a given turn command (installed as `steer_calibration`).

    ros2 run jetracer steer_calibration --ros-args -p agent_name:=agent_2

Nav2 sends speed v and turn rate w on /<agent>/cmd_vel and expects the car to
drive a curve of radius v / w. On the real car the motor board turns w into a
servo position with its own, uncalibrated coefficients, so the curve it
drives can differ. This drives a series of fixed (v, w) commands, a few
seconds each, left and right (reversing back along the same arc after
each, so the car stays on a small patch), and measures the curve driven from
the SLAM pose of the rear axle (/<agent>/orb_slam3/odom, so the robot needs
`run_slam3.sh --teleop --grid`). It prints, per command, the commanded and
the driven radius, and at the end the tightest radius the car can drive and
how far gentle curves are off, with the settings that follow from them.

Needs about 2 x 2 m of free floor around the car, and the floor in view of
the camera. The car stops between tests and on Ctrl-C, and a test is
abandoned if the SLAM pose stops coming (tracking lost).

Parameters: agent_name, speed (m/s, 0.15), rates (rad/s, the turn rates to
test; their sign is the direction), duration (s per test, 4.0), settle (s at
the start of each test that are not measured, while the steering moves, 1.0),
csv (file for the raw results, default ~/steer_calibration_<time>.csv).
"""
import math
import os
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node

WHEELBASE = 0.145   # m, the real car (README section 8)
MAX_STEER = 0.6     # rad, its steering limit


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class SteerCalibration(Node):
    def __init__(self):
        super().__init__('steer_calibration')
        self.agent = self.declare_parameter('agent_name', os.environ.get('AGENT_NAME', 'agent_0')).value
        self.speed = float(self.declare_parameter('speed', 0.15).value)
        self.rates = [float(r) for r in self.declare_parameter(
            'rates', [0.2, -0.2, 0.4, -0.4, 0.6, -0.6, 0.8, -0.8]).value]
        self.duration = float(self.declare_parameter('duration', 4.0).value)
        self.settle = float(self.declare_parameter('settle', 1.0).value)
        self.csv = os.path.expanduser(self.declare_parameter(
            'csv', time.strftime('~/steer_calibration_%Y%m%d_%H%M%S.csv')).value)
        self.pub = self.create_publisher(Twist, f'/{self.agent}/cmd_vel', 10)
        self.create_subscription(Odometry, f'/{self.agent}/orb_slam3/odom', self.on_odom, 50)
        self.poses = []          # (wall time, x, y, yaw)

    def on_odom(self, m):
        p = m.pose.pose
        self.poses.append((time.monotonic(), p.position.x, p.position.y, yaw_of(p.orientation)))

    def wait(self, seconds, cmd=None):
        """Spin for `seconds`, sending `cmd` at 20 Hz if given."""
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            if cmd is not None:
                self.pub.publish(cmd)
            rclpy.spin_once(self, timeout_sec=0.05)

    def stop(self, seconds=1.5):
        self.wait(seconds, Twist())

    def pose_fresh(self):
        return self.poses and time.monotonic() - self.poses[-1][0] < 0.5

    def run_test(self, w):
        """Drive (speed, w); returns (driven curvature, driven speed) or None."""
        cmd = Twist()
        cmd.linear.x, cmd.angular.z = self.speed, w
        self.poses.clear()
        t0 = time.monotonic()
        while time.monotonic() - t0 < self.duration:
            self.wait(0.05, cmd)
            if time.monotonic() - t0 > 0.5 and not self.pose_fresh():
                self.stop()
                self.get_logger().warn('no SLAM pose (tracking lost?): test abandoned')
                return None
        self.stop()
        seg = [p for p in self.poses if t0 + self.settle <= p[0] <= t0 + self.duration]
        # Reverse along the same arc (same steering, opposite direction) back
        # to about where the test started, so the car stays on a small patch.
        back = Twist()
        back.linear.x, back.angular.z = -self.speed, -w
        self.wait(self.duration, back)
        self.stop()
        if len(seg) < 10:
            self.get_logger().warn(f'only {len(seg)} poses measured: test abandoned')
            return None
        length, turned = 0.0, 0.0
        for a, b in zip(seg, seg[1:]):
            length += math.hypot(b[1] - a[1], b[2] - a[2])
            turned += math.remainder(b[3] - a[3], 2 * math.pi)
        if length < 0.05:
            self.get_logger().warn(f'the car moved only {length * 100:.0f} cm: test abandoned')
            return None
        return turned / length, length / (seg[-1][0] - seg[0][0])


def radius(k):
    return math.inf if abs(k) < 1e-6 else 1.0 / abs(k)


def fmt_r(r):
    return '   inf' if math.isinf(r) else f'{r:6.2f}'


def main():
    rclpy.init()
    n = SteerCalibration()
    print(f'Steering calibration for /{n.agent}: {len(n.rates)} tests at {n.speed:.2f} m/s, '
          f'{n.duration:.0f} s each. Keep 2 x 2 m around the car free; Ctrl-C stops.')
    t0 = time.monotonic()
    while not n.pose_fresh() and time.monotonic() - t0 < 10:
        rclpy.spin_once(n, timeout_sec=0.1)
    if not n.pose_fresh():
        print(f'No SLAM pose on /{n.agent}/orb_slam3/odom. Start the robot with '
              f'`./run_slam3.sh --teleop --grid`, wait for "camera mount fitted to the floor", '
              f'and check ROS_DOMAIN_ID.')
        return
    k_max = math.tan(MAX_STEER) / WHEELBASE
    rows = []
    try:
        for w in n.rates:
            k_cmd = w / n.speed
            print(f'  turn rate {w:+.2f} rad/s (commanded radius {fmt_r(radius(k_cmd))} m '
                  f'{"left" if w > 0 else "right"}) ...', end='', flush=True)
            res = n.run_test(w)
            if res is None:
                print(' abandoned')
                continue
            k, v = res
            rows.append((w, k_cmd, k, v))
            print(f' drove radius {fmt_r(radius(k))} m, {k / k_cmd * 100:4.0f} % of the commanded curvature, '
                  f'{v:.2f} m/s')
    except KeyboardInterrupt:
        pass
    finally:
        n.stop(0.5)
    if not rows:
        return
    with open(n.csv, 'w') as f:
        f.write('turn_rate,commanded_curvature,driven_curvature,driven_speed\n')
        for r in rows:
            f.write(','.join(f'{x:.5f}' for x in r) + '\n')

    print(f'\nRaw results: {n.csv}')
    print(f'Geometry says: tightest radius {1 / k_max:.2f} m (wheelbase {WHEELBASE} m, steering {MAX_STEER} rad).')
    for side, sign in (('left', 1), ('right', -1)):
        k_side = [abs(k) for w, _, k, _ in rows if w * sign > 0 and k * sign > 0]
        if k_side:
            print(f'Tightest radius driven ({side}): {1 / max(k_side):.2f} m')
    # Gentle curves (below the steering limit): driven / commanded curvature.
    gentle = [(kc, k) for _, kc, k, _ in rows if abs(kc) < 0.8 * k_max]
    if gentle:
        g = sum(kc * k for kc, k in gentle) / sum(kc * kc for kc, _ in gentle)
        print(f'Gentle curves: the car drives {g * 100:.0f} % of the commanded curvature '
              f'(100 % = exact; from {len(gentle)} tests).')
    k_all = [abs(k) for _, _, k, _ in rows]
    r_min = 1 / max(k_all)
    print(f'\nSuggested planner setting (nav2_jetracer.yaml, GridBased): '
          f'minimum_turning_radius: {max(0.35, 1.3 * r_min):.2f}   (30 % above the tightest driven radius)')


if __name__ == '__main__':
    main()
