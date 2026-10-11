#!/usr/bin/env python3
"""Drive several simulated cars along one route, repeatably (multi-robot tests).

    ros2 run jetracer_sim route_driver --ros-args -p spec:=<ma_*.json> -p out:=<dir>

Each car follows the spec's route (jetracer_sim.routes) with pure pursuit,
steering on its TRUE pose (Isaac's /<agent>/ground_truth/odom placed at the
car's start pose), so every run drives exactly the same motion whatever SLAM
does: the SLAM stacks are measured, not used for driving. All cars start
together, drive at the same speed and stop after `laps` laps of their own.

Writes, per agent, <out>/<agent>_online.csv: the SLAM node's live pose
estimate (/<agent>/orb_slam3/odom, rear axle in map_nav) with the wall time it
arrived, for comparing the online estimate with ground truth afterwards.
Exits when every car has finished (or after `timeout` s).
"""
import math
import os
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node

from jetracer_sim import routes

REAR = 0.0698        # rear axle behind gt_base_link (m)
LOOKAHEAD = 0.35     # pure pursuit lookahead (m)
MIN_RADIUS = 0.35    # tightest curve asked of the car (m); the sim car manages about 0.35


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class Car:
    def __init__(self, name, start, pts, s_list, total):
        self.name, self.start = name, start
        self.pts, self.s_list, self.total = pts, s_list, total
        self.pose = None          # (t, x, y, yaw) of the rear axle in Isaac's world frame
        self.idx = None           # nearest route point
        self.travel = 0.0         # arc length driven along the route (m)
        self.done = False

    def on_gt(self, m):
        p = m.pose.pose
        x0, y0, a0 = self.start
        # Odometry starts at zero at the car's start pose, in its start heading.
        dx, dy, da = p.position.x, p.position.y, yaw_of(p.orientation)
        x = x0 + dx * math.cos(a0) - dy * math.sin(a0)
        y = y0 + dx * math.sin(a0) + dy * math.cos(a0)
        a = a0 + da
        self.pose = (time.monotonic(), x - REAR * math.cos(a), y - REAR * math.sin(a), a)

    def command(self, speed):
        """Twist for this step (zero when finished or without a fresh pose)."""
        cmd = Twist()
        if self.done or self.pose is None or time.monotonic() - self.pose[0] > 1.0:
            return cmd
        _, x, y, a = self.pose
        n = len(self.pts)
        if self.idx is None:
            self.idx = min(range(n), key=lambda i: math.dist(self.pts[i], (x, y)))
        # Advance along the route only (a window ahead), never jump across it.
        best, bd = self.idx, math.dist(self.pts[self.idx], (x, y))
        for k in range(1, 40):
            j = (self.idx + k) % n
            d = math.dist(self.pts[j], (x, y))
            if d < bd:
                best, bd = j, d
        step = (self.s_list[best] - self.s_list[self.idx]) % self.total
        self.travel += step
        self.idx = best
        if self.travel >= self.laps * self.total:
            self.done = True
            return Twist()
        tx, ty, _ = routes.pose_at(self.pts, self.s_list, self.total, self.s_list[self.idx] + LOOKAHEAD)
        alpha = math.atan2(ty - y, tx - x) - a
        alpha = math.atan2(math.sin(alpha), math.cos(alpha))
        ld = max(0.1, math.hypot(tx - x, ty - y))
        w = 2 * speed * math.sin(alpha) / ld
        wmax = speed / MIN_RADIUS
        cmd.linear.x = speed
        cmd.angular.z = max(-wmax, min(wmax, w))
        return cmd


class RouteDriver(Node):
    def __init__(self):
        super().__init__('route_driver')
        spec_path = self.declare_parameter('spec', '').value
        self.out = os.path.expanduser(self.declare_parameter('out', '.').value)
        self.timeout = float(self.declare_parameter('timeout', 900.0).value)
        spec = routes.load_spec(spec_path)
        self.speed = float(spec['speed'])
        pts = routes.route_points(spec['route'])
        s_list, total = routes.cumulative(pts)
        os.makedirs(self.out, exist_ok=True)
        self.cars, self.pubs, self.logs = [], {}, {}
        for name, start in zip(spec['agents'], routes.start_poses(spec)):
            car = Car(name, start, pts, s_list, total)
            car.laps = float(spec['laps'])
            self.cars.append(car)
            self.pubs[name] = self.create_publisher(Twist, f'/{name}/cmd_vel', 10)
            self.create_subscription(Odometry, f'/{name}/ground_truth/odom', car.on_gt, 50)
            log = open(os.path.join(self.out, f'{name}_online.csv'), 'w')
            log.write('wall,stamp,x,y,yaw\n')
            self.logs[name] = log
            self.create_subscription(Odometry, f'/{name}/orb_slam3/odom',
                                     lambda m, n=name: self.on_est(n, m), 50)
        self.get_logger().info(
            f'{len(self.cars)} cars, route {total:.1f} m, {spec["laps"]} laps at {self.speed} m/s '
            f'(about {spec["laps"] * total / self.speed:.0f} s)')
        self.t0 = time.monotonic()
        self.timer = self.create_timer(0.05, self.tick)
        self.finished = False

    def on_est(self, name, m):
        p = m.pose.pose
        st = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        self.logs[name].write(f'{time.time():.4f},{st:.4f},{p.position.x:.5f},{p.position.y:.5f},'
                              f'{yaw_of(p.orientation):.5f}\n')

    def tick(self):
        for car in self.cars:
            self.pubs[car.name].publish(car.command(self.speed))
        if all(c.done for c in self.cars) or time.monotonic() - self.t0 > self.timeout:
            self.finished = True

    def stop(self):
        for _ in range(10):
            for name in self.pubs:
                self.pubs[name].publish(Twist())
            time.sleep(0.02)
        for log in self.logs.values():
            log.close()


def main():
    rclpy.init()
    node = RouteDriver()
    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.05)
    except KeyboardInterrupt:
        pass
    node.stop()
    for c in node.cars:
        node.get_logger().info(f'{c.name}: drove {c.travel:.1f} m along the route'
                               f'{"" if c.done else " (NOT finished)"}')
    rclpy.shutdown()


if __name__ == '__main__':
    main()
