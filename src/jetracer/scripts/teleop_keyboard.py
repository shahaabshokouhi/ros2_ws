#!/usr/bin/env python3
"""Drive one JetRacer from this computer's keyboard (installed as `teleop`).

    ros2 run jetracer teleop --ros-args -p agent_name:=agent_2

Publishes geometry_msgs/Twist on /<agent_name>/cmd_vel at 20 Hz, which the
robot's base driver (`jetracer`) executes. Hold a key to move: the command
drops to zero hold_timeout seconds after the last key press, and the base
driver stops the motors on its own after 1 s without any command (for
example when Wi-Fi drops).

  w / s  or  arrow up / down     forward / backward
  a / d  or  arrow left / right  turn left / right on the spot
  q / e                          forward while turning left / right
  z / c                          backward while turning left / right
  space or x                     stop
  + / -                          faster / slower (capped by max_linear, max_angular)
  Ctrl-C                         quit (sends stop)

Parameters: agent_name, speed (m/s, 0.15), turn (rad/s, 0.8), max_linear (0.3),
max_angular (1.5), hold_timeout (s, 0.7: longer than the keyboard's
auto-repeat delay, so holding a key does not stutter), latch (false: when
true, a key keeps its command until space).
"""
import os
import select
import sys
import termios
import time
import tty

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions

# key -> (linear sign, angular sign)
KEYS = {'w': (1, 0), 's': (-1, 0), 'a': (0, 1), 'd': (0, -1),
        'q': (1, 1), 'e': (1, -1), 'z': (-1, 1), 'c': (-1, -1)}
ARROWS = {'A': 'w', 'B': 's', 'D': 'a', 'C': 'd'}   # ESC [ A..D


class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__('teleop_keyboard')
        self.agent = self.declare_parameter('agent_name', 'agent_0').value
        self.speed = float(self.declare_parameter('speed', 0.15).value)
        self.turn = float(self.declare_parameter('turn', 0.8).value)
        self.max_lin = float(self.declare_parameter('max_linear', 0.3).value)
        self.max_ang = float(self.declare_parameter('max_angular', 1.5).value)
        self.hold = float(self.declare_parameter('hold_timeout', 0.7).value)
        self.latch = bool(self.declare_parameter('latch', False).value)
        self.speed = min(self.speed, self.max_lin)
        self.turn = min(self.turn, self.max_ang)
        self.pub = self.create_publisher(Twist, f'/{self.agent}/cmd_vel', 10)
        self.dir = (0, 0)
        self.last_key = 0.0

    def command(self):
        t = Twist()
        if self.dir != (0, 0) and (self.latch or time.monotonic() - self.last_key < self.hold):
            t.linear.x = self.dir[0] * self.speed
            t.angular.z = self.dir[1] * self.turn
        return t

    def key(self, k):
        if k in KEYS:
            self.dir = KEYS[k]
            self.last_key = time.monotonic()
        elif k in (' ', 'x'):
            self.dir = (0, 0)
        elif k in ('+', '='):
            self.scale(1.25)
        elif k in ('-', '_'):
            self.scale(0.8)

    def scale(self, f):
        self.speed = min(self.max_lin, max(0.02, self.speed * f))
        self.turn = min(self.max_ang, max(0.1, self.turn * f))

    def stop(self):
        for _ in range(5):
            self.pub.publish(Twist())
            time.sleep(0.02)


def read_keys(fd, timeout):
    """Keys typed since the last call (arrow keys mapped to w/a/s/d); [] after
    `timeout` s. Reads the raw descriptor: mixing select() with Python's
    buffered stdin leaves the tail of an arrow sequence in the buffer, where
    it is later read as letters (ESC [ C became the key 'c')."""
    if not select.select([fd], [], [], timeout)[0]:
        return []
    data = os.read(fd, 64).decode(errors='ignore')
    keys, i = [], 0
    while i < len(data):
        c = data[i]
        if c == '\x1b':                               # arrow: ESC [ A..D
            if data[i + 1:i + 2] == '[' and i + 2 < len(data):
                k = ARROWS.get(data[i + 2])
                if k:
                    keys.append(k)
                i += 3
            else:
                i += 1                                 # lone or unknown escape: ignore
            continue
        keys.append(c if c in '+-=_' else c.lower())
        i += 1
    return keys


def main():
    if not sys.stdin.isatty():
        sys.exit('teleop needs a terminal: run it with `ros2 run jetracer teleop` (not from a launch file)')
    # Ctrl-C is handled here (KeyboardInterrupt), not by rclpy, so the node is
    # still alive to send the final stop.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = KeyboardTeleop()
    print(__doc__.split('\n\n')[3])
    print(f'Driving /{node.agent}/cmd_vel. Hold a key to move.\n')
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        period, next_pub = 0.05, time.monotonic()
        while rclpy.ok():
            for k in read_keys(fd, max(0.0, next_pub - time.monotonic())):
                node.key(k)
            if time.monotonic() >= next_pub:
                cmd = node.command()
                node.pub.publish(cmd)
                next_pub += period
                sys.stdout.write(f'\r  linear {cmd.linear.x:+.2f} m/s  angular {cmd.angular.z:+.2f} rad/s'
                                 f'   (speed {node.speed:.2f} m/s, turn {node.turn:.2f} rad/s)   ')
                sys.stdout.flush()
            rclpy.spin_once(node, timeout_sec=0.0)
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
        node.stop()
        print('\nstopped')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
