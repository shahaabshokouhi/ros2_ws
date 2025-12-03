#!/usr/bin/env python3
import math
from typing import List, Tuple

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

# TODO: change this import to the actual package that defines Position.msg
# Example possibilities:
#   from vicon_receiver.msg import Position
#   from my_vicon_msgs.msg import Position
from geometry_msgs.msg import PoseStamped

def normalize_angle(angle: float) -> float:
    """Wrap angle to [-pi, pi]."""
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


class PID:
    def __init__(self, kp: float, ki: float, kd: float, output_limit: float = None):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_limit = output_limit

        self.integral = 0.0
        self.prev_error = 0.0
        self.initialized = False

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0
        self.initialized = False

    def update(self, error: float, dt: float) -> float:
        if dt <= 0.0:
            # No time passed, just P term
            return self.kp * error

        if not self.initialized:
            self.prev_error = error
            self.initialized = True

        # PID terms
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt

        output = self.kp * error + self.ki * self.integral + self.kd * derivative

        self.prev_error = error

        # Limit output if requested
        if self.output_limit is not None:
            if output > self.output_limit:
                output = self.output_limit
            elif output < -self.output_limit:
                output = -self.output_limit

        return output

 
class ViconPIDWaypointFollower(Node):
    def __init__(self):
        super().__init__('pid_controller')

        # Parameters
        self.declare_parameter('agent_name', 'agent_0')
        self.declare_parameter('control_rate', 1.0)          # Hz
        self.declare_parameter('pos_tolerance', 0.2)         # meters
        self.declare_parameter('max_linear_speed', 0.3)       # m/s
        self.declare_parameter('max_angular_speed', 0.3)      # rad/s

        # Simple gains (tune these!)
        self.declare_parameter('kp_dist', 20.0)
        self.declare_parameter('kp_yaw', 0.5)
        self.declare_parameter('kd_yaw', 0.0)

        agent_name = self.get_parameter('agent_name').get_parameter_value().string_value

        vicon_topic = f'/vicon/{agent_name}/{agent_name}'
        self.get_logger().info(f'Subscribing to Vicon topic: {vicon_topic}')

        self.subscription = self.create_subscription(
            PoseStamped,
            vicon_topic,
            self.vicon_callback,
            10
        )

        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        control_rate = self.get_parameter('control_rate').get_parameter_value().double_value
        self.dt = 1.0 / control_rate

        # Waypoints: list of (x, y, z). You can change this or load from a file/parameter.
        # Make sure they are in the same coordinate frame as your Vicon "Global" translation.
        self.waypoints: List[Tuple[float, float, float]] = [
            (2.05, 2.05, 0.0),
            (-2.05, 2.05, 0.0),
            (-2.05, 0.5, 0.0),
            (2.0, 0.5, 0.0),

        ]
        self.current_wp_index = 0

        # State from Vicon
        self.have_pose = False
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.yaw = 0.0

        # PID controllers
        kp_dist = self.get_parameter('kp_dist').get_parameter_value().double_value
        kp_yaw = self.get_parameter('kp_yaw').get_parameter_value().double_value
        kd_yaw = self.get_parameter('kd_yaw').get_parameter_value().double_value

        max_lin = self.get_parameter('max_linear_speed').get_parameter_value().double_value
        max_ang = self.get_parameter('max_angular_speed').get_parameter_value().double_value

        # Distance -> linear velocity (only P; I and D usually not needed here)
        self.pid_dist = PID(kp=kp_dist, ki=0.0, kd=0.0, output_limit=max_lin)

        # Yaw error -> angular velocity
        self.pid_yaw = PID(kp=kp_yaw, ki=0.0, kd=kd_yaw, output_limit=max_ang)

        self.last_time = self.get_clock().now()

        self.timer = self.create_timer(self.dt, self.control_loop)

    # ---------- Callbacks ----------

    def vicon_callback(self, msg: PoseStamped):
        """
        Position.msg:
        float32 x_trans 
        float32 y_trans
        float32 z_trans
        float32 x_rot
        float32 y_rot
        float32 z_rot
        float32 w
        string segment_name
        string subject_name
        int32 frame_number
        string translation_type
        """

        self.x = msg.pose.position.x
        self.y = msg.pose.position.y
        self.z = msg.pose.position.z

        qx = msg.pose.orientation.x
        qy = msg.pose.orientation.y
        qz = msg.pose.orientation.z
        qw = msg.pose.orientation.w

        # Compute yaw from quaternion
        # Formula for yaw (Z axis) from quaternion (x,y,z,w)
        # yaw = atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        self.yaw = math.atan2(siny_cosp, cosy_cosp)

        # self.get_logger().info(f'X: {self.x}, Y: {self.y}, Yaw: {self.yaw}')

        self.have_pose = True

    # ---------- Control Loop ----------

    def control_loop(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        self.get_logger().info("debug 0")

        if dt <= 0.0:
            dt = self.dt
        self.last_time = now

        if not self.have_pose:
            self.get_logger().info("debug 1")
            # No feedback yet
            return

        if self.current_wp_index >= len(self.waypoints):
            self.get_logger().info("debug 2")
            # Finished all waypoints: stop
            cmd = Twist()
            self.cmd_pub.publish(cmd)
            return

        target = self.waypoints[self.current_wp_index]
        tx, ty, tz = target

        # Position errors in world frame
        ex = tx - self.x
        ey = ty - self.y

        self.get_logger().info(f'error x: {ex}, error y: {ey}')
        # ez = tz - self.z  # If you want to control altitude for a drone

        dist_error = math.hypot(ex, ey)
        pos_tolerance = self.get_parameter('pos_tolerance').get_parameter_value().double_value
        self.get_logger().info(f'Dist error: {dist_error}')
        # Check if we reached current waypoint
        if dist_error < pos_tolerance:
            self.get_logger().info(
                f'Reached waypoint {self.current_wp_index}: ({tx:.2f}, {ty:.2f}, {tz:.2f})'
            )
            self.current_wp_index += 1

            # Reset PIDs when switching waypoints
            self.pid_dist.reset()
            self.pid_yaw.reset()

            # Stop if finished
            if self.current_wp_index >= len(self.waypoints):
                self.current_wp_index = 0
                # cmd = Twist()
                # self.cmd_pub.publish(cmd)
                # self.get_logger().info('All waypoints completed.')
                # return
            else:
                # Use new target for control this cycle
                target = self.waypoints[self.current_wp_index]
                tx, ty, tz = target
                ex = tx - self.x
                ey = ty - self.y
                dist_error = math.hypot(ex, ey)

        # Desired heading towards waypoint
        desired_yaw = math.atan2(ey, ex)
        yaw_error = normalize_angle(desired_yaw - self.yaw)

        # PID outputs
        v = self.pid_dist.update(dist_error, dt)  # linear velocity
        w = self.pid_yaw.update(yaw_error, dt)    # angular velocity
        # Optionally reduce forward speed when yaw error is large
        # e.g., simple scaling:
        # angle_factor = max(0.0, math.cos(yaw_error))
        # v *= angle_factor
        # self.get_logger().info(f'pid output: v: {v}, w: {w}')

        cmd = Twist()
        cmd.linear.x = v
        cmd.angular.z = w

        # If you have a drone or want z-control:
        # pid_z = ...
        # cmd.linear.z = pid_z.update(ez, dt)

        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = ViconPIDWaypointFollower()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Stop the robot on shutdown
        stop_cmd = Twist()
        node.cmd_pub.publish(stop_cmd)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
