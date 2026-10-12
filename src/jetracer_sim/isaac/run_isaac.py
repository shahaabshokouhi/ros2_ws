#!/usr/bin/env python3
"""Isaac Sim stand-in for one or more JetRacers: the robots' topics, from simulated cars.

Run with Isaac Sim's Python (../run_isaac_sim.sh does that):

    python run_isaac.py [--usd FILE] [--agent sim] [--headless] [--view Top] [--no-realtime] [--plain-room]
                        [--agents N] [--poses "x,y,yaw;..."] [--gt-log DIR]

Opens the JetRacer scene (Ackermann car with a RealSense D455 in a room),
furnishes the room so visual SLAM can track (room_dressing.py: textured walls
and floor, shelves and boxes, a ceiling with panel lights; nothing is saved
to the .usd) and wires it to ROS 2 like the real robot:

  /<agent>/camera/realsense2_camera/color/image_raw  rgb8, 640x480, 30 Hz
  /<agent>/camera/realsense2_camera/depth/image_rect_raw
                                                     32FC1 metres from the same camera
                                                     (the SLAM node also reads the real
                                                     camera's 16UC1 millimetres)
  /<agent>/camera/realsense2_camera/color/camera_info
  /<agent>/drive            ackermann_msgs/AckermannDriveStamped in (jetracer_sim
                            cmd_vel_to_ackermann sends it from /<agent>/cmd_vel)
  /<agent>/ground_truth/odom, /<agent>/ground_truth/tf
                            the car's true pose (frames gt_odom -> gt_base_link),
                            kept off /tf so it cannot mix with the SLAM frames

With --agents N > 1 the car is copied N times (agents <agent>_1 .. <agent>_N),
each with its own camera and topics, for multi-robot tests (see
run_ma_sim_test.sh). --poses places the cars ("x,y,yaw_deg" per car, world
frame). --gt-log DIR writes DIR/<agent>_gt.csv: every simulation step, the
wall time and each car's true base_link pose and camera pose (optical frame:
x right, y down, z forward, as ORB-SLAM3 uses), in Isaac's world frame.

Images carry the SYSTEM clock and the simulation is paced to real time, so
the SLAM node, Nav2 and RViz run exactly as on the robot (no use_sim_time).
"""
import argparse
import math
import os
import sys
import time

ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
ap.add_argument('--usd', default='/home/shahab/Jetracer/jetracer3.usd')
ap.add_argument('--agent', default='sim')
ap.add_argument('--agents', type=int, default=1,
                help='number of cars; above 1 they are named <agent>_1 .. <agent>_N')
ap.add_argument('--poses', default='',
                help='start poses "x,y,yaw_deg;..." (world frame), one per car')
ap.add_argument('--gt-log', default='', help='directory for <agent>_gt.csv ground-truth logs')
ap.add_argument('--headless', action='store_true')
ap.add_argument('--view', default='Corner_SW', choices=['Top', 'Corner_NE', 'Corner_NW', 'Corner_SW', 'Corner_SE'],
                help='viewing camera the window starts with (others: viewport camera menu)')
ap.add_argument('--no-realtime', action='store_true', help='run as fast as possible')
ap.add_argument('--width', type=int, default=640)
ap.add_argument('--height', type=int, default=480)
ap.add_argument('--plain-room', action='store_true',
                help='leave the room as the .usd has it (bare walls, sun light) instead of '
                     'furnishing it for visual SLAM (room_dressing.py)')
ap.add_argument('--light', type=float, default=None,
                help='brightness of each of the 9 ceiling panels (default 12000); with '
                     '--plain-room: of an even dome light (default 1000, 0: none)')
ap.add_argument('--max-steer', type=float, default=0.59,
                help='steering limit (rad); the real car has 0.6, the joints allow 0.59')
args = ap.parse_args()

from isaacsim import SimulationApp  # noqa: E402  (must come before any omni import)

app = SimulationApp({'headless': args.headless, 'width': 1280, 'height': 720})

from isaacsim.core.utils.extensions import enable_extension  # noqa: E402
enable_extension('isaacsim.ros2.bridge')
app.update()

import omni.graph.core as og  # noqa: E402
import omni.usd  # noqa: E402
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics  # noqa: E402
from isaacsim.core.api import SimulationContext  # noqa: E402

CAR = '/World/Jetracer'
CAMERA_REL = '/base_link/Realsense/RSD455/Camera_OmniVision_OV9782_Color'

ctx = omni.usd.get_context()
ctx.open_stage(args.usd)
while ctx.get_stage_loading_status()[2] > 0:
    app.update()
for _ in range(10):
    app.update()
stage = ctx.get_stage()


def set_input(node_path, name, value):
    og.Controller.attribute(f'{node_path}.inputs:{name}').set(value)


# The cars: the scene's one, plus copies for a multi-robot test. Sdf.CopySpec
# remaps relationship targets inside the copied tree, so each copy's drive and
# odometry graphs act on that copy.
if args.agents > 1:
    cars = [(CAR if k == 0 else f'{CAR}_{k + 1}', f'{args.agent}_{k + 1}') for k in range(args.agents)]
    for path, _ in cars[1:]:
        if not Sdf.CopySpec(stage.GetRootLayer(), CAR, stage.GetRootLayer(), path):
            sys.exit(f'[jetracer_sim] could not copy the car to {path}')
else:
    cars = [(CAR, args.agent)]
if args.poses:
    poses = [tuple(float(v) for v in p.split(',')) for p in args.poses.split(';') if p.strip()]
    if len(poses) != len(cars):
        sys.exit(f'[jetracer_sim] --poses has {len(poses)} poses for {len(cars)} cars')
    for (path, _), (x, y, yaw) in zip(cars, poses):
        for op in UsdGeom.Xformable(stage.GetPrimAtPath(path)).GetOrderedXformOps():
            if op.GetOpName() == 'xformOp:translate':
                t = op.Get()
                op.Set(type(t)(x, y, t[2]))
            elif op.GetOpName() == 'xformOp:orient':
                q = op.Get()
                op.Set(type(q)(math.cos(math.radians(yaw) / 2), 0, 0, math.sin(math.radians(yaw) / 2)))
for _ in range(5):
    app.update()


def setup_car(car, agent):
    """Wire one car to ROS 2 under /<agent>/ (the robot's topic names)."""
    A = '/' + agent
    camera = car + CAMERA_REL

    # The camera sits inside the D455 housing model, which then fills the whole
    # image (black, depth 1.2 cm). Move the near clipping plane past the housing,
    # far below the real camera's minimum range (~0.2 m).
    stage.GetPrimAtPath(camera).GetAttribute('clippingRange').Set(Gf.Vec2f(0.03, 100.0))

    # The D455 model is its own rigid body nested inside the car's (PhysX:
    # "missing xformstack reset ... unpredicted results"), so the camera can
    # move relative to the chassis. A real camera is bolted on: drop its
    # separate physics so it simply rides along with base_link.
    rs = stage.GetPrimAtPath(car + '/base_link/Realsense')
    for prim in rs.GetAllChildren() + [rs]:
        for p in [prim] + list(prim.GetAllChildren()):
            for api in (UsdPhysics.RigidBodyAPI, UsdPhysics.MassAPI):
                if p.HasAPI(api):
                    p.RemoveAPI(api)
                    print(f'[jetracer_sim] removed {api.__name__} from {p.GetPath()}', flush=True)

    # Drive: the scene's Ackermann graph, on the robot's topic.
    drive = car + '/ROS_Ackermann_Drive'
    set_input(drive + '/ros2_subscribe_ackermanndrive', 'topicName', A + '/drive')
    set_input(drive + '/ackermann_steering', 'maxWheelRotation', args.max_steer)

    # Ground truth: the scene's odometry graph, renamed so it stays off /tf.
    odom = car + '/ROS_Odom'
    set_input(odom + '/ros2_publish_odometry', 'topicName', A + '/ground_truth/odom')
    set_input(odom + '/ros2_publish_odometry', 'odomFrameId', 'gt_odom')
    set_input(odom + '/ros2_publish_odometry', 'chassisFrameId', 'gt_base_link')
    set_input(odom + '/ros2_publish_raw_transform_tree', 'topicName', A + '/ground_truth/tf')
    set_input(odom + '/ros2_publish_raw_transform_tree', 'parentFrameId', 'gt_odom')
    set_input(odom + '/ros2_publish_raw_transform_tree', 'childFrameId', 'gt_base_link')

    # Camera: RGB and depth from the SAME camera (aligned, like the robot's
    # aligned_depth_to_color), every second rendered frame = 30 Hz.
    K = og.Controller.Keys
    color = A + '/camera/realsense2_camera/color/'
    # A scene saved from a previous run (Isaac's "save changes?" on close)
    # already holds this graph, possibly with old settings: rebuild it.
    if stage.GetPrimAtPath(car + '/ROS_Camera').IsValid():
        stage.RemovePrim(car + '/ROS_Camera')
        app.update()
        print('[jetracer_sim] replaced the ROS_Camera graph saved in the scene', flush=True)
    og.Controller.edit(
        {'graph_path': car + '/ROS_Camera', 'evaluator_name': 'execution'},
        {
            K.CREATE_NODES: [
                ('tick', 'omni.graph.action.OnPlaybackTick'),
                ('ctx', 'isaacsim.ros2.bridge.ROS2Context'),
                ('rp', 'isaacsim.core.nodes.IsaacCreateRenderProduct'),
                ('rgb', 'isaacsim.ros2.bridge.ROS2CameraHelper'),
                ('depth', 'isaacsim.ros2.bridge.ROS2CameraHelper'),
                ('info', 'isaacsim.ros2.bridge.ROS2CameraInfoHelper'),
            ],
            K.CONNECT: [
                ('tick.outputs:tick', 'rp.inputs:execIn'),
                ('rp.outputs:execOut', 'rgb.inputs:execIn'),
                ('rp.outputs:execOut', 'depth.inputs:execIn'),
                ('rp.outputs:execOut', 'info.inputs:execIn'),
                ('rp.outputs:renderProductPath', 'rgb.inputs:renderProductPath'),
                ('rp.outputs:renderProductPath', 'depth.inputs:renderProductPath'),
                ('rp.outputs:renderProductPath', 'info.inputs:renderProductPath'),
                ('ctx.outputs:context', 'rgb.inputs:context'),
                ('ctx.outputs:context', 'depth.inputs:context'),
                ('ctx.outputs:context', 'info.inputs:context'),
            ],
            K.SET_VALUES: [
                ('rp.inputs:cameraPrim', [Sdf.Path(camera)]),
                ('rp.inputs:width', args.width),
                ('rp.inputs:height', args.height),
                ('rgb.inputs:type', 'rgb'),
                ('rgb.inputs:topicName', color + 'image_raw'),
                ('rgb.inputs:frameId', 'camera_color_optical_frame'),
                ('rgb.inputs:frameSkipCount', 1),
                ('rgb.inputs:useSystemTime', True),
                ('depth.inputs:type', 'depth'),
                ('depth.inputs:topicName', A + '/camera/realsense2_camera/depth/image_rect_raw'),
                ('depth.inputs:frameId', 'camera_color_optical_frame'),
                ('depth.inputs:frameSkipCount', 1),
                ('depth.inputs:useSystemTime', True),
                ('info.inputs:topicName', color + 'camera_info'),
                ('info.inputs:frameId', 'camera_color_optical_frame'),
                ('info.inputs:frameSkipCount', 1),
                ('info.inputs:useSystemTime', True),
            ],
        },
    )
    print(f'[jetracer_sim] car {car} -> agent {A}', flush=True)


for car, agent in cars:
    setup_car(car, agent)

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
if not args.plain_room:
    import room_dressing  # noqa: E402
    for line in room_dressing.dress(stage, args.light):
        print(f'[jetracer_sim] room: {line}', flush=True)
elif (1000.0 if args.light is None else args.light) > 0:
    # The bare room is lit only around the start: the floor beyond ~1 m is
    # black, and visual SLAM loses tracking there.
    from pxr import UsdLux  # noqa: E402
    dome = UsdLux.DomeLight.Define(stage, '/World/jetracer_sim_dome_light')
    dome.CreateIntensityAttr(1000.0 if args.light is None else args.light)
    print(f'[jetracer_sim] added a dome light ({dome.GetIntensityAttr().Get():g})', flush=True)

# Viewing cameras for the window (top + 4 upper corners); the scene's own
# default view looks at the room from outside, i.e. at the ceiling.
import views  # noqa: E402
view_paths = views.add_views(stage)
if not args.headless:
    from omni.kit.viewport.utility import get_active_viewport  # noqa: E402
    get_active_viewport().camera_path = next(p for p in view_paths if p.endswith('/' + args.view))
print('[jetracer_sim] views (viewport camera menu): ' + ', '.join(p.split('/')[-1] for p in view_paths), flush=True)


# Ground-truth log: each car's base_link and camera (optical frame) in the world.
def _unit(v):
    n = math.sqrt(v[0] ** 2 + v[1] ** 2 + v[2] ** 2) or 1.0
    return (v[0] / n, v[1] / n, v[2] / n)


def _cross(a, b):
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


def _quat(c0, c1, c2):
    """Quaternion (x, y, z, w) of the rotation with columns c0, c1, c2."""
    m00, m11, m22 = c0[0], c1[1], c2[2]
    tr = m00 + m11 + m22
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2
        return ((c1[2] - c2[1]) / s, (c2[0] - c0[2]) / s, (c0[1] - c1[0]) / s, 0.25 * s)
    if m00 > m11 and m00 > m22:
        s = math.sqrt(1.0 + m00 - m11 - m22) * 2
        return (0.25 * s, (c1[0] + c0[1]) / s, (c2[0] + c0[2]) / s, (c1[2] - c2[1]) / s)
    if m11 > m22:
        s = math.sqrt(1.0 + m11 - m00 - m22) * 2
        return ((c1[0] + c0[1]) / s, 0.25 * s, (c2[1] + c1[2]) / s, (c2[0] - c0[2]) / s)
    s = math.sqrt(1.0 + m22 - m00 - m11) * 2
    return ((c2[0] + c0[2]) / s, (c2[1] + c1[2]) / s, 0.25 * s, (c0[1] - c1[0]) / s)


def _frame(prim, fwd, up):
    """World position and orthonormal (x, y, z) axes of a prim, from its
    forward and up axes (the car's root is scaled unevenly)."""
    m = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    p = m.ExtractTranslation()
    f = _unit(m.TransformDir(Gf.Vec3d(*fwd)))
    u = _unit(m.TransformDir(Gf.Vec3d(*up)))
    r = _unit(_cross(f, u))
    u = _cross(r, f)
    return (p[0], p[1], p[2]), f, u, r


gt_logs = []
if args.gt_log:
    os.makedirs(args.gt_log, exist_ok=True)
    for car, agent in cars:
        f = open(os.path.join(args.gt_log, f'{agent}_gt.csv'), 'w')
        f.write('wall,base_x,base_y,base_z,base_yaw,cam_x,cam_y,cam_z,cam_qx,cam_qy,cam_qz,cam_qw\n')
        gt_logs.append((f, stage.GetPrimAtPath(car + '/base_link'), stage.GetPrimAtPath(car + CAMERA_REL)))


def log_ground_truth():
    now = time.time()
    for f, base, cam in gt_logs:
        bp, bf, _, _ = _frame(base, (1, 0, 0), (0, 0, 1))
        # USD cameras look along -Z with +Y up; the optical frame is x right,
        # y down, z forward.
        cp, cf, cu, cr = _frame(cam, (0, 0, -1), (0, 1, 0))
        q = _quat(cr, (-cu[0], -cu[1], -cu[2]), cf)
        f.write(f'{now:.4f},{bp[0]:.5f},{bp[1]:.5f},{bp[2]:.5f},{math.atan2(bf[1], bf[0]):.5f},'
                f'{cp[0]:.5f},{cp[1]:.5f},{cp[2]:.5f},{q[0]:.6f},{q[1]:.6f},{q[2]:.6f},{q[3]:.6f}\n')


sim = SimulationContext(physics_dt=1.0 / 60.0, rendering_dt=1.0 / 60.0, stage_units_in_meters=1.0)
sim.play()
names = ', '.join(a for _, a in cars)
print(f'[jetracer_sim] running {args.usd} as {"agents" if len(cars) > 1 else "agent"} "{names}"'
      f'{" in real time" if not args.no_realtime else ""}; Ctrl-C or close the window to stop',
      flush=True)

# Real-time pacing: one step is 1/60 s of simulated time.
t0, steps, last_report = time.time(), 0, time.time()
try:
    while app.is_running():
        sim.step(render=True)
        if gt_logs:
            log_ground_truth()
        steps += 1
        now = time.time()
        if not args.no_realtime:
            ahead = steps / 60.0 - (now - t0)
            if ahead > 0:
                time.sleep(ahead)
        if now - last_report > 30.0:
            print(f'[jetracer_sim] {steps / 60.0:.0f} s simulated in {now - t0:.0f} s '
                  f'(real-time factor {steps / 60.0 / max(1e-6, now - t0):.2f})', flush=True)
            last_report = now
except KeyboardInterrupt:
    pass
for f, _, _ in gt_logs:
    f.close()
sim.stop()
app.close()
