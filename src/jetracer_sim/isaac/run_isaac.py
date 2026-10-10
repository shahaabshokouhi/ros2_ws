#!/usr/bin/env python3
"""Isaac Sim stand-in for one JetRacer: the robot's topics, from a simulated car.

Run with Isaac Sim's Python (../run_isaac_sim.sh does that):

    python run_isaac.py [--usd FILE] [--agent sim] [--headless] [--no-realtime]

Opens the JetRacer scene (Ackermann car with a RealSense D455 in a room) and
wires it to ROS 2 like the real robot:

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

Images carry the SYSTEM clock and the simulation is paced to real time, so
the SLAM node, Nav2 and RViz run exactly as on the robot (no use_sim_time).
"""
import argparse
import time

ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
ap.add_argument('--usd', default='/home/shahab/Jetracer/jetracer3.usd')
ap.add_argument('--agent', default='sim')
ap.add_argument('--headless', action='store_true')
ap.add_argument('--no-realtime', action='store_true', help='run as fast as possible')
ap.add_argument('--width', type=int, default=640)
ap.add_argument('--height', type=int, default=480)
ap.add_argument('--light', type=float, default=1000.0,
                help='intensity of an even dome light added to the room (0: none). The scene '
                     'lights only the area around the start: the floor beyond ~1 m is black, '
                     'and visual SLAM loses tracking there')
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
from pxr import Sdf  # noqa: E402
from isaacsim.core.api import SimulationContext  # noqa: E402

CAR = '/World/Jetracer'
CAMERA = CAR + '/base_link/Realsense/RSD455/Camera_OmniVision_OV9782_Color'
A = '/' + args.agent

ctx = omni.usd.get_context()
ctx.open_stage(args.usd)
while ctx.get_stage_loading_status()[2] > 0:
    app.update()
for _ in range(10):
    app.update()


def set_input(node_path, name, value):
    og.Controller.attribute(f'{node_path}.inputs:{name}').set(value)


# The camera sits inside the D455 housing model, which then fills the whole
# image (black, depth 1.2 cm). Move the near clipping plane past the housing,
# far below the real camera's minimum range (~0.2 m).
from pxr import Gf  # noqa: E402
ctx.get_stage().GetPrimAtPath(CAMERA).GetAttribute('clippingRange').Set(Gf.Vec2f(0.03, 100.0))

# The D455 model is its own rigid body nested inside the car's (PhysX:
# "missing xformstack reset ... unpredicted results"), so the camera can
# move relative to the chassis. A real camera is bolted on: drop its
# separate physics so it simply rides along with base_link.
from pxr import UsdPhysics  # noqa: E402
for prim in ctx.get_stage().GetPrimAtPath(CAR + '/base_link/Realsense').GetAllChildren() + \
        [ctx.get_stage().GetPrimAtPath(CAR + '/base_link/Realsense')]:
    for p in [prim] + list(prim.GetAllChildren()):
        for api in (UsdPhysics.RigidBodyAPI, UsdPhysics.MassAPI):
            if p.HasAPI(api):
                p.RemoveAPI(api)
                print(f'[jetracer_sim] removed {api.__name__} from {p.GetPath()}', flush=True)

if args.light > 0:
    from pxr import UsdLux  # noqa: E402
    dome = UsdLux.DomeLight.Define(ctx.get_stage(), '/World/jetracer_sim_dome_light')
    dome.CreateIntensityAttr(args.light)
    print(f'[jetracer_sim] added a dome light ({args.light:g})', flush=True)

# Drive: the scene's Ackermann graph, on the robot's topic.
drive = CAR + '/ROS_Ackermann_Drive'
set_input(drive + '/ros2_subscribe_ackermanndrive', 'topicName', A + '/drive')
set_input(drive + '/ackermann_steering', 'maxWheelRotation', args.max_steer)

# Ground truth: the scene's odometry graph, renamed so it stays off /tf.
odom = CAR + '/ROS_Odom'
set_input(odom + '/ros2_publish_odometry', 'topicName', A + '/ground_truth/odom')
set_input(odom + '/ros2_publish_odometry', 'odomFrameId', 'gt_odom')
set_input(odom + '/ros2_publish_odometry', 'chassisFrameId', 'gt_base_link')
set_input(odom + '/ros2_publish_raw_transform_tree', 'topicName', A + '/ground_truth/tf')
set_input(odom + '/ros2_publish_raw_transform_tree', 'parentFrameId', 'gt_odom')
set_input(odom + '/ros2_publish_raw_transform_tree', 'childFrameId', 'gt_base_link')

# Camera: RGB and depth from the SAME camera (aligned, like the robot's
# aligned_depth_to_color), every second rendered frame = 30 Hz.
K = og.Controller.Keys
rs = A + '/camera/realsense2_camera/color/'
og.Controller.edit(
    {'graph_path': CAR + '/ROS_Camera', 'evaluator_name': 'execution'},
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
            ('rp.inputs:cameraPrim', [Sdf.Path(CAMERA)]),
            ('rp.inputs:width', args.width),
            ('rp.inputs:height', args.height),
            ('rgb.inputs:type', 'rgb'),
            ('rgb.inputs:topicName', rs + 'image_raw'),
            ('rgb.inputs:frameId', 'camera_color_optical_frame'),
            ('rgb.inputs:frameSkipCount', 1),
            ('rgb.inputs:useSystemTime', True),
            ('depth.inputs:type', 'depth'),
            ('depth.inputs:topicName', A + '/camera/realsense2_camera/depth/image_rect_raw'),
            ('depth.inputs:frameId', 'camera_color_optical_frame'),
            ('depth.inputs:frameSkipCount', 1),
            ('depth.inputs:useSystemTime', True),
            ('info.inputs:topicName', rs + 'camera_info'),
            ('info.inputs:frameId', 'camera_color_optical_frame'),
            ('info.inputs:frameSkipCount', 1),
            ('info.inputs:useSystemTime', True),
        ],
    },
)

sim = SimulationContext(physics_dt=1.0 / 60.0, rendering_dt=1.0 / 60.0, stage_units_in_meters=1.0)
sim.play()
print(f'[jetracer_sim] running {args.usd} as agent "{args.agent}"'
      f'{" in real time" if not args.no_realtime else ""}; Ctrl-C or close the window to stop',
      flush=True)

# Real-time pacing: one step is 1/60 s of simulated time.
t0, steps, last_report = time.time(), 0, time.time()
try:
    while app.is_running():
        sim.step(render=True)
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
sim.stop()
app.close()
