"""Fixed viewing cameras for the Isaac window: one looking straight down from
under the ceiling, one in each upper corner looking at the room centre (the
room, or with --world a larger world: its half sizes and ceiling height).
Pick them in the viewport's camera menu (top left of the viewport); they are
not published to ROS and cost nothing until selected."""
from pxr import Gf, UsdGeom

ROOT = '/World/SimViews'
HALF = 2.975            # inner faces of the walls (m)
HEIGHT = 2.0            # ceiling (m)
APERTURE = 20.955       # horizontal aperture (mm), USD default


def _camera(stage, name, eye, target, up, focal=None, ortho_width=None):
    cam = UsdGeom.Camera.Define(stage, f'{ROOT}/{name}')
    if ortho_width:
        # Apertures are in tenths of a scene unit (here 10 cm).
        cam.CreateProjectionAttr(UsdGeom.Tokens.orthographic)
        cam.CreateHorizontalApertureAttr(ortho_width * 10)
        cam.CreateVerticalApertureAttr(ortho_width * 10)
    else:
        cam.CreateFocalLengthAttr(focal)
        cam.CreateHorizontalApertureAttr(APERTURE)
    cam.CreateClippingRangeAttr(Gf.Vec2f(0.05, 100.0))
    m = Gf.Matrix4d().SetLookAt(Gf.Vec3d(*eye), Gf.Vec3d(*target), Gf.Vec3d(*up)).GetInverse()
    UsdGeom.Xformable(cam).AddTransformOp().Set(m)
    return f'{ROOT}/{name}'


def add_views(stage, half=(HALF, HALF), height=HEIGHT, centre=(0.0, 0.0)):
    """Returns the camera paths, top view first. half: the walls' inner half
    sizes in x and y (m); height: the ceiling (m); centre: the floor's centre."""
    UsdGeom.Xform.Define(stage, ROOT)
    hx, hy = half
    cx, cy = centre
    # Top: orthographic (a plan view; a normal lens under the ceiling
    # cannot see the whole room), x to the right and y up, like RViz.
    paths = [_camera(stage, 'Top', (cx, cy, height - 0.05), (cx, cy, 0), (0, 1, 0), ortho_width=2 * max(hx, hy) + 0.4)]
    for name, sx, sy in [('Corner_NE', 1, 1), ('Corner_NW', -1, 1), ('Corner_SW', -1, -1), ('Corner_SE', 1, -1)]:
        paths.append(_camera(stage, name, (cx + sx * (hx - 0.2), cy + sy * (hy - 0.2), height - 0.15), (cx, cy, 0.3),
                             (0, 0, 1), 10.0))
    return paths
