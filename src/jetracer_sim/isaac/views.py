"""Fixed viewing cameras for the Isaac window: one looking straight down from
under the ceiling, one in each upper corner looking at the room centre.
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


def add_views(stage):
    """Returns the camera paths, top view first."""
    UsdGeom.Xform.Define(stage, ROOT)
    # Top: orthographic (a plan view; a normal lens under a 2 m ceiling
    # cannot see the whole room), x to the right and y up, like RViz.
    paths = [_camera(stage, 'Top', (0, 0, HEIGHT - 0.05), (0, 0, 0), (0, 1, 0), ortho_width=2 * HALF + 0.4)]
    c = HALF - 0.2
    for name, sx, sy in [('Corner_NE', 1, 1), ('Corner_NW', -1, 1), ('Corner_SW', -1, -1), ('Corner_SE', 1, -1)]:
        paths.append(_camera(stage, name, (sx * c, sy * c, HEIGHT - 0.15), (0, 0, 0.3), (0, 0, 1), 10.0))
    return paths
