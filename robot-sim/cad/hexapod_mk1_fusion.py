"""Hexapod Mk1 (rev B, bus servos) - parametric generator for Autodesk Fusion.

Run it inside Fusion (Utilities > Scripts and Add-Ins, or through the Fusion MCP
server).  It builds the design "Hexapod Mk1" from scratch in the active design:
one component per unique part, user parameters named after the fields in
robot-sim/src/robots/hexapod.config.js, revolute joints at every coxa, femur and
tibia axis, and the manufacturers' STEP models for the servos and boards.

Rev B is built round the Feetech STS3215 / Waveshare ST3215 serial-bus servo
(and the STS3250 in the same housing for the femurs).  Their official STEP
model was measured to place every bracket: the servo is clamped between two
5 mm printed faces that sit on its real case faces, and each moving bracket
bolts to the driven horn on one side and the idler horn on the other.

Vendor CAD: run cad/vendor/fetch_vendor_cad.py once to download the STEP files
from the manufacturers into cad/vendor/ (they are not committed: their
licences don't grant redistribution).  If a file is missing, build(use_vendor=
True) falls back to an envelope taken from the same datasheet numbers.

To change the robot: edit the user parameters (Modify > Change Parameters),
then run this script again.  It reads the parameters back, deletes the old
geometry and rebuilds every part.  Keep hexapod.config.js in step so the
simulator stays the digital twin.

Frames
------
Robot frame (Fusion world): +X = robot RIGHT, +Y = FORWARD, +Z = UP, origin on
the coxa joints' horn mid-plane (the "coxa-joint height"), centred on the body.
Simulator frame (three.js): +X = robot LEFT, +Y = UP, +Z = FORWARD.
So  X_fusion = -x_sim,  Y_fusion = z_sim,  Z_fusion = y_sim.

Servo frame L (every ST3215 / STS3250 envelope and STEP placement): origin on
the output axis half way between the two horn faces, +Z towards the driven
horn, case along +X (the axis is 10.11 mm from the case end), +Y across.

Leg-part frames: origin on the joint axis, +X along the leg, +Z up, +Y = left of
the leg (Z cross X) = the servo's +Z (driven horn side).

All dimensions below are millimetres; the helpers convert to Fusion's cm.
"""

import adsk.core
import adsk.fusion
import math
import os
import json

app = adsk.core.Application.get()
TB = adsk.fusion.TemporaryBRepManager.get()
UNION = adsk.fusion.BooleanTypes.UnionBooleanType
DIFF = adsk.fusion.BooleanTypes.DifferenceBooleanType
INTER = adsk.fusion.BooleanTypes.IntersectionBooleanType

HERE = os.path.dirname(os.path.abspath(__file__)) if '__file__' in globals() else None
VENDOR_DIR = os.path.join(HERE, 'vendor') if HERE else None
REBUILD = True


def cm(v):
    return v / 10.0


def P(x, y, z):
    return adsk.core.Point3D.create(cm(x), cm(y), cm(z))


def V(x, y, z):
    v = adsk.core.Vector3D.create(x, y, z)
    v.normalize()
    return v


# ----------------------------------------------------------------------------
# User parameters (names follow hexapod.config.js: body.length -> body_length)
# ----------------------------------------------------------------------------
PARAMS = [
    ('body_length', '260 mm', 'mm', 'config body.length - core hexagon length'),
    ('body_width', '140 mm', 'mm', 'config body.width - core hexagon width'),
    ('body_thickness', '5 mm', 'mm', 'config body.thickness - plate thickness'),
    ('body_plateGap', '30.4 mm', 'mm', 'config body.plateGap - set by the ST3215 case (13.825 + 16.575); do not change'),
    ('body_coxaInset', '11.5 mm', 'mm', 'plate edge distance inside each coxa axis (clears the horn and bracket pads)'),
    ('leg_coxa', '45 mm', 'mm', 'config leg.coxa - coxa axis to femur axis'),
    ('leg_femur', '75 mm', 'mm', 'config leg.femur - femur axis to tibia axis'),
    ('leg_tibia', '120 mm', 'mm', 'config leg.tibia - tibia axis to foot contact (incl. 3 mm TPU boot)'),
    ('leg_footReach', '100 mm', 'mm', 'config leg.footReach (load check: keep feet 80-115 mm from the coxa axis)'),
    ('standHeight', '90 mm', 'mm', 'config standHeight - coxa-joint height above the floor'),
    ('topMount_y', '13 mm', 'mm', 'attachment datum above the top plate'),
    ('frontMount_y', '4 mm', 'mm', 'config frontMount[1] - sonar centre height'),
    ('frontMount_z', '135 mm', 'mm', 'config frontMount[2] - sonar bracket front'),
    ('fit_clearance', '0.25 mm', 'mm', 'clearance per side for pockets and drop-in fits'),
    ('slide_clearance', '0.2 mm', 'mm', 'clearance per side for the dovetail and jigsaw seams'),
    ('insert_hole', '4.0 mm', 'mm', 'hole for M3 x 5 x 4 heat-set inserts'),
    ('clear_hole', '3.4 mm', 'mm', 'M3 clearance hole'),
    ('m2_clear', '2.4 mm', 'mm', 'M2 clearance hole (servo case screws)'),
    ('first_layer_chamfer', '0.5 mm', 'mm', 'elephant-foot chamfer on bed-side edges'),
    ('root_fillet', '3 mm', 'mm', 'fillet on load-path inside corners'),
]
SIM_MOUNTS = [(-60, 110), (-80, 0), (-60, -110), (60, 110), (80, 0), (60, -110)]
SIM_SPLAY = [35, 0, -35, 35, 0, -35]
for i, (x, z) in enumerate(SIM_MOUNTS):
    PARAMS.append(('mounts_%d_x' % i, '%g mm' % x, 'mm', 'config mounts[%d][0] (sim x, +x = robot left)' % i))
    PARAMS.append(('mounts_%d_z' % i, '%g mm' % z, 'mm', 'config mounts[%d][1] (sim z, forward)' % i))
    PARAMS.append(('legSplay_%d' % i, '%g deg' % SIM_SPLAY[i], 'deg', 'config legSplay[%d]' % i))


def ensure_params(design):
    up = design.userParameters
    for name, expr, units, comment in PARAMS:
        if up.itemByName(name) is None:
            up.add(name, adsk.core.ValueInput.createByString(expr), units, comment)


def pmm(design, name):
    return design.userParameters.itemByName(name).value * 10.0  # cm -> mm


def pdeg(design, name):
    return math.degrees(design.userParameters.itemByName(name).value)


# ----------------------------------------------------------------------------
# Temporary-BRep primitives (mm)
# ----------------------------------------------------------------------------
def box(x0, x1, y0, y1, z0, z1):
    c = P((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2)
    ob = adsk.core.OrientedBoundingBox3D.create(c, V(1, 0, 0), V(0, 1, 0), cm(abs(x1 - x0)), cm(abs(y1 - y0)), cm(abs(z1 - z0)))
    return TB.createBox(ob)


def obox(center, ldir, wdir, L, W, H):
    ob = adsk.core.OrientedBoundingBox3D.create(P(*center), V(*ldir), V(*wdir), cm(L), cm(W), cm(H))
    return TB.createBox(ob)


def cyl(p0, p1, r):
    return TB.createCylinderOrCone(P(*p0), cm(r), P(*p1), cm(r))


def sphere(c, r):
    return TB.createSphere(P(*c), cm(r))


def U(a, *bs):
    for b in bs:
        TB.booleanOperation(a, b, UNION)
    return a


def D(a, *bs):
    for b in bs:
        TB.booleanOperation(a, b, DIFF)
    return a


def I(a, b):
    TB.booleanOperation(a, b, INTER)
    return a


def _vec(a):
    n = math.sqrt(sum(c * c for c in a))
    return [c / n for c in a]


def _cross(a, b):
    return [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]


def _perp(a):
    t = [1, 0, 0] if abs(a[0]) < 0.9 else [0, 1, 0]
    return _vec(_cross(a, t))


def halfspace(p0, n, size=2000):
    """Big box occupying the side of the plane (p0, n) that n points into."""
    n = _vec(n)
    w = _perp(n)
    c = [p0[k] + n[k] * size / 2 for k in range(3)]
    return obox(c, n, w, size, size, size)


def teardrop(p0, p1, r, up):
    """Horizontal-hole teardrop (cylinder plus 45 deg roof pointing 'up')."""
    a = _vec([p1[k] - p0[k] for k in range(3)])
    u = _vec(up)
    w = _cross(a, u)
    L = math.sqrt(sum((p1[k] - p0[k]) ** 2 for k in range(3)))
    mid = [(p0[k] + p1[k]) / 2 + u[k] * r / math.sqrt(2) for k in range(3)]
    d = _vec([u[k] + w[k] for k in range(3)])
    roof = obox(mid, a, d, L, r, r)
    return U(cyl(p0, p1, r), roof)


def hole(p0, p1, d, up=None):
    """Round hole, or a teardrop when the hole is horizontal in the print."""
    if up is None:
        return cyl(p0, p1, d / 2)
    a = _vec([p1[k] - p0[k] for k in range(3)])
    if abs(sum(a[k] * up[k] for k in range(3))) > 0.9:
        return cyl(p0, p1, d / 2)
    return teardrop(p0, p1, d / 2, up)


def octagon_y(cx, cz, h, y0, y1):
    """Regular octagon prism along Y, half-width h (flats on X and Z)."""
    b = box(cx - h, cx + h, y0, y1, cz - h, cz + h)
    r = obox([cx, (y0 + y1) / 2, cz], [1, 0, 1], [0, 1, 0], 2 * h, abs(y1 - y0), 2 * h)
    return I(b, r)


def to_world(body, m):
    TB.transform(body, m)
    return body


def frame(origin, xdir, zdir):
    x = _vec(xdir)
    z = _vec(zdir)
    y = _cross(z, x)
    m = adsk.core.Matrix3D.create()
    m.setWithCoordinateSystem(P(*origin), V(*x), V(*y), V(*z))
    return m


def chain(local, parent):
    m = local.copy()
    m.transformBy(parent)
    return m


# ----------------------------------------------------------------------------
# Servo geometry, measured from the official Waveshare ST3215 STEP
# (files.waveshare.com/upload/5/59/ST3215-3D.zip, drawing "SCS215" 2022-06-08).
# STEP frame -> servo frame L:  x_L = x_S + 25.5,  y_L = -z_S,  z_L = y_S + 9.025
# ----------------------------------------------------------------------------
ST = dict(
    L0=-10.11, L1=35.11, W=12.36,          # case along X, half width
    top=13.825, bot=-16.575,               # seating faces of the case (top cover / bottom cover)
    boss_top=16.525, boss_x=(9.6, 33.7), boss_y=6.92,     # label boss on the top face
    pad_bot=-18.475, pad_x=(16.25, 29.7), pad_y=9.24,     # raised pad on the bottom face
    horn=18.625, horn_r=9.6, horn_t=4.5, hub=-19.175,     # horn faces, Ø19.2 aluminium horns
    holes_top=[(29.0, -10.25), (29.0, 10.25)],            # M2 holes usable beside the horn
    holes_bot=[(32.75, -10.25), (32.75, 10.25)],
    horn_holes=[(7, 0), (0, 7), (-7, 0), (0, -7)],        # M3 tapped, on a 14 mm circle
    step_to_L=(25.5, 9.025),
)
STS3032 = dict(L=23.2, W=12.1, H=28.5, shaft_off=6.0)    # micro bus servo (attachments)

# Clamp-plate and horn-arm planes, shared by the body plates and every bracket
PT = 5.0                                   # plate thickness
Y_PP = (ST['top'], ST['top'] + PT)         # + clamp plate (13.825 .. 18.825)
Y_MP = (ST['bot'] - PT, ST['bot'])         # - clamp plate (-21.575 .. -16.575)
Y_PA = (21.125, 26.125)                    # + horn arm (pad 2.5 mm)
Y_MA = (-28.125, -23.125)                  # - horn arm (pad 4.5 mm)
Y_TP = (27.625, 32.625)                    # tibia + horn arm (outside the femur arms)
Y_TM = (-34.625, -29.625)                  # tibia - horn arm
PAD_R = 9.3
Y_LEG = (Y_PA[1] + Y_MA[0]) / 2.0          # leg centre plane (-1.0)
FOOT_Y = {1: 35.0, -1: -37.0}               # foot-lead plane outside the tibia arms (by cable side)
FOOT_X = 4.5                                # foot-lead groove on the tibia, clear of the horn-screw heads


def st_envelope():
    """ST3215 / STS3250 envelope in frame L (used when the STEP is missing and for
    collision-light exports)."""
    s = ST
    b = box(s['L0'], s['L1'], -s['W'], s['W'], s['bot'], s['top'])
    U(b, box(s['boss_x'][0], s['boss_x'][1], -s['boss_y'], s['boss_y'], s['top'] - 0.01, s['boss_top']))
    U(b, box(s['pad_x'][0], s['pad_x'][1], -s['pad_y'], s['pad_y'], s['pad_bot'], s['bot'] + 0.01))
    U(b, cyl((0, 0, s['horn'] - s['horn_t']), (0, 0, s['horn']), s['horn_r']))
    U(b, cyl((0, 0, -s['horn']), (0, 0, -s['horn'] + 3.35), s['horn_r']))
    U(b, cyl((0, 0, s['hub']), (0, 0, -s['horn'] + 0.01), 3.0))
    for (x, y) in s['horn_holes']:
        for z0, z1 in ((s['horn'] - 2.5, s['horn'] + 0.1), (-s['horn'] - 0.1, -s['horn'] + 2.2)):
            D(b, cyl((x, y, z0), (x, y, z1), 1.25))
    return b


def sts3032_envelope():
    s = STS3032
    b = box(-s['shaft_off'], s['L'] - s['shaft_off'], -s['W'] / 2, s['W'] / 2, -s['H'] + 4, 0)
    U(b, cyl((0, 0, -0.01), (0, 0, 4), 2.4))
    return b


def servo_frame(origin, case_dir, shaft_dir):
    """Matrix placing servo frame L: +X_L along case_dir, +Z_L along shaft_dir."""
    return frame(origin, case_dir, shaft_dir)


def xf_body(temp, m):
    TB.transform(temp, m)
    return temp


def clamp_tools(m, g, top=True, bottom=True, cb_top=True, cb_bot=True):
    """Pockets and M2 screw holes in the clamp plates of a servo placed with m."""
    s, f = ST, g.fit
    t = []
    if top:
        t.append(box(s['boss_x'][0] - 0.4, s['boss_x'][1] + 0.4, -s['boss_y'] - 0.4, s['boss_y'] + 0.4, s['top'] - 0.2, s['boss_top'] + 0.4))
        for (x, y) in s['holes_top']:
            t.append(cyl((x, y, s['top'] - 1), (x, y, Y_PP[1] + 1), g.m2 / 2))
            if cb_top:
                t.append(cyl((x, y, Y_PP[1] - 2.2), (x, y, Y_PP[1] + 1), 2.3))
    if bottom:
        t.append(box(s['pad_x'][0] - 0.4, s['pad_x'][1] + 0.4, -s['pad_y'] - 0.4, s['pad_y'] + 0.4, s['pad_bot'] - 0.4, s['bot'] + 0.2))
        for (x, y) in s['holes_bot']:
            t.append(cyl((x, y, s['bot'] + 1), (x, y, Y_MP[0] - 1), g.m2 / 2))
            if cb_bot:
                t.append(cyl((x, y, Y_MP[0] + 2.2), (x, y, Y_MP[0] - 1), 2.3))
    return [xf_body(x, m) for x in t]


def servo_pockets(m):
    """Clearance round the label boss and the bottom pad of a servo placed with m."""
    s = ST
    t = [box(s['boss_x'][0] - 0.5, s['boss_x'][1] + 0.5, -s['boss_y'] - 0.5, s['boss_y'] + 0.5, s['top'] - 0.2, s['boss_top'] + 0.6),
         box(s['pad_x'][0] - 0.5, s['pad_x'][1] + 0.5, -s['pad_y'] - 0.5, s['pad_y'] + 0.5, s['pad_bot'] - 0.6, s['bot'] + 0.2)]
    return [xf_body(x, m) for x in t]


def horn_arm(m, g, side, up, planes=None):
    """Bolt holes through the pad + arm on one horn face of a servo placed with m.
    side=+1 driven horn (pad 2.5 mm), -1 idler horn (pad 4.5 mm).  4 x M3 on the
    14 mm circle (horns are tapped M3) with head counterbores, and a 7 mm centre
    hole for the hub screw / idler post.  Teardrops when horizontal in the print."""
    s = ST
    pa, ma = planes or (Y_PA, Y_MA)
    sg = 1 if side > 0 else -1
    p0 = s['horn'] * sg
    a1 = pa[1] if side > 0 else ma[0]
    z0, z1 = p0 - 0.5 * sg, a1 + 1.0 * sg
    tools = [hole(_apply(m, (0, 0, z0)), _apply(m, (0, 0, z1)), 7.0, up)]
    for (x, y) in s['horn_holes']:
        tools.append(hole(_apply(m, (x, y, z0)), _apply(m, (x, y, z1)), g.clr, up))
        tools.append(hole(_apply(m, (x, y, a1 - 2.0 * sg)), _apply(m, (x, y, z1)), 6.2, up))
    return None, tools


def pad_solid(m, side, up_world, planes=None):
    """Horn pad as a teardrop (roof towards the print's up direction) so it prints
    without support when its axis is horizontal on the bed."""
    s = ST
    pa, ma = planes or (Y_PA, Y_MA)
    if side > 0:
        p0, p1 = s['horn'], pa[0] + 0.01
    else:
        p0, p1 = -s['horn'], ma[1] - 0.01
    a = _apply(m, (0, 0, p0))
    b = _apply(m, (0, 0, p1))
    return hole(a, b, 2 * PAD_R, up_world)


def _apply(m, p):
    q = P(*p)
    q.transformBy(m)
    return (q.x * 10, q.y * 10, q.z * 10)


class Geo:
    def __init__(self, d):
        self.T = PT
        self.Lc = pmm(d, 'leg_coxa')
        self.Lf = pmm(d, 'leg_femur')
        self.Lt = pmm(d, 'leg_tibia')
        self.inset = pmm(d, 'body_coxaInset')
        self.fit = pmm(d, 'fit_clearance')
        self.slide = pmm(d, 'slide_clearance')
        self.ins = pmm(d, 'insert_hole')
        self.clr = pmm(d, 'clear_hole')
        self.m2 = pmm(d, 'm2_clear')
        self.cham = pmm(d, 'first_layer_chamfer')
        self.fil = pmm(d, 'root_fillet')
        self.z_top = Y_PP[1]          # top face of the top plate
        self.z_bot = Y_MP[0]          # bottom face of the bottom plate
        self.yc = Y_LEG


# ----------------------------------------------------------------------------
# Leg parts
# ----------------------------------------------------------------------------
def femur_servo_m(g):
    """Femur servo in the coxa frame: shaft along +Y at (Lc, 0, 0), case down."""
    return servo_frame((g.Lc, 0, 0), (0, 0, -1), (0, 1, 0))


def tibia_servo_m(g):
    """Tibia servo in the femur frame: shaft along +Y at (Lf, 0, 0), case back towards the femur root."""
    return servo_frame((g.Lf, 0, 0), (-1, 0, 0), (0, 1, 0))


def coxa_servo_m_local():
    """Coxa servo in the coxa frame (at zero angle): shaft up, case pointing back into the body."""
    return servo_frame((0, 0, 0), (-1, 0, 0), (0, 0, 1))


def part_coxa_main(g):
    """Coxa bracket: top and bottom horn arms round the coxa servo, a vertical web,
    and the + clamp plate of the femur servo.  Prints on the + clamp plate's outer
    face (+Y down), so every wall is vertical."""
    Lc = g.Lc
    yb0, yb1 = Y_MP[1], Y_PP[1]          # the main part spans y -16.575 .. 18.825
    up = (0, -1, 0)                       # print 'up' (bed is the +Y face)
    xw0, xw1 = 17.0, 26.0
    s = box(xw0, xw1, yb0, yb1, -38.5, Y_PA[1])                  # web (down to the cap's lowest screw)
    mz = coxa_servo_m_local()
    for (z0, z1) in ((Y_PA[0], Y_PA[1]), (Y_MA[0], Y_MA[1])):
        arm = box(0, xw1, yb0, yb1, z0, z1)
        U(arm, I(cyl((0, 0, z0), (0, 0, z1), 17.0), box(-20, 0.01, yb0, yb1, z0, z1)))
        U(s, arm)
    U(s, pad_solid(mz, +1, up))
    U(s, pad_solid(mz, -1, up))
    # + clamp plate for the femur servo (below the horn, plus a link up to the web)
    U(s, box(xw0, Lc + 16, Y_PP[0], Y_PP[1], -38.5, -12.5))
    U(s, box(xw0, 30, Y_PP[0], Y_PP[1], -12.5, 10.0))
    tools = []
    for side in (+1, -1):
        tools += horn_arm(mz, g, side, up)[1]
    # (horn-arm holes are vertical in the robot = horizontal in the print -> teardrops)
    tools = [t for t in tools]
    mf = femur_servo_m(g)
    tools += clamp_tools(mf, g, top=True, bottom=False)
    tools.append(cyl((Lc, Y_PP[0] - 1, 0), (Lc, Y_PP[1] + 1, 0), 11.5))   # horn clearance
    hx, hy, hz = Harness.COXA_HUB
    for dx in (-4.5, 4.5):                       # bus-hub standoffs on the top arm
        U(s, hole((hx + dx, hy, Y_PA[1] - 0.01), (hx + dx, hy, hz), 5.0, up))
        tools.append(cyl((hx + dx, hy, hz - 5.0), (hx + dx, hy, hz + 0.1), 0.9))
    # inserts for the CoxaCap screws (from the - side into the web and the link)
    for (x, z) in ((21.5, -34.0), (21.5, -24.0), (21.5, -2.0)):
        tools.append(cyl((x, yb0 - 1, z), (x, yb0 + 6, z), g.ins / 2))
    return dict(solid=s, tools=tools, bed=(0, 1, 0), fillet_axis=(0, 1, 0), edge_break=True)


def part_coxa_cap(g):
    """The - clamp plate of the femur servo, bolted to the coxa bracket.  Prints flat."""
    Lc = g.Lc
    s = box(17.0, Lc + 16, Y_MP[0], Y_MP[1], -38.5, -12.5)
    U(s, box(17.0, 30, Y_MP[0], Y_MP[1], -12.5, 10.0))
    U(s, box(17.0, 26.0, Y_MP[0], Y_MP[1], -38.5, 10.0))
    tools = clamp_tools(femur_servo_m(g), g, top=False, bottom=True)
    tools.append(cyl((Lc, Y_MP[0] - 1, 0), (Lc, Y_MP[1] + 1, 0), 11.5))
    for (x, z) in ((21.5, -34.0), (21.5, -24.0), (21.5, -2.0)):
        tools.append(cyl((x, Y_MP[0] - 1, z), (x, Y_MP[1] + 1, z), g.clr / 2))
        tools.append(cyl((x, Y_MP[0] - 1, z), (x, Y_MP[0] + 2.2, z), 3.1))
    return dict(solid=s, tools=tools, bed=(0, -1, 0), fillet_axis=None, edge_break=True)


def _chamfer_corners_xz(b, corners, c, y0, y1):
    for (x, z, sx, sz) in corners:
        D(b, halfspace((x - sx * c, 0, z), (sx, 0, sz)))
    return b


def part_femur(g):
    """Femur: two horn arms round the femur servo that clamp the tibia servo case
    via bosses, joined by a bridge under the tibia servo.  Prints bridge-down."""
    Lf = g.Lf
    up = (0, 0, 1)
    zb, zt = -19.5, 14.0
    xe = 54.0
    s = None
    for (y0, y1) in (Y_PA, Y_MA):
        arm = box(-14, xe, y0, y1, zb, zt)
        D(arm, halfspace((-14 + 7, 0, zt), (-1, 0, 1)))      # 45 deg chamfers at the root
        D(arm, halfspace((-14 + 7, 0, zb), (-1, 0, -1)))
        s = arm if s is None else U(s, arm)
    mf = servo_frame((0, 0, 0), (0, 0, -1), (0, 1, 0))   # femur servo seen from the femur frame
    U(s, pad_solid(mf, +1, up))
    U(s, pad_solid(mf, -1, up))
    U(s, box(38.0, xe, Y_MA[0], Y_PA[1], zb, -13.6))      # bridge under the tibia servo
    # foot-lead fins on both arms: the lead snaps into the notch (it runs outside
    # the tibia arms at y +35 / -37 so it crosses the knee on the tibia axis)
    for (ya, yb, yl) in ((Y_PA[1] - 0.01, FOOT_Y[1] + 2.2, FOOT_Y[1]), (FOOT_Y[-1] - 2.2, Y_MA[0] + 0.01, FOOT_Y[-1])):
        fin = box(11.0, 21.0, ya, yb, zb, 3.0)          # near the femur axis: the folding tibia never reaches here
        D(fin, box(10.0, 22.0, yl - 1.7, yl + 1.7, -1.7, 4.0))
        U(s, fin)
    mt = tibia_servo_m(g)
    # bosses from the arms to the tibia servo case faces (teardrop: printed lying down)
    for (x, y) in ST['holes_top']:
        a, b = _apply(mt, (x, y, ST['top'] + 0.01)), _apply(mt, (x, y, Y_PA[0] + 0.01))
        U(s, hole(a, b, 6.4, up))
    for (x, y) in ST['holes_bot']:
        a, b = _apply(mt, (x, y, ST['bot'] - 0.01)), _apply(mt, (x, y, Y_MA[1] - 0.01))
        U(s, hole(a, b, 6.4, up))
    tools = servo_pockets(mt)
    for side in (+1, -1):
        tools += horn_arm(mf, g, side, up)[1]
    for (x, y) in ST['holes_top']:
        a, b = _apply(mt, (x, y, ST['top'] - 1)), _apply(mt, (x, y, Y_PA[1] + 1))
        tools.append(hole(a, b, g.m2, up))
        a = _apply(mt, (x, y, Y_PA[1] - 2.2))
        tools.append(hole(a, b, 4.6, up))
    for (x, y) in ST['holes_bot']:
        a, b = _apply(mt, (x, y, ST['bot'] + 1)), _apply(mt, (x, y, Y_MA[0] - 1))
        tools.append(hole(a, b, g.m2, up))
        a = _apply(mt, (x, y, Y_MA[0] + 2.2))
        tools.append(hole(a, b, 4.6, up))
    return dict(solid=s, tools=tools, bed=(0, 0, -1), fillet_axis=(0, 0, 1), edge_break=True,
                fillet_skip=lambda p: 9.5 < p[0] < 22.5 and abs(p[1]) > 27.5 and p[2] > -3.0)


def part_tibia(g):
    """Tibia: two horn arms round the tibia servo (outside the femur arms, so the
    leg can fold), a bridge below it with a chamfered inner edge, a tapered shank
    with a lightening window and a rounded foot for the TPU boot.
    Prints on its outer side (+X face down)."""
    h = 14.0
    yc = g.yc
    up = (-1, 0, 0)
    planes = (Y_TP, Y_TM)
    z_tip = -(g.Lt - 3.0 - 8.0)
    s = None
    for (y0, y1) in (Y_TP, Y_TM):
        arm = U(octagon_y(0, 0, h, y0, y1), box(-h, h, y0, y1, -46, 0))
        s = arm if s is None else U(s, arm)
    mt = servo_frame((0, 0, 0), (-1, 0, 0), (0, 1, 0))
    U(s, pad_solid(mt, +1, up, planes))
    U(s, pad_solid(mt, -1, up, planes))
    U(s, box(-h, h, Y_TM[0], Y_TP[1], -46, -38))
    shank = box(-h, h, Y_TM[0], Y_TP[1], z_tip + 14, -46)
    for sgn, ye in ((1, Y_TP[1]), (-1, Y_TM[0])):
        p0 = (0, ye, -46)
        p1 = (0, yc + sgn * 8.0, z_tip + 14)
        dy, dz = abs(p1[1] - p0[1]), abs(p1[2] - p0[2])
        D(shank, halfspace(p0, (0, sgn * dz, -dy)))
    U(s, shank)
    U(s, box(-h, h, yc - 8, yc + 8, z_tip, z_tip + 14.01))
    U(s, cyl((-h, yc, z_tip), (h, yc, z_tip), 8.0))
    D(s, box(-h - 1, h + 1, yc - 13, yc + 13, -66, -52))
    # 45 deg chamfer on the inner (-X) top edge of the bridge and arms: lets the
    # tibia fold further towards the femur before it touches
    D(s, I(halfspace((-h + 7, 0, -38), (-1, 0, 1)), box(-h - 1, 0, Y_TM[1], Y_TP[0], -47, -37.99)))
    tools = []
    for side in (+1, -1):
        tools += horn_arm(mt, g, side, up, planes)[1]
    tools += foot_switch_tools(g)
    # foot-lead grooves (2.6 wide, 1.3 deep) down both outer faces: arm face, then the shank face
    for ye, yt in ((Y_TP[1], yc + 8.0), (Y_TM[0], yc - 8.0)):
        tools.append(obox((FOOT_X, ye, -24.25), (0, 0, 1), (1, 0, 0), 43.5, 2.6, 2.6))
        a, b = (FOOT_X, ye, -46.0), (FOOT_X, yt, z_tip + 14.0)
        L = math.dist(a, b)
        n = _vec([0, b[1] - a[1], b[2] - a[2]])
        tools.append(obox(tuple((a[k] + b[k]) / 2 for k in range(3)), n, (1, 0, 0), L + 3.0, 2.6, 2.6))
    return dict(solid=s, tools=tools, bed=(1, 0, 0), fillet_axis=(1, 0, 0), edge_break=True)


# Omron D2F-01 pin-plunger micro-switch (12.8 x 5.8 x 6.5 mm, 2 x M2 holes 6.5 apart):
# foot contact sensor in the tibia tip, sealed under the TPU boot.
D2F = dict(L=12.8, W=5.8, H=6.5, hole_dx=3.25, hole_z=3.25, plunger_d=1.6, plunger_pressed=0.45)
BOOT_TRAVEL = 1.0      # the boot slides 1 mm up the tip when loaded and presses the plunger


def d2f_frame(g):
    """Switch frame in the tibia frame: origin at the bottom face centre, +Z up."""
    z_tip = -(g.Lt - 3.0 - 8.0)
    return frame((0, g.yc, z_tip - 5.0), (1, 0, 0), (0, 0, 1))


def d2f_envelope():
    s = D2F
    b = box(-s['L'] / 2, s['L'] / 2, -s['W'] / 2, s['W'] / 2, 0, s['H'])
    U(b, cyl((0, 0, 0.01), (0, 0, -s['plunger_pressed']), s['plunger_d'] / 2))
    for x in (-2.54, 0, 2.54):                       # PCB terminals
        U(b, box(x - 0.4, x + 0.4, -0.25, 0.25, s['H'] - 0.01, s['H'] + 3.5))
    for x in (-s['hole_dx'], s['hole_dx']):
        D(b, cyl((x, -s['W'] / 2 - 0.1, s['hole_z']), (x, s['W'] / 2 + 0.1, s['hole_z']), 1.05))
    return b


def foot_switch_tools(g):
    """Pocket (open to the +Y face, covered by the boot), plunger hole, 2 x M2
    screw holes, terminal space and the wire channel up to a cross hole above the
    boot."""
    z_tip = -(g.Lt - 3.0 - 8.0)
    yc = g.yc
    zb = z_tip - 5.0
    t = [box(-D2F['L'] / 2 - 0.2, D2F['L'] / 2 + 0.2, yc - D2F['W'] / 2 - 0.2, yc + 9.0, zb - 0.2, zb + D2F['H'] + 0.2),
         cyl((0, yc, zb + 0.1), (0, yc, z_tip - 8.5), 1.1),                                   # plunger / nub hole
         box(-4.0, 4.0, yc - 1.5, yc + 1.5, zb + D2F['H'], zb + D2F['H'] + 4.2),             # terminals
         cyl((0, yc, zb + D2F['H'] + 3.0), (0, yc, z_tip + 16.0), 1.6),                      # wire channel
         box(-1.6, FOOT_X + 1.6, yc - 12.0, yc + 12.0, z_tip + 14.4, z_tip + 17.6)]           # cross slot to both faces
    for x in (-D2F['hole_dx'], D2F['hole_dx']):
        t.append(cyl((x, yc - 9.0, zb + D2F['hole_z']), (x, yc, zb + D2F['hole_z']), 1.1))
        t.append(cyl((x, yc - 9.0, zb + D2F['hole_z']), (x, yc - 6.8, zb + D2F['hole_z']), 2.1))   # head counterbore
    # retention grooves for the boot's lips (both faces): 2.5 mm tall = lip 1.5 + 1 mm travel
    for sg in (1, -1):
        y0, y1 = sorted((yc + sg * 8.01, yc + sg * 7.2))
        t.append(box(-12.0, 12.0, y0, y1, z_tip + 7.5, z_tip + 10.0))
    return t


def part_foot_boot(g):
    """TPU boot: slides 1 mm on the tibia tip (lips in the tip's grooves); a nub in
    the sole presses the foot switch's plunger when the foot is loaded.  Drawn in
    the loaded (bottomed) position."""
    z_tip = -(g.Lt - 3.0 - 8.0)
    yc = g.yc
    outer = U(box(-16, 16, yc - 11, yc + 11, z_tip, z_tip + 12), cyl((-16, yc, z_tip), (16, yc, z_tip), 11.0))
    inner = U(box(-14.2, 14.2, yc - 8.2, yc + 8.2, z_tip, z_tip + 13), cyl((-14.2, yc, z_tip), (14.2, yc, z_tip), 8.2))
    D(outer, inner)
    U(outer, cyl((0, yc, z_tip - 8.6), (0, yc, z_tip - 5.0 - D2F['plunger_pressed'] - 0.05), 0.9))      # nub
    for sg in (1, -1):                                                                               # lips
        y0, y1 = sorted((yc + sg * 8.21, yc + sg * 7.4))
        U(outer, box(-11.5, 11.5, y0, y1, z_tip + 8.5, z_tip + 10.0))
    return dict(solid=outer, tools=[], bed=(0, 0, 1), fillet_axis=None, chamfer=False, fillet2=False)


# ----------------------------------------------------------------------------
# Body
# ----------------------------------------------------------------------------
class Body:
    def __init__(self, d, g):
        self.g = g
        self.L = pmm(d, 'body_length')
        self.W = pmm(d, 'body_width')
        self.legs = []
        for i in range(6):
            xs, zs = pmm(d, 'mounts_%d_x' % i), pmm(d, 'mounts_%d_z' % i)
            splay = pdeg(d, 'legSplay_%d' % i)
            X, Y = -xs, zs
            phi = splay if X > 0 else 180 - splay
            self.legs.append(dict(i=i, X=X, Y=Y, phi=phi,
                                  dir=(math.cos(math.radians(phi)), math.sin(math.radians(phi)), 0)))
        self.seam_top = 20.0
        self.seam_bot = -20.0
        self.pillars = [(44, 40), (-44, 40), (36, 0), (-36, 0), (32, -60), (-32, -60)]
        self.grid = []
        for x in (-20, 0, 20):
            for y in range(-70, 71, 20):
                if x == 0 and y == -50:
                    continue
                self.grid.append((x, y))
        self.pass_through = (0, -55, 30, 14)
        self.bracket_inserts = [(18, 124), (-18, 124)]
        self.panel_inserts = [(20, -112), (-20, -112)]
        self.panel_slot = (0, -117, 40, 6)
        self.carrier_screws = [(10, 53), (30, 80), (-5, 82)]
        self.tray_inserts = [(27, -30), (-27, -30), (27, -82), (-27, -82)]
        # Waveshare Servo Driver with ESP32: 65 x 30 mm, holes 58 x 23 (M2.5), long side along Y
        self.driver_c = (0.0, 89.0)
        self.driver_holes = [(sx * 11.5, 89.0 + sy * 29.0) for sx in (-1, 1) for sy in (-1, 1)]
        self.battery = dict(x=43.0, y=138.5, z=24.5, y0=-112.0)    # Gens Ace 3S 4000 mAh, slides out the rear
        self.hubs = [(34.5, -15.0), (-34.5, -15.0)]                # servo-bus hubs (18 x 18 perfboard, 5 x Molex 5264)
        self.hub_holes = [(hx + sx * 7.0, hy + sy * 7.0) for (hx, hy) in self.hubs for sx in (-1, 1) for sy in (-1, 1)]
        self.xt60 = (-32.0, 2.0, 34.0, 44.0)                       # mated XT60 pair on the bottom plate (x0, x1, y0, y1)
        self.fuse = (8.0, 34.0, 34.0, 47.0)                        # inline ATC fuse holder
        self.acs_slot = (17.0, 49.0, 20.3, 17.8)                   # ACS724 on the bottom plate, under the MOSFET switch
        self.acs_holes = [(17.0 + 2.5, 51.5), (17.0 + 17.8, 51.5), (17.0 + 2.5, 64.3), (17.0 + 17.8, 64.3)]
        self.coxa_cable_side = {0: 1, 1: -1, 2: 1, 3: 1, 4: 1, 5: -1}
        self.hub_side_port = {k: -v for k, v in self.coxa_cable_side.items()}
        # rev B2: control pod on the top-front of the top plate (Feather ESP32-S3 main
        # controller + ExpressLRS receiver), IMU pod under the belly, foot-lead routes
        self.pod = dict(x=27.0, y0=24.0, y1=108.0, h=13.0, floor=2.0, wall=2.0)
        self.pod_floor_screws = [(20, 30), (-20, 30), (20, 70), (-20, 70)]     # into the top-plate grid inserts
        self.pod_lid_bosses = [(23, 64), (-23, 64), (23, 80), (-23, 80)]
        self.pod_floor_holes = [(21.5, 38.0, 6.0), (-20.0, 38.0, 7.0), (8.0, 34.0, 5.0), (20.0, 100.0, 6.0)]   # (x, y, d) entries from inside
        self.pod_wall_entries = [40.0, 52.0, 70.0, 88.0]                        # foot leads (88 = spare) on both side walls
        self.feather = dict(x=0.0, y_usb=106.0)                                 # Feather ESP32-S3, USB-C forward
        self.rp1 = (-18.0, 93.5, 10.0, 11.0)                                    # ELRS receiver (cx, cy, w, h)
        self.antenna = (-18.0, 99.0)
        self.imu_c = (-10.0, -32.0)                                             # BNO085 board centre, under the belly
        self.imu_pod = (-34.0, 8.0, -50.0, -14.0, 12.0)                         # x0, x1, y0, y1, depth
        self.imu_inserts = [(-31.0, -47.0), (5.0, -47.0), (5.0, -17.0), (-31.0, -17.0)]
        self.imu_hole = (-28.5, -40.0, 7.0)
        # zip-tie slot pairs (centre, direction of the lead) for the foot leads on the top plate
        self.top_ties = []
        for (x, y, dx, dy) in ((34.5, -40.0, -0.24, 0.97), (39.7, 18.0, -0.22, 0.98), (33.25, 82.0, -0.21, 0.98)):
            self.top_ties += [((x, y), (dx, dy)), ((-x, y), (-dx, dy))]

    def foot_route(self, i):
        """Foot-switch lead over the top plate (world frame, lying on the plate)
        from where it comes down behind coxa i to its entry in the control pod's
        side wall.  Right-side legs listed; left legs are the mirror image."""
        zl, ze = Y_PP[1] + 0.9, Y_PP[1] + 5.0
        right = [
            [(35.0, 90.0), (31.5, 74.0), (29.5, 70.0)],                                     # 0 front: entry y 70
            [(46.0, 0.0), (42.0, 8.0), (37.0, 30.0), (31.0, 50.0), (29.5, 52.0)],           # 1 middle: entry y 52
            [(36.5, -75.0), (37.0, -50.0), (32.0, -30.0), (32.0, 18.0), (31.0, 34.0), (29.5, 40.0)],   # 2 rear: entry y 40
        ][i % 3]
        sx = 1 if i < 3 else -1
        lg = self.legs[i]
        d = lg['dir']
        s0 = (lg['X'] - 27.0 * d[0], lg['Y'] - 27.0 * d[1], zl)
        pts = [s0] + [(sx * x, y, zl) for (x, y) in right]
        y_e = pts[-1][1]
        pts += [(sx * 28.2, y_e, ze), (sx * 23.5, y_e, ze)]
        return pts

    def coxa_servo_m(self, lg):
        dx, dy = lg['dir'][0], lg['dir'][1]
        return servo_frame((lg['X'], lg['Y'], 0), (-dx, -dy, 0), (0, 0, 1))

    def outline(self, z0, z1):
        L2, W2 = self.L / 2, self.W / 2
        b = box(-W2 * 1.15, W2 * 1.15, -L2, L2, z0, z1)
        for sx in (1, -1):
            for sy in (1, -1):
                p0 = (sx * W2 * 0.75, sy * L2, 0)
                p1 = (sx * W2 * 1.15, sy * 30.0, 0)
                dx, dy = p1[0] - p0[0], p1[1] - p0[1]
                n = _vec([dy, -dx, 0])
                if n[0] * sx < 0:
                    n = [-n[0], -n[1], 0]
                D(b, halfspace(p0, n))
        for lg in self.legs:
            d = lg['dir']
            p0 = (lg['X'] - d[0] * self.g.inset, lg['Y'] - d[1] * self.g.inset, 0)
            D(b, halfspace(p0, d))
        return b

    def keys(self, seam, toward, z0, z1, grow=0.0):
        out = None
        for cx in (-33, 33):
            n0, n1 = sorted((seam, seam + toward * 6))
            h0, h1 = sorted((seam + toward * 6, seam + toward * 12))
            zz0, zz1 = (z0 - 1, z1 + 1) if grow else (z0, z1)
            k = U(box(cx - 4 - grow, cx + 4 + grow, n0 - (grow if toward < 0 else 0.01), n1 + (grow if toward > 0 else 0.01), zz0, zz1),
                  box(cx - 8 - grow, cx + 8 + grow, h0 - grow, h1 + grow, zz0, zz1))
            out = k if out is None else U(out, k)
        return out

    def plate(self, which, half):
        g = self.g
        if which == 'top':
            z0, z1, seam, toward = Y_PP[0], Y_PP[1], self.seam_top, -1
        else:
            z0, z1, seam, toward = Y_MP[0], Y_MP[1], self.seam_bot, 1
        o = self.outline(z0, z1)
        if half == 'front':
            I(o, box(-500, 500, seam, 500, -500, 500))
        else:
            I(o, box(-500, 500, -500, seam, -500, 500))
        key_owner = 'front' if toward < 0 else 'rear'
        tools = []
        if half == key_owner:
            U(o, self.keys(seam, toward, z0, z1))
        else:
            tools.append(self.keys(seam, toward, z0, z1, grow=g.slide))
        zz = (z0 - 1, z1 + 1)
        inside = lambda y: (y > seam) == (half == 'front')
        for lg in self.legs:
            m = self.coxa_servo_m(lg)
            tools += clamp_tools(m, g, top=(which == 'top'), bottom=(which == 'bottom'))
            # cable strain-relief slot pairs beside each coxa servo
            if which == 'bottom':
                d = lg['dir']
                for side in (-1, 1):
                    cx = lg['X'] - d[0] * 18 - d[1] * side * 17
                    cy = lg['Y'] - d[1] * 18 + d[0] * side * 17
                    tools.append(obox((cx, cy, (z0 + z1) / 2), (d[0], d[1], 0), (-d[1], d[0], 0), 4.5, 2.2, 20))
        if which == 'top':
            for (x, y) in self.pillars:
                tools.append(cyl((x, y, zz[0]), (x, y, zz[1]), g.clr / 2))
            for (x, y) in self.grid + self.bracket_inserts + self.panel_inserts + self.carrier_screws:
                tools.append(cyl((x, y, zz[0]), (x, y, zz[1]), g.ins / 2))
            px, py, pw, ph = self.pass_through
            pt = box(px - pw / 2 + ph / 2, px + pw / 2 - ph / 2, py - ph / 2, py + ph / 2, *zz)
            U(pt, cyl((px - pw / 2 + ph / 2, py, zz[0]), (px - pw / 2 + ph / 2, py, zz[1]), ph / 2))
            U(pt, cyl((px + pw / 2 - ph / 2, py, zz[0]), (px + pw / 2 - ph / 2, py, zz[1]), ph / 2))
            tools.append(pt)
            sx, sy, sw, sh = self.panel_slot
            tools.append(box(sx - sw / 2, sx + sw / 2, sy - sh / 2, sy + sh / 2, *zz))
            # control-pod cable entries (the pod floor has matching holes)
            for (x, y, dd) in self.pod_floor_holes:
                tools.append(cyl((x, y, zz[0]), (x, y, zz[1]), dd / 2))
            # zip-tie slot pairs either side of each foot lead
            for ((x, y), (dx, dy)) in self.top_ties:
                n = _vec([dx, dy, 0])
                for side in (-1, 1):
                    c = (x - n[1] * side * 4.0, y + n[0] * side * 4.0, (z0 + z1) / 2)
                    tools.append(obox(c, (n[0], n[1], 0), (-n[1], n[0], 0), 3.2, 1.8, 20))
        else:
            for (x, y) in self.pillars:
                if inside(y):
                    U(o, cyl((x, y, z1 - 0.01), (x, y, Y_PP[0]), 4.5))
                tools.append(cyl((x, y, Y_PP[0] + 1), (x, y, Y_PP[0] - 6), g.ins / 2))
            for (x, y) in self.tray_inserts:
                tools.append(cyl((x, y, zz[0]), (x, y, zz[1]), g.ins / 2))
            for (x, y) in self.driver_holes + self.hub_holes + self.acs_holes:
                if inside(y):
                    U(o, cyl((x, y, z1 - 0.01), (x, y, z1 + 4), 3.0))
                tools.append(cyl((x, y, z1 - 3), (x, y, z1 + 5), 1.1))
            # IMU pod: lead hole and four inserts (set from below)
            ix, iy, idd = self.imu_hole
            tools.append(cyl((ix, iy, zz[0]), (ix, iy, zz[1]), idd / 2))
            for (x, y) in self.imu_inserts:
                tools.append(cyl((x, y, zz[0]), (x, y, z0 + 4.5), g.ins / 2))
        tools = [t for t in tools if t is not None]
        return dict(solid=o, tools=tools, bed=(0, 0, -1), fillet_axis=(0, 0, 1), fillet_r=4.0,
                    fillet_skip=lambda p: abs(p[1] - seam) < 14 or abs(p[2] - z0) > 6)


def part_battery_tray(b):
    g = b.g
    z0 = Y_MP[1]
    bt = b.battery
    y0, y1 = bt['y0'] - 2, bt['y0'] + bt['y'] + 3.5
    hw = bt['x'] / 2 + 1.0
    s = box(-hw - 2, hw + 2, y0, y1, z0, z0 + 2.0)
    yw = -96.0                                     # side walls stop here: the rear coxa servos are beside the pack's tail
    U(s, box(hw, hw + 2, yw, y1, z0, z0 + 17))
    U(s, box(-hw - 2, -hw, yw, y1, z0, z0 + 17))
    I(s, U(box(-hw - 2, hw + 2, yw - 0.01, y1 + 1, z0 - 1, z0 + 30), box(-22.5, 22.5, y0 - 1, yw + 0.01, z0 - 1, z0 + 30)))
    U(s, box(-hw - 2, hw + 2, y1 - 2.0, y1, z0, z0 + 17))
    for (x, y) in b.tray_inserts:
        U(s, box(hw + 1, 31, y - 4, y + 4, z0, z0 + 3) if x > 0 else box(-31, -hw - 1, y - 4, y + 4, z0, z0 + 3))
    tools = [box(-8, 8, y1 - 4, y1 + 1, z0 + 5, z0 + 18)]               # lead exit (XT60 + balance)
    for (x, y) in b.tray_inserts:
        tools.append(cyl((x, y, z0 - 1), (x, y, z0 + 5), g.clr / 2))
    for yc in (-75, -5):                                                  # 20 mm strap slots
        tools.append(box(-hw - 3, hw + 3, yc - 11, yc + 11, z0 + 3.5, z0 + 6.5))
    return dict(solid=s, tools=tools, bed=(0, 0, -1), fillet_axis=(0, 1, 0), fillet_r=1.0, edge_break=True)


def part_electronics_carrier(b):
    """Hangs under the front of the top plate: Pololu MOSFET switch, ACS724 current
    sensor and 5 V buck, each on four bosses (M2 self-tapping)."""
    g = b.g
    zt = Y_PP[0]
    s = box(-38, 38, 49, 74, zt - 3, zt)
    U(s, box(-31, 31, 73.99, 90, zt - 3, zt))          # clear of the front coxa servo cases
    tools = []
    for (x, y) in b.carrier_screws:
        tools.append(cyl((x, y, zt - 4), (x, y, zt + 1), g.clr / 2))
        tools.append(cyl((x, y, zt - 4), (x, y, zt - 1.0), 3.1))
    bosses = []
    for (x0, y0, w, h) in b.board_slots():
        for (x, y) in ((x0 + 2.5, y0 + 2.5), (x0 + w - 2.5, y0 + 2.5), (x0 + 2.5, y0 + h - 2.5), (x0 + w - 2.5, y0 + h - 2.5)):
            bosses.append((x, y))
    for (x, y) in bosses:
        U(s, cyl((x, y, zt - 7), (x, y, zt - 2.99), 2.8))
        tools.append(cyl((x, y, zt - 8), (x, y, zt - 1.5), 1.1))
    return dict(solid=s, tools=tools, bed=(0, 0, 1), fillet_axis=None, edge_break=True)


def _board_slots(self):
    """(x0, y0, w, h) of the boards hanging from the carrier (body frame, mm)."""
    return [(16.0, 50.0, 20.3, 22.9),      # Pololu Big MOSFET switch HP
            (-30.0, 69.0, 17.8, 20.3)]     # Pololu D36V28F5 5 V


Body.board_slots = _board_slots


def part_power_panel(b):
    """On the top plate at the rear: rocker (MOSFET-switch control), XT60 charge
    socket and JST-XH 4-pin balance socket.  Prints face-down."""
    g = b.g
    zt = Y_PP[1]
    s = box(-24, 24, -124, -100, zt, zt + 3)
    U(s, box(-24, 24, -124, -121, zt, zt + 22))
    tools = [box(8.4, 21.6, -125, -120, zt + 1.5, zt + 20.9)]            # KCD1 rocker cut-out 13.2 x 19.4
    tools.append(box(-10.2, 6.2, -125, -120, zt + 6.0, zt + 14.6))       # XT60 female socket 16.4 x 8.6
    tools.append(box(-23.2, -12.8, -125, -120, zt + 8.0, zt + 14.0))     # JST-XH 4-pin balance socket 10.4 x 6
    for (x, y) in b.panel_inserts:
        tools.append(cyl((x, y, zt - 1), (x, y, zt + 4), g.clr / 2))
        tools.append(cyl((x, y, zt + 1.0), (x, y, zt + 4), 3.1))
    sx, sy, sw, sh = b.panel_slot
    tools.append(box(sx - sw / 2, sx + sw / 2, sy - sh / 2, sy + sh / 2, zt - 1, zt + 4))   # cable slot (matches the plate)
    return dict(solid=s, tools=tools, bed=(0, 1, 0), fillet_axis=(1, 0, 0), fillet_r=2.0, edge_break=True)


def part_front_bracket(b, d):
    g = b.g
    zc = pmm(d, 'frontMount_y')
    yf = pmm(d, 'frontMount_z')
    zt = Y_PP[1]
    s = box(-25, 25, 116, yf - 3, zt, zt + 3)
    U(s, box(-25, 25, yf - 3, yf, Y_MP[0], zt + 3))
    up = (0, -1, 0)
    tools = []
    for x in (-13, 13):                               # HC-SR04 transducers (26 mm apart)
        tools.append(cyl((x, yf - 4, zc), (x, yf + 1, zc), 8.4))
    for x in (-21, 21):
        for z in (zc - 8.5, zc + 8.5):
            tools.append(cyl((x, yf - 4, z), (x, yf + 1, z), 0.9))
    tools.append(box(-8, 8, yf - 4, yf + 1, Y_MP[1] + 1.0, Y_MP[1] + 11.0))   # USB-C access
    for (x, y) in b.bracket_inserts:
        tools.append(hole((x, y, zt - 1), (x, y, zt + 4), g.clr, up))
    # VL53L1X time-of-flight sensor on a 30 deg wedge behind the front plate, looking
    # forward-down at the ground ahead (terrain look-ahead / step detection)
    F = tof_frame()
    wedge = xf_body(box(-9.8, 9.8, -6.6, 14.0, -8.0, 0.0), F)
    I(wedge, box(-60, 60, 0, yf - 0.01, zt + 2.99, 80))
    U(s, wedge)
    tools += [xf_body(t, F) for t in (
        box(-9.3, 9.3, -6.75, 6.75, -2.8, 0.5),                       # carrier pocket
        box(-5.5, 5.5, -4.5, 4.5, -0.5, 14.0),                        # window
        cyl((0, -5.0, -2.6), (0, -5.0, -9.0), 1.6),                    # lead channel
        cyl((6.36, 3.81, -2.6), (6.36, 3.81, -7.5), 0.8),             # M2 self-tapping x 2
        cyl((-6.34, 3.81, -2.6), (-6.34, 3.81, -7.5), 0.8))]
    return dict(solid=s, tools=tools, bed=(0, 1, 0), fillet_axis=(1, 0, 0), fillet_r=3.0,
                fillet_skip=lambda p: p[2] < zt + 1.0, edge_break=True)


TOF = dict(o=(17.0, 131.0, 28.0), tilt=30.0)          # on the bracket's top flange, looking forward-down


def tof_frame():
    t = math.radians(TOF['tilt'])
    return frame(TOF['o'], (1, 0, 0), (0, math.cos(t), -math.sin(t)))


def tof_board_m():
    """Pololu VL53L1X carrier STEP (12.7 x 17.8, sensor on +Z) -> tof frame."""
    return chain(frame((8.9, -6.35, -2.6), (0, 1, 0), (0, 0, 1)), tof_frame())


# ----------------------------------------------------------------------------
# Control pod (Feather ESP32-S3 + ELRS receiver) and IMU pod
# ----------------------------------------------------------------------------
def feather_m(b):
    """Adafruit 5323 STEP (board 0..50.8 x 0..22.86, USB-C at x = 0) -> world:
    long side along -Y, USB-C facing forward at y = b.feather['y_usb']."""
    return frame((b.feather['x'] - 11.43, b.feather['y_usb'], Y_PP[1] + b.pod['floor'] + 3.0), (0, -1, 0), (0, 0, 1))


def feather_env():
    b = box(0, 50.8, 0, 22.86, 0, 1.6)
    U(b, box(9.0, 48.0, 1.5, 21.3, 1.59, 4.2))                 # ESP32-S3 module + parts
    U(b, box(-1.1, 7.3, 7.4, 15.4, 1.59, 4.9))                 # USB-C
    return b


def feather_holes(b):
    m = feather_m(b)
    return [_apply(m, (x, y, 0))[:2] for (x, y) in ((2.54, 2.54), (2.54, 20.32), (48.26, 1.84), (48.26, 20.95))]


def part_control_pod(b):
    """Tray on the top-front of the top plate: Feather ESP32-S3 on 3 mm bosses, ELRS
    receiver, cable entries from below and from both side walls (foot leads), USB-C
    opening at the front.  Prints floor-down."""
    g, p = b.g, b.pod
    zt = Y_PP[1]
    x, y0, y1, h, fl, w = p['x'], p['y0'], p['y1'], p['h'], p['floor'], p['wall']
    s = box(-x, x, y0, y1, zt, zt + h)
    D(s, box(-x + w, x - w, y0 + w, y1 - w, zt + fl, zt + h + 1))
    up = (0, 0, 1)
    tools = []
    for (bx, by) in b.pod_lid_bosses:
        U(s, cyl((bx, by, zt + fl - 0.01), (bx, by, zt + h), 3.5))
        tools.append(cyl((bx, by, zt + h + 1), (bx, by, zt + h - 6), g.ins / 2))
    for (fx, fy) in feather_holes(b):
        U(s, cyl((fx, fy, zt + fl - 0.01), (fx, fy, zt + fl + 3.0), 2.5))
        tools.append(cyl((fx, fy, zt + fl + 3.5), (fx, fy, zt + 0.8), 1.05))
    for (sx_, sy_) in b.pod_floor_screws:
        tools.append(cyl((sx_, sy_, zt - 1), (sx_, sy_, zt + fl + 1), g.clr / 2))
    for (hx, hy, hd) in b.pod_floor_holes:
        tools.append(cyl((hx, hy, zt - 1), (hx, hy, zt + fl + 1), hd / 2))
    for ye in b.pod_wall_entries:
        for sg in (1, -1):
            tools.append(hole((sg * (x + 1), ye, zt + 5.0), (sg * (x - w - 1), ye, zt + 5.0), 3.2, up))
    tools.append(box(-7.0, 7.0, y1 - w - 2.5, y1 + 1, zt + 5.2, zt + 12.0))          # Feather USB-C
    tools.append(hole((17.0, y1 + 1, zt + 5.0), (17.0, y1 - w - 1, zt + 5.0), 3.2, up))   # ToF lead from the front bracket
    # (the receiver sits on foam tape between the Feather and the left wall)
    return dict(solid=s, tools=tools, bed=(0, 0, -1), fillet_axis=(0, 0, 1), fillet_r=2.0, edge_break=True)


def part_control_pod_lid(b):
    """Flat lid with the receiver's antenna post (the stock T-antenna stands
    vertically in it, clear of the Feather's ground plane and any metal)."""
    g, p = b.g, b.pod
    zt = Y_PP[1] + p['h']
    s = box(-p['x'], p['x'], p['y0'], p['y1'], zt, zt + 2.0)
    ax, ay = b.antenna
    U(s, cyl((ax, ay, zt + 1.99), (ax, ay, zt + 32.0), 3.0))
    tools = [cyl((ax, ay, zt - 1), (ax, ay, zt + 33.0), 1.6)]
    for (bx, by) in b.pod_lid_bosses:
        tools.append(cyl((bx, by, zt - 1), (bx, by, zt + 3), g.clr / 2))
    for k in range(4):                                            # vents over the Feather
        tools.append(box(-9.0, 9.0, 62.0 + k * 8.0, 64.0 + k * 8.0, zt - 1, zt + 3))
    return dict(solid=s, tools=tools, bed=(0, 0, -1), fillet_axis=(0, 0, 1), fillet_r=2.0, edge_break=True)


def bno_m(b):
    """Adafruit 4754 STEP (25.4 x 22.86, components +Z) -> world, under the belly."""
    cx, cy = b.imu_c
    return frame((cx - 12.7, cy - 11.43, Y_MP[0] - b.imu_pod[4] + 2.0 + 3.0), (1, 0, 0), (0, 0, 1))


def part_imu_pod(b):
    """Cover under the bottom plate near the centre of mass: BNO085 on four
    TPU-grommet bosses (vibration isolation), lead up through the plate.  Prints
    bottom-down."""
    g = b.g
    x0, x1, y0, y1, dp = b.imu_pod
    zt = Y_MP[0]
    s = box(x0, x1, y0, y1, zt - dp, zt)
    D(s, box(x0 + 2, x1 - 2, y0 + 2, y1 - 2, zt - dp + 2.0, zt + 1))
    tools = []
    for (x, y) in b.imu_inserts:
        U(s, cyl((x, y, zt - dp + 0.01), (x, y, zt), 3.5))
        tools.append(cyl((x, y, zt - dp - 1), (x, y, zt + 1), g.clr / 2))
        tools.append(cyl((x, y, zt - dp - 1), (x, y, zt - dp + 2.5), 3.1))
    m = bno_m(b)
    for (hx, hy) in ((2.54, 2.54), (2.54, 20.32), (22.86, 2.54), (22.86, 20.32)):
        x, y, _ = _apply(m, (hx, hy, 0))
        U(s, cyl((x, y, zt - dp + 1.99), (x, y, zt - dp + 5.0), 2.5))
        tools.append(cyl((x, y, zt - dp + 5.5), (x, y, zt - dp + 0.8), 1.05))
    return dict(solid=s, tools=tools, bed=(0, 0, -1), fillet_axis=(0, 0, 1), fillet_r=2.0, edge_break=True)


# ----------------------------------------------------------------------------
# Brain module (swappable top-deck autonomy module): Raspberry Pi 5 + active
# cooler, Luxonis OAK-D Lite, LDROBOT LD19 LiDAR, Pololu D36V50F5 5 V / 5.5 A buck.
# It slides onto the same dovetail as any attachment and plugs into J1.
# ----------------------------------------------------------------------------
BR = dict(x=53.0, y0=-80.0, y1=22.0, top=69.0, wall=2.0, pi=(-49.0, -76.0), pi_so=12.0,
          buck=(-47.5, -10.0), buck_so=7.0, ld19=(-8.0, -40.0),
          lid_bosses=[(49.5, -78.5), (-49.5, -78.5), (49.5, 19.5), (-49.5, 19.5)])
PI5 = dict(L=85.0, W=56.0, holes=[(3.5, 3.5), (61.5, 3.5), (3.5, 52.5), (61.5, 52.5)])


def brain_floor_z(g):
    return g.z_top + QR['base_h']            # rests on the receiver's top face


def part_brain_base(b):
    """Dovetail rail + floor + walls + standoffs (one print, floor down)."""
    g = b.g
    sc = g.slide
    z0 = brain_floor_z(g)
    zf = g.z_top + QR['floor'] + sc
    y0r = QR['y_rear'] + QR['stop'] + sc
    s = dovetail(zf, z0 + 0.01, QR['hw_bot'] - sc, QR['hw_top'] - sc, y0r, QR['y_front'])
    X, w = BR['x'], BR['wall']
    U(s, box(-X, X, BR['y0'], BR['y1'], z0, z0 + 5.0))
    walls = box(-X, X, BR['y0'], BR['y1'], z0 + 4.99, BR['top'])
    D(walls, box(-X + w, X - w, BR['y0'] + w, BR['y1'] - w, z0, BR['top'] + 1))
    U(s, walls)
    zfl = z0 + 5.0
    tools = [box(QR['key_x'][0] - 0.3, QR['key_x'][1] + 0.3, y0r - 1, QR['y_front'] + 1, zf - 1, zf + QR['key_h'] + 0.3 - sc)]
    px, py = BR['pi']
    for (hx, hy) in PI5['holes']:
        U(s, cyl((px + hx, py + hy, zfl - 0.01), (px + hx, py + hy, zfl + BR['pi_so']), 3.0))
        tools.append(cyl((px + hx, py + hy, zfl + BR['pi_so'] + 1), (px + hx, py + hy, zfl + BR['pi_so'] - 6), 1.05))
    bx, by = BR['buck']
    for (hx, hy) in ((2.16, 2.16), (23.24, 2.16), (2.16, 23.24)):
        U(s, cyl((bx + hx, by + hy, zfl - 0.01), (bx + hx, by + hy, zfl + BR['buck_so']), 2.6))
        tools.append(cyl((bx + hx, by + hy, zfl + BR['buck_so'] + 1), (bx + hx, by + hy, zfl + BR['buck_so'] - 5), 0.9))
    for (lx, ly) in BR['lid_bosses']:
        U(s, cyl((lx, ly, BR['top'] - 8.0), (lx, ly, BR['top']), 3.0))
        tools.append(cyl((lx, ly, BR['top'] + 1), (lx, ly, BR['top'] - 6), g.ins / 2))
    # J1 plug and lock screw (the hex key reaches it through the lid)
    yr = QR['y_rear']
    tools.append(box(-11.6, 11.6, yr - 10.6, yr - 3.6, z0 - 1, zfl + 1))
    lx, ly = QR['lock_brain']
    tools.append(cyl((lx, ly, z0 - 1), (lx, ly, zfl + 1), g.clr / 2))
    tools.append(cyl((lx, ly, zfl - 2.0), (lx, ly, zfl + 1), 3.1))
    # side vents (Pi 5 active cooler exhaust) and the camera-cable hole
    for k in range(8):
        yv = -68.0 + k * 9.0
        for sg in (1, -1):
            tools.append(box(sg * X - 3, sg * X + 3, yv, yv + 3.0, z0 + 17.0, z0 + 33.0) if sg > 0 else
                         box(-X - 3, -X + 3, yv, yv + 3.0, z0 + 17.0, z0 + 33.0))
    tools.append(hole((CAM_CABLE_X, BR['y1'] - 3, 52.0), (CAM_CABLE_X, BR['y1'] + 4, 52.0), 6.0, (0, 0, 1)))
    for xs in (30.0, -30.0):                                            # camera-mount screws (from inside)
        tools.append(hole((xs, BR['y1'] - 3, 55.0), (xs, BR['y1'] + 1, 55.0), g.clr, (0, 0, 1)))
    return dict(solid=s, tools=tools, bed=(0, 0, -1), fillet_axis=(0, 0, 1), fillet_r=2.0, edge_break=True,
                fillet_skip=lambda p: abs(p[0]) < 16.5 and p[2] < z0 + 0.5)


def part_brain_lid(b):
    g = b.g
    X = BR['x']
    zt = BR['top']
    s = box(-X, X, BR['y0'], BR['y1'], zt, zt + 2.0)
    lx, ly = BR['ld19']
    tools = [cyl((lx, ly, zt - 1), (lx, ly, zt + 3), 4.0)]                 # LD19 lead
    for (dx, dy) in ((-20.0, -17.0), (20.0, -17.0), (-20.0, 17.0), (20.0, 17.0)):
        tools.append(cyl((lx + dx, ly + dy, zt - 1), (lx + dx, ly + dy, zt + 3), 1.4))   # LD19 base screws (M2.5)
    bx, by = QR['lock_brain']
    tools.append(cyl((bx, by, zt - 1), (bx, by, zt + 3), 3.5))             # hex-key access to the lock screw
    for (px, py) in BR['lid_bosses']:
        tools.append(cyl((px, py, zt - 1), (px, py, zt + 3), g.clr / 2))
    for k in range(5):
        tools.append(box(28.0, 46.0, -70.0 + k * 9.0, -67.0 + k * 9.0, zt - 1, zt + 3))   # vents over the Pi ports
    return dict(solid=s, tools=tools, bed=(0, 0, -1), fillet_axis=None, edge_break=True)


OAK = dict(back=30.0, bottom=38.0)
CAM_CABLE_X = 20.0                         # OAK USB-C cable passes the front wall and the mount here


def oak_m():
    """Luxonis DM9095 enclosure STEP (x along the baseline, y up, lenses on -z) -> world:
    lenses forward (+Y), centred on x = 0, back face at y = OAK['back']."""
    return frame((-41.0, OAK['back'] + 8.77, OAK['bottom'] + 3.02), (1, 0, 0), (0, -1, 0))


def oak_envelope():
    return box(-4.5, 86.5, -3.0, 25.0, -9.0, 8.8)


def part_brain_cam_mount(b):
    """Camera bracket screwed to the outside of the brain's front wall: back plate,
    a shelf under the OAK-D Lite and two standoffs to its rear M2 holes.  Prints
    back-plate down."""
    g = b.g
    y0 = BR['y1']
    s = box(-46.0, 46.0, y0, y0 + 3.0, 34.6, 62.0)
    U(s, box(-46.0, 46.0, y0 + 2.99, OAK['back'] + 17.7, OAK['bottom'] - 2.5, OAK['bottom']))
    zc = OAK['bottom'] + 3.02 + 8.5
    for xs in (-37.5, 37.5):
        U(s, cyl((xs, y0 + 2.99, zc), (xs, OAK['back'] - 0.2, zc), 3.0))
    tools = []
    for xs in (-37.5, 37.5):      # M2 x 12 from the back plate into the camera's rear holes (fit before the bracket)
        tools.append(hole((xs, y0 - 1, zc), (xs, OAK['back'] + 1, zc), 2.4, (0, 1, 0)))
        tools.append(hole((xs, y0 - 1, zc), (xs, y0 + 2.0, zc), 4.2, (0, 1, 0)))
    tools += [hole((xs, y0 - 1, 55.0), (xs, y0 + 4, 55.0), g.ins, (0, 1, 0)) for xs in (30.0, -30.0)]
    tools.append(hole((CAM_CABLE_X, y0 - 1, 52.0), (CAM_CABLE_X, y0 + 4, 52.0), 6.0, (0, 1, 0)))
    return dict(solid=s, tools=tools, bed=(0, -1, 0), fillet_axis=(0, 1, 0), fillet_r=2.0, edge_break=True)


def pi5_envelope():
    """Raspberry Pi 5 + Active Cooler, board frame (85 x 56, ports on the x = 85 edge)."""
    b = box(0, PI5['L'], 0, PI5['W'], 0, 1.6)
    U(b, box(66.0, PI5['L'] + 2.5, 2.0, 54.0, 1.59, 17.6))            # USB / Ethernet stacks
    U(b, box(7.1, 57.9, 49.5, 54.5, 1.59, 10.1))                     # GPIO header
    return b


def pi5_cooler():
    """Raspberry Pi Active Cooler envelope (heatsink + fan), Pi board frame."""
    return box(10.0, 60.0, 8.0, 48.0, 3.2, 13.0)


def d36v50_env():
    b = box(0, 25.4, 0, 25.4, 0, 1.6)
    U(b, box(4.5, 21.0, 4.5, 21.0, 1.59, 3.4))
    U(b, box(5.0, 20.0, 5.0, 20.0, -6.1, 0.01))                # inductor underneath
    return b


def ld19_envelope():
    b = box(-27.0, 27.0, -23.15, 23.15, 0, 8.0)
    U(b, cyl((0, 0, 7.99), (0, 0, 34.8), 19.3))
    return b


def dovetail(z_bot, z_top, hw_bot, hw_top, y0, y1):
    s = box(-hw_bot, hw_bot, y0, y1, z_bot, z_top)
    for sx in (1, -1):
        p0 = (sx * hw_bot, 0, z_bot)
        dz = z_top - z_bot
        D(s, halfspace(p0, (sx * dz, 0, (hw_bot - hw_top))))
    return s


QR = dict(base_h=8.0, floor=2.0, hw_bot=14.0, hw_top=10.0, key_x=(4.0, 8.0), key_h=2.0,
          y_rear=-44.0, y_front=20.0, stop=4.0, lock=(24.0, -36.0), lock_brain=(25.0, -10.0))


def part_qr_receiver(b):
    g = b.g
    z0 = g.z_top
    zf = z0 + QR['floor']
    zt = z0 + QR['base_h']
    yr, yfr = QR['y_rear'], QR['y_front']
    s = box(-30, 30, yr, yfr, z0, zt)
    U(s, box(-26, 26, yr - 14, yr + 0.01, z0, zt))
    slot = dovetail(zf, zt + 0.01, QR['hw_bot'], QR['hw_top'], yr + QR['stop'], yfr + 1)
    U(slot, box(-QR['hw_top'], QR['hw_top'], yr + QR['stop'], yfr + 1, zf, zt + 1))
    D(s, slot)
    U(s, box(QR['key_x'][0], QR['key_x'][1], yr + QR['stop'] - 0.01, yfr, zf - 0.01, zf + QR['key_h']))
    tools = []
    for (x, y) in [(-20, -30), (20, -30), (-20, 10), (20, 10)]:
        tools.append(cyl((x, y, z0 - 1), (x, y, zt + 1), g.clr / 2))
        tools.append(cyl((x, y, z0 + 3), (x, y, zt + 1), 3.25))
    lx, ly = QR['lock']
    tools.append(cyl((lx, ly, zt + 1), (lx, ly, zt - 6), g.ins / 2))
    bx, by = QR['lock_brain']
    tools.append(cyl((bx, by, zt + 1), (bx, by, zt - 6), g.ins / 2))
    # J1 attachment port: JST-XH 8-pin (GND, GND, VBAT, VBAT, +5V, BUS DATA, AUX1, AUX2),
    # through to the pass-through.  AUX1/AUX2 = fire / aux for weapons, or the UART to the brain module.
    tools.append(box(-11.2, 11.2, yr - 10.2, yr - 4.0, z0 - 1, zt + 1))
    return dict(solid=s, tools=tools, bed=(0, 0, -1), fillet_axis=None, edge_break=True,
                fillet_skip=lambda p: abs(p[0]) < 16.5)   # the dovetail slot is a sliding fit: keep it sharp


def part_qr_adapter(b):
    g = b.g
    sc = g.slide
    zf = g.z_top + QR['floor'] + sc
    zt = g.z_top + QR['base_h']
    y0 = QR['y_rear'] + QR['stop'] + sc
    rail = dovetail(zf, zt + 0.01, QR['hw_bot'] - sc, QR['hw_top'] - sc, y0, QR['y_front'])
    U(rail, box(-35, 35, QR['y_rear'] + 2, QR['y_front'], zt, zt + 5))
    tools = [box(QR['key_x'][0] - 0.3, QR['key_x'][1] + 0.3, y0 - 1, QR['y_front'] + 1, zf - 1, zf + QR['key_h'] + 0.3 - sc)]
    for x in (-20, 0, 20):
        for y in (-30, -10, 10):
            tools.append(cyl((x, y, zt + 6), (x, y, zt + 5 - 5.5), g.ins / 2))
    lx, ly = QR['lock']
    tools.append(cyl((lx, ly, zt - 1), (lx, ly, zt + 6), g.clr / 2))
    return dict(solid=rail, tools=tools, bed=(0, 0, 1), fillet_axis=None, edge_break=True,
                fillet2_r=0.5)   # rail-root fillet sits inside the receiver's 0.8 mm lip round


# ----------------------------------------------------------------------------
# Wiring harness: every cable as a routed pipe, every connector as a body
# ----------------------------------------------------------------------------
# ST3215 bus sockets (from the STEP's socket block): side entry, one on each long
# side, 13.9 mm from the shaft, 8.5 mm below the horn mid-plane.  Molex 5264-3P
# plugs (Feetech/Waveshare 3-pin bus leads).
PORT_X, PORT_Z, PORT_Y = 13.9, -8.5, ST['W']
PLUG_L = 9.0
R_BUS, R_PWR, R_SIG = 1.6, 1.7, 1.0      # 3-core bus lead, 14 AWG silicone, 26 AWG signal
R_FOOT, R_I2C = 0.8, 1.1                 # 2-core 28 AWG foot lead, 4-core 28 AWG (I2C / UART) lead


def pipe(pts, r):
    b = None
    for a, c in zip(pts[:-1], pts[1:]):
        if sum((a[k] - c[k]) ** 2 for k in range(3)) < 1e-6:
            continue
        seg = cyl(a, c, r)
        b = seg if b is None else U(b, seg)
    for p in pts[1:-1]:
        U(b, sphere(p, r))
    return b


def servo_port(m, side):
    """(plug body, cable start point) for the bus socket on `side` (+1/-1 = +Y_L/-Y_L)
    of a servo placed with m, in the parent frame of m."""
    s = side
    plug = box(PORT_X - 2.5, PORT_X + 2.5, *sorted((s * PORT_Y, s * (PORT_Y + PLUG_L))), PORT_Z - 3.0, PORT_Z + 3.0)
    TB.transform(plug, m)
    return plug, _apply(m, (PORT_X, s * (PORT_Y + PLUG_L), PORT_Z))


def _mx(p):
    return (-p[0], p[1], p[2])


class Harness:
    """Cable routes.  Body cables in the robot frame; leg cables in the coxa /
    femur frames (they ride with those links; the service loops at the joints
    are drawn at the zero pose)."""

    def __init__(self, B, g):
        self.B, self.g = B, g

    # -- body ---------------------------------------------------------------
    HUB_PLUGS = [(-4.5, 6.0), (4.5, 6.0), (4.5, 0.0), (-4.5, -6.0), (4.5, -6.0), (-4.5, 0.0)]
    # roles: 0 trunk in, 1 J1 (right hub) / spare, 2 front leg, 3 middle leg, 4 rear leg, 5 spare

    def hub_plug(self, side, k):
        hx, hy = self.B.hubs[0 if side > 0 else 1]
        dx, dy = self.HUB_PLUGS[k]
        return (hx + side * dx, hy + dy, -1.0)           # grid is symmetric, so the turned left hub matches

    def body(self):
        B, g = self.B, self.g
        cables, plugs = [], []
        # servo bus: driver -> two side hubs -> six coxa servos
        for side in (1, -1):
            f = (lambda p: p) if side > 0 else _mx
            h0 = self.hub_plug(side, 0)
            cables.append(('Bus trunk %s' % ('R' if side > 0 else 'L'), R_BUS, [
                f((8, 55.5, -9)), f((8, 51, -3)), f((28, 48, 0.4)), f((28, -4, 0.4)), (h0[0], h0[1], 0.4), h0]))
        legs = B.legs
        for lg in legs:
            i = lg['i']
            side = 1 if lg['X'] > 0 else -1
            f = (lambda p: p) if side > 0 else _mx
            hs = B.hub_side_port[i]
            plug, p_end = servo_port(B.coxa_servo_m(lg), hs)
            plugs.append(('Plug coxa%d hub' % i, plug))
            h = self.hub_plug(side, 2 + (i % 3))
            up = (h[0], h[1], 2.5)
            if i % 3 == 0:            # front legs: north between the pillar and the middle servo
                if side > 0:
                    mid = [(43.0, h[1], 2.5), (43.0, -3, -2), (43.0, 24, -2), (37, 32, -2), (37, 50, -2), (20, 78, -3), (20, 104, -3)]
                else:
                    mid = [(-43.0, h[1], 2.5), (-43.0, -3, -2), (-43.0, 24, -2), (-52, 30, -3), (-56, 70, -6)]
            elif i % 3 == 1:          # middle legs: south of the hub then out to the socket
                mid = [(h[0], -27.5, 2.5), f((42, -27.5, 2.5)), f((48, -27, -6.0))]
            else:                     # rear legs
                mid = [(h[0], h[1], 6.5), (h[0], -28.5, 6.5), f((44, -34, 6.5)), f((50, -45, -3)), f((55, -75, -6))]
            cables.append(('Bus coxa%d' % i, R_BUS, [h, up] + mid + [p_end]))
        # J1 attachment port (bus + VBAT): receiver bay -> pass-through -> over the battery -> right hub
        zt = Y_PP[1]
        h4 = self.hub_plug(1, 1)
        cables.append(('J1 bus+power', R_BUS, [(0, -51.0, zt + 3.0), (0, -51.0, zt - 2.0), (4, -55, 11.9), (4, 44, 11.9), (4, 46.5, 8.0),
                                                (10, 43, 4.5), (28, 43, 4.5), (28, -7, 4.5), (h4[0], -7, 4.5), (h4[0], h4[1], 2.0), h4]))
        # power: battery XT60 lead -> XT60 pair -> ATC fuse -> ACS724 -> MOSFET switch -> loads
        bt = B.battery
        yb = bt['y0'] + bt['y']
        x0, x1, ya, yb2 = B.xt60
        cables.append(('Battery lead', R_PWR, [(-4, yb, -4), (-4, yb + 2.5, -4), (-4, 32.2, -10), (x0 - 2.5, 32.2, -12), (x0 - 2.5, (ya + yb2) / 2, -12), (x0, (ya + yb2) / 2, -12)]))
        fx0, fx1, fy0, fy1 = B.fuse
        fm = (fy0 + fy1) / 2
        cables.append(('XT60 to fuse', R_PWR, [(x1 + 0.5, (ya + yb2) / 2, -12), (fx0 - 0.5, fm, -11)]))
        sw, d36 = B.board_slots()
        ax0, ay0, aw, ah = B.acs_slot
        cables.append(('Fuse to ACS724', R_PWR, [(fx1 + 0.5, fm, -11), (fx1 + 2.5, fm + 1.5, -11), (fx1 + 2.5, ay0 - 1.0, -11), (ax0 + aw - 4, ay0 - 0.5, -10)]))
        cables.append(('ACS724 to switch', R_PWR, [(ax0 + aw - 4, ay0 + ah - 3, -8.4), (ax0 + aw - 4, ay0 + ah - 3, 2.4)]))
        # switched VBAT: to the driver's DC jack and to the buck (the hub feeds ride in the bus trunks)
        cables.append(('VBAT to driver', R_SIG + 0.3, [(sw[0] + 4, sw[1] + sw[3] + 0.5, 1.0), (17, 76, -1.0), (16.5, 76, -6.0)]))
        cables.append(('VBAT to buck', R_SIG, [(sw[0] + 8, sw[1] + sw[3] + 0.5, 1.0), (12, 88, 0.3), (-10, 88, 0.3), (-12.5, 90.5, 0.0)]))
        cables.append(('VBAT sense', 0.8, [(sw[0] + 13, sw[1] + sw[3] + 0.5, 1.0), (17.5, 70, -1.0), (15.8, 70, -6.0)]))
        # charge port (rear panel XT60) -> battery node at the XT60 pair, and the balance-lead extension
        sx, sy, sw_, sh = B.panel_slot
        for k, (x, pin, zz) in enumerate(((-12.5, -6.0, 10.0), (-8.5, 2.0, 5.0))):
            cables.append(('Charge lead %d' % k, R_PWR, [(pin, -120.0, zt + 10.0), (pin, -118.5, zt + zz), (x, sy, zt + zz), (x, sy, 11.9), (x, 46.5, 11.9),
                                                         (x, 46.5, -2.0)] + ([(x + 3, 44.5, -6.5), (x1 + 0.5, 36.0, -7.0)] if k == 0 else [(6.5, 47.0, -3.5)])))
        cables.append(('Balance extension', R_SIG + 0.3, [(-18, -120.0, zt + 11.0), (-17, sy, zt + 2.0), (-17, sy, 11.9), (-17, yb + 1.5, 11.9), (-17, yb + 1.5, 6.0)]))
        # panel rocker -> MOSFET switch control input
        cables.append(('Switch control', 0.8, [(15, -120.0, zt + 11.0), (18, sy, zt + 2.0), (18, sy, 11.9), (18, 46.5, 11.9), (18, 47, 1.5),
                                                (sw[0] + 1, sw[1] - 0.5, 1.5)]))
        # sensors: ACS724 out + VBAT sense to the driver, sonar to the driver
        cables.append(('ACS724 signal', 0.8, [(ax0 - 0.5, ay0 + 8, -9.5), (15.8, ay0 + 8, -9.5)]))
        yf = self.sonar_y
        zc = self.sonar_z
        zp = Y_PP[1] + B.pod['floor'] + 3.0 + 2.2          # Feather pad height (wires soldered to the pads)
        zi = Y_PP[1] + 3.2                                  # inside the pod, just above the floor
        # front sensors -> control pod (through the plate at (20, 100))
        cables.append(('Sonar', R_SIG, [(0, yf - 5.0, zc), (0, yf - 9, zc + 2), (10, 116, 8), (20.0, 104, 10), (20.0, 100, 14),
                                        (20.0, 100, zi), (12.6, 95.5, zp)]))
        F = tof_frame()
        c0, c1, c2 = _apply(F, (0, -5.0, -2.6)), _apply(F, (0, -5.0, -9.5)), _apply(F, (0, 10.0, -10.0))
        cables.append(('ToF sensor', R_FOOT, [c0, c1, c2, (17.0, 112.0, zi + 1.6), (17.0, 103.0, zi + 1.6), (12.6, 100.5, zp)]))
        # IMU (under the belly) -> through the bottom plate -> left corridor -> pod
        cx, cy = B.imu_c
        ix, iy, _ = B.imu_hole
        zpod = Y_MP[0] - B.imu_pod[4] + 2.0 + 3.0 + 2.5
        cables.append(('IMU lead', R_I2C, [(cx - 14.1, cy, zpod), (-25.5, -36, zpod), (ix, iy, zpod), (ix, iy, 4.0), (-24, -34, 10.5),
                                           (-24, 30, 10.5), (-21.6, 38, 13.5), (-21.6, 38, zi), (-12.6, 59.5, zp)]))
        # 5 V for the Feather from the D36V28F5 buck
        cables.append(('Feather 5V', R_FOOT, [(-21.0, 68.0, 3.5), (-19.5, 60, 8), (-18.4, 42, 8), (-18.4, 38, 12), (-18.4, 38, zi),
                                              (-12.6, 56.0, zp)]))
        # Feather <-> servo-bus bridge (Waveshare board, header pins 21/22 as UART)
        cables.append(('Bridge UART', R_I2C, [(13.0, 64.0, -2.4), (14.0, 57, -1.0), (13.0, 48, 6.0), (16.0, 44, 7.0), (21.0, 41, 8.0),
                                              (21.5, 38, 12.0), (21.5, 38, zi), (12.6, 56.0, zp)]))
        # J1 AUX1/AUX2 (weapon fire / brain UART) -> J1 on the receiver
        cables.append(('J1 aux', R_FOOT, [(6.0, 54.4, zp), (8.0, 34, zi), (8.0, 34, 12.5), (7.0, 30, 11.9), (7.0, -52, 11.9),
                                          (7.0, -50, 16.8), (3.0, -50, 19.0), (3.0, -50, zt + 3.0)]))
        # foot leads over the top plate into the pod's side walls, then to the Feather
        pin_y = {40.0: 63.0, 52.0: 70.0, 70.0: 78.0}
        for i in range(6):
            pts = B.foot_route(i)
            sx = 1 if i < 3 else -1
            if sx < 0 and pts[-1][1] == 40.0:
                pin = 64.0
            else:
                pin = pin_y[pts[-1][1]]
            cables.append(('Foot lead plate %d' % i, R_FOOT, pts + [(sx * 12.6, pin, zp)]))
        # ELRS receiver -> Feather, and its antenna up the lid post
        rx, ry, rw, rh = B.rp1
        cables.append(('Receiver CRSF', R_I2C, [(rx + rw / 2 + 0.2, ry, zi + 0.3), (-12.6, ry - 2.0, zp)]))
        ax, ay = B.antenna
        cables.append(('Receiver antenna', 0.7, [(ax, ry + rh / 2 + 0.2, zi + 0.4), (ax, ay, zi + 3.0), (ax, ay, Y_PP[1] + B.pod['h'] + 30.0)]))
        return cables, plugs

    def brain(self):
        """Cables inside the brain module (world frame)."""
        z0 = brain_floor_z(self.g) + 5.0
        px, py = BR['pi']
        bx, by = BR['buck']
        hdr_y = py + 52.5
        out = [('Brain power', R_I2C, [(-1.5, -51.0, z0 - 4.0), (-1.5, -51.0, z0 + 4.0), (-20.0, -30.0, z0 + 4.0), (-35.0, by - 1.2, z0 + 4.0),
                                         (-35.0, by - 1.2, z0 + BR['buck_so'] + 1.0)]),
               ('Brain UART', R_FOOT, [(1.8, -51.0, z0 - 4.0), (1.8, -51.0, z0 + 5.0), (1.8, -17.0, z0 + 5.0), (1.8, -17.0, z0 + 25.0),
                                       (px + 17.5, -18.0, z0 + 25.0), (px + 17.5, hdr_y, z0 + BR['pi_so'] + 11.0)]),
               ('Brain 5V Pi', R_I2C, [(-24.0, by - 1.2, z0 + BR['buck_so'] + 1.0), (px + 9.6, -15.0, z0 + 14.0), (px + 9.6, -21.0, z0 + 20.0),
                                       (px + 9.6, hdr_y, z0 + BR['pi_so'] + 11.0)]),
               ('OAK USB', 2.0, [(px + PI5['L'] + 6.0, py + 9.0, z0 + 21.0), (48.5, py + 16.0, z0 + 21.0), (48.5, 10.0, z0 + 21.0),
                                 (CAM_CABLE_X, 16.0, 52.0), (CAM_CABLE_X, OAK['back'] - 2.3, 52.0)]),
               ('LD19 lead', R_I2C, [(BR['ld19'][0], BR['ld19'][1], BR['top'] + 3.0), (BR['ld19'][0], BR['ld19'][1], BR['top'] - 3.0),
                                     (px + 30.0, -14.0, z0 + 29.0), (px + 20.0, hdr_y, z0 + BR['pi_so'] + 11.0)])]
        return out

    # -- legs ---------------------------------------------------------------
    COXA_HUB = (17.0, 0.0, 29.125)          # 3-socket bus hub on the coxa's top arm (centre of the PCB underside)
    COXA_HUB_PLUGS = [(0.0, -4.5), (0.0, 0.0), (0.0, 4.5)]   # in, to femur servo, to tibia servo
    PLUG_TOP = 11.5                          # plug top above the hub PCB underside

    def coxa_hub_plug(self, k):
        hx, hy, hz = self.COXA_HUB
        dx, dy = self.COXA_HUB_PLUGS[k]
        return (hx + dx, hy + dy, hz + self.PLUG_TOP)

    def leg(self, s):
        """One leg's bus leads.  The coxa servo feeds a 3-socket hub on top of the
        coxa bracket (crossing the coxa joint on its axis, above the top arm);
        from the hub one short lead goes to the femur servo and one goes over the
        femur arms, across the femur axis, to the tibia servo's top socket.  Each
        lead is split where it crosses a joint: the split point lies on that
        joint's axis, so the segments on either side stay put as it turns.
        s = which coxa-servo socket feeds the leg (+1/-1)."""
        g = self.g
        yo, yf = -s, s
        plug_c, p0 = servo_port(coxa_servo_m_local(), s)
        plug_to, pt_out = servo_port(tibia_servo_m(g), +1)          # tibia servo top socket (femur frame)
        plug_f, pf = servo_port(femur_servo_m(g), -1)               # femur servo outer socket (coxa frame)
        Lc = g.Lc
        zc = 32.0                                                    # coxa-axis crossing, above the top-arm screw heads
        h_in, h_f, h_t = (self.coxa_hub_plug(k) for k in range(3))
        out = {'body': [], 'coxa': [], 'femur': []}
        out['body'].append(('Bus coxa servo-hub A', R_BUS, [p0, (p0[0], yo * 24.5, PORT_Z), (-9, yo * 24.5, PORT_Z), (-9, yo * 24.5, zc), (0, 0, zc)]))
        out['coxa'].append(('Bus coxa servo-hub B', R_BUS, [(0, 0, zc), (6, h_in[1], zc + 13), (h_in[0], h_in[1], zc + 13), h_in]))
        out['coxa'].append(('Bus hub-femur', R_BUS, [h_f, (h_f[0], h_f[1], zc + 15.5), (28.5, PORT_Z, zc + 15.5), (28.5, PORT_Z, 12), (70, PORT_Z, 12), (70, PORT_Z, pf[2]), pf]))
        out['coxa'].append(('Bus hub-tibia A', R_BUS, [h_t, (h_t[0], h_t[1], zc + 19.5), (28.5, yf * 31.5, zc + 19.5), (28.5, yf * 31.5, 12), (Lc, yf * 31.5, 0)]))
        out['femur'].append(('Bus hub-tibia B', R_BUS, [(0, yf * 31.5, 0), (20, yf * 31.5, 24.5), (20, yf * 12, 24.5), (pt_out[0], PORT_Z, 24.5), pt_out]))
        # foot-switch lead (2-core 28 AWG silicone): crosses the coxa axis high above
        # the hub, runs outside the leg at y = FOOT_Y and crosses the femur and knee
        # axes on the axis; down the tibia it lies in the face grooves.
        yl = FOOT_Y[s]
        tz = -(g.Lt - 3.0 - 8.0)
        face = (Y_TP[1] - 0.5) if s > 0 else (Y_TM[0] + 0.5)
        ye_ = Y_TP[1] if s > 0 else Y_TM[0]
        yb_ = g.yc + 8.0 * s
        fr = (tz + 16.0 + 46.0) / (tz + 14.0 + 46.0)
        yt = ye_ + (yb_ - ye_) * fr - 0.5 * s                      # groove centre line at the cross slot, 0.5 mm in
        zx = 52.0
        out['body'].append(('Foot lead A', R_FOOT, [(0, 0, zx), (-12, 0, zx), (-20, 0, 28.0), (-27, 0, Y_PP[1] + 0.9)]))
        out['coxa'].append(('Foot lead B', R_FOOT, [(0, 0, zx), (10, 0.5 * yl, zx), (28.5, yl, 45.0), (28.5, yl, 12.0), (Lc, yl, 0)]))
        out['femur'].append(('Foot lead C', R_FOOT, [(0, yl, 0), (g.Lf, yl, 0)]))
        face_out = (Y_TP[1] + 1.0) if s > 0 else (Y_TM[0] - 1.0)
        out['tibia'] = [('Foot lead D', R_FOOT, [(0, yl, 0), (FOOT_X, face_out, -3.0), (FOOT_X, face, -5.0), (FOOT_X, face, -46.0), (FOOT_X, yt, tz + 16.0),
                                                   (0, g.yc, tz + 16.0), (0, g.yc, tz - 5.0 + D2F['H'] + 3.5)])]
        out['plugs_body'] = [('Plug coxa servo', plug_c)]
        out['plugs_coxa'] = [('Plug femur', plug_f)]
        out['plugs_femur'] = [('Plug tibia', plug_to)]
        out['plugs_tibia'] = []
        return out


def coxa_hub_env():
    """3 x Molex 5264 on a 14 x 14 perfboard; frame origin = PCB underside centre."""
    b = box(-7, 7, -7, 7, 0, 1.6)
    for (dx, dy) in Harness.COXA_HUB_PLUGS:
        U(b, box(dx - 4.0, dx + 4.0, dy - 2.25, dy + 2.25, 1.59, Harness.PLUG_TOP))
    return b


# ----------------------------------------------------------------------------
# Vendor STEP models
# ----------------------------------------------------------------------------
VENDOR = {
    # name: (file, matrix-builder from the STEP frame to the part frame, envelope builder)
    'ST3215': 'ST3215.step',
    'ServoDriverESP32': 'Servo Driver with ESP32.step',
    'PololuD36V28': 'd36v28fx-step-down-voltage-regulators.step',
    'PololuBigMOSFET': 'big-mosfet-slide-switch-with-reverse-voltage-protection.step',
    'PololuACS724': 'acs724-current-sensor-carrier.step',
    'Feather': '5323 Feather ESP32-S3.step',
    'BNO085': '4754 BNO085 STEMMA QT.step',
    'VL53L1X': 'vl53l1x-carrier.step',
    'PololuD36V50': 'd36v50fx-step-down-voltage-regulator.step',
    'OAKDLite': 'DM9095_enclosure.stp',
    'RPi5': 'rpi5_slim.step',          # slimmed copy of the official STEP (vendor/slim_pi5.py)
}


# Importing the Pi 5 STEP (the official 77 MB file, and the 4 MB slim copy made by
# vendor/slim_pi5.py) stalls Fusion inside this assembly on 2026-11-25: the main thread
# blocks right after the import, with no dialog.  Use the envelope until that is solved.
VENDOR_SKIP = {'RPi5'}


def step_to_L():
    """STEP frame of ST3215.step -> servo frame L."""
    m = adsk.core.Matrix3D.create()
    m.setWithCoordinateSystem(P(25.5, 0, 9.025), V(1, 0, 0), V(0, 0, 1), V(0, -1, 0))
    return m


def vendor_path(name):
    """Path of a downloaded vendor STEP, copied to a short temp path first
    (the repo can sit in a path longer than Windows' 260-character limit)."""
    if not VENDOR_DIR:
        return None
    if name in VENDOR_SKIP:
        return None
    p = os.path.join(VENDOR_DIR, VENDOR[name])
    if os.name == 'nt' and len(p) > 240:
        p = '\\\\?\\' + p
    if not os.path.exists(p):
        return None
    import shutil
    import tempfile
    short_dir = os.path.join(tempfile.gettempdir(), 'hexmk1_vendor')
    os.makedirs(short_dir, exist_ok=True)
    dst = os.path.join(short_dir, name + '.step')
    if not os.path.exists(dst) or os.path.getsize(dst) != os.path.getsize(p):
        shutil.copyfile(p, dst)
    return dst


# ----------------------------------------------------------------------------
# Fusion plumbing
# ----------------------------------------------------------------------------
def edge_mid(e):
    ok, t0, t1 = e.evaluator.getParameterExtents()
    ok, p = e.evaluator.getPointAtParameter((t0 + t1) / 2)
    return p


def concave_edges(body, axis, skip=None, any_geom=False):
    """Concave (inside-corner) edges: straight edges along `axis`, or with
    any_geom=True every straight, circular or arc edge in any direction."""
    out = adsk.core.ObjectCollection.create()
    curve_types = (adsk.core.Circle3D.classType(), adsk.core.Arc3D.classType())
    for e in body.edges:
        if e.faces.count != 2:
            continue
        geo = e.geometry
        if geo.objectType != adsk.core.Line3D.classType():
            if not (any_geom and geo.objectType in curve_types and e.length > 0.15):
                continue
            d = None
        else:
            d = geo.startPoint.vectorTo(geo.endPoint)
            if d.length < 0.15:
                continue
            d.normalize()
        if d is not None and axis is not None and abs(d.x * axis[0] + d.y * axis[1] + d.z * axis[2]) < 0.99:
            continue
        p = edge_mid(e)
        if skip is not None and skip((p.x * 10, p.y * 10, p.z * 10)):
            continue
        ns = []
        for f in e.faces:
            ok, n = f.evaluator.getNormalAtPoint(p)
            ns.append(n)
        inside = 0
        for (a, b) in ((ns[0], ns[1]), (ns[1], ns[0])):
            q = adsk.core.Point3D.create(p.x + (a.x - b.x) * 0.02, p.y + (a.y - b.y) * 0.02, p.z + (a.z - b.z) * 0.02)
            if body.pointContainment(q) == adsk.fusion.PointContainment.PointInsidePointContainment:
                inside += 1
        if inside == 2:
            out.add(e)
    return out


def bed_edges(body, bed):
    best, faces = None, []
    for f in body.faces:
        if f.geometry.objectType != adsk.core.Plane.classType():
            continue
        ok, n = f.evaluator.getNormalAtPoint(f.pointOnFace)
        if n.x * bed[0] + n.y * bed[1] + n.z * bed[2] < 0.999:
            continue
        p = f.pointOnFace
        h = p.x * bed[0] + p.y * bed[1] + p.z * bed[2]
        if best is None or h > best + 1e-4:
            best, faces = h, [f]
        elif abs(h - best) < 1e-4:
            faces.append(f)
    edges = adsk.core.ObjectCollection.create()
    for f in faces:
        for e in f.edges:
            edges.add(e)
    return edges


def try_fillet(comp, edges, r):
    if edges.count == 0:
        return 0
    fi = comp.features.filletFeatures
    inp = fi.createInput()
    inp.edgeSetInputs.addConstantRadiusEdgeSet(edges, adsk.core.ValueInput.createByReal(cm(r)), False)
    try:
        fi.add(inp)
        return edges.count
    except Exception:
        pass
    n = 0
    for e in list(edges):
        one = adsk.core.ObjectCollection.create()
        one.add(e)
        inp = fi.createInput()
        inp.edgeSetInputs.addConstantRadiusEdgeSet(one, adsk.core.ValueInput.createByReal(cm(r)), False)
        try:
            fi.add(inp)
            n += 1
        except Exception:
            pass
    return n


def try_chamfer(comp, body, bed, dist):
    for d_ in (dist, dist * 0.6):
        edges = bed_edges(body, bed)
        if edges.count == 0:
            return 0
        ch = comp.features.chamferFeatures
        inp = ch.createInput2()
        inp.chamferEdgeSets.addEqualDistanceChamferEdgeSet(edges, adsk.core.ValueInput.createByReal(cm(d_)), False)
        try:
            ch.add(inp)
            return edges.count
        except Exception:
            continue
    # all-at-once failed (usually a tiny edge where a hole meets the bed face):
    # chamfer edge by edge and skip the ones that won't take it
    n = 0
    for e in list(bed_edges(body, bed)):
        one = adsk.core.ObjectCollection.create()
        one.add(e)
        inp = comp.features.chamferFeatures.createInput2()
        inp.chamferEdgeSets.addEqualDistanceChamferEdgeSet(one, adsk.core.ValueInput.createByReal(cm(dist * 0.6)), False)
        try:
            comp.features.chamferFeatures.add(inp)
            n += 1
            body = comp.bRepBodies.item(0)
        except Exception:
            pass
    return n


def add_body(comp, temp, name):
    bf = comp.features.baseFeatures.add()
    bf.startEdit()
    b = comp.bRepBodies.add(temp, bf)
    bf.finishEdit()
    bf.name = name
    body = comp.bRepBodies.item(comp.bRepBodies.count - 1)
    body.name = name
    return body


def build_printed(comp, spec, g, name, log):
    body = add_body(comp, spec['solid'], name)
    nf = 0
    if spec.get('fillet_axis') is not None:
        edges = concave_edges(body, spec['fillet_axis'], spec.get('fillet_skip'))
        nf = try_fillet(comp, edges, spec.get('fillet_r', g.fil))
        body = comp.bRepBodies.item(0)
    if spec.get('fillet2', True):
        # every other inside corner (pad and boss roots, bridge-to-arm joints, pillar
        # roots ...) gets a smaller fillet; holes and pockets are cut afterwards
        edges = concave_edges(body, None, spec.get('fillet_skip'), any_geom=True)
        nf += try_fillet(comp, edges, spec.get('fillet2_r', 1.5))
        body = comp.bRepBodies.item(0)
    tools = [t for t in spec['tools'] if t is not None]
    if tools:
        tool = tools[0]
        for t in tools[1:]:
            U(tool, t)
        tb = add_body(comp, tool, name + '_cut')
        cf = comp.features.combineFeatures
        coll = adsk.core.ObjectCollection.create()
        coll.add(tb)
        inp = cf.createInput(body, coll)
        inp.operation = adsk.fusion.FeatureOperations.CutFeatureOperation
        inp.isKeepToolBodies = False
        cf.add(inp)
        body = comp.bRepBodies.item(0)
    nc = 0
    if spec.get('chamfer', True):
        nc = try_chamfer(comp, body, spec['bed'], g.cham)
        body = comp.bRepBodies.item(0)
    ne = 0
    if spec.get('edge_break', False):
        # break the sharp outside edges of the top (last-printed) face: no cut
        # fingers, no crack starters, and lead-ins for the screws
        bed = spec['bed']
        ne = try_fillet(comp, bed_edges(body, (-bed[0], -bed[1], -bed[2])), 0.8)
        body = comp.bRepBodies.item(0)
    body.name = name
    if comp.bRepBodies.count != 1:
        raise RuntimeError('%s has %d bodies (expected 1)' % (name, comp.bRepBodies.count))
    bb = body.boundingBox
    ext = [(bb.maxPoint.x - bb.minPoint.x) * 10, (bb.maxPoint.y - bb.minPoint.y) * 10, (bb.maxPoint.z - bb.minPoint.z) * 10]
    vol = body.volume  # cm3
    log.append(dict(part=name, bed=spec['bed'], extents_mm=[round(v, 1) for v in ext], volume_cm3=round(vol, 2),
                    fillets=nf, chamfer_edges=nc, edge_breaks=ne))
    return body


def new_comp(root, name, m=None):
    occ = root.occurrences.addNewComponent(m or adsk.core.Matrix3D.create())
    occ.component.name = name
    return occ


def add_occ(root, comp, m):
    return root.occurrences.addExistingComponent(comp, m)


def axis_line(comp, p0, p1):
    sk = comp.sketches.add(comp.xYConstructionPlane)
    sk.name = 'joint axis'
    sk.isVisible = False
    a = sk.modelToSketchSpace(P(*p0))
    b = sk.modelToSketchSpace(P(*p1))
    return sk.sketchCurves.sketchLines.addByTwoPoints(a, b)


def appearance(design, names):
    lib = None
    for l in app.materialLibraries:
        if l.name in ('Fusion Appearance Library', 'Fusion 360 Appearance Library'):
            lib = l
    if lib is None:
        return None
    for n in names:
        try:
            a = lib.appearances.itemByName(n)
        except Exception:
            a = None
        if a:
            return design.addAppearanceIfMissing(a) if hasattr(design, 'addAppearanceIfMissing') else a
    return None


def paint(comp, design, names):
    a = appearance(design, names)
    if a is None:
        return
    for b in comp.bRepBodies:
        try:
            b.appearance = a
        except Exception:
            pass


def revolute(root, occ_move, occ_fixed, line, occ_line, lo, hi, name):
    lp = line.createForAssemblyContext(occ_line)
    geo = adsk.fusion.JointGeometry.createByCurve(lp, adsk.fusion.JointKeyPointTypes.StartKeyPoint)
    inp = root.asBuiltJoints.createInput(occ_move, occ_fixed, geo)
    inp.setAsRevoluteJointMotion(adsk.fusion.JointDirections.CustomJointDirection, lp)
    j = root.asBuiltJoints.add(inp)
    j.name = name
    try:
        lim = j.jointMotion.rotationLimits
        lim.isMinimumValueEnabled = True
        lim.minimumValue = math.radians(lo)
        lim.isMaximumValueEnabled = True
        lim.maximumValue = math.radians(hi)
    except Exception:
        pass
    return j


def rigid(root, occs, name):
    c = adsk.core.ObjectCollection.create()
    for o in occs:
        c.add(o)
    rg = root.rigidGroups.add(c, False)
    rg.name = name
    return rg


# ----------------------------------------------------------------------------
# Build
# ----------------------------------------------------------------------------
MASS_G = {  # bought parts (datasheet / listing values), grams
    'ST3215': 69.0, 'STS3250': 74.5, 'STS3032': 20.0, 'Battery_3S_4000': 305.0,
    'ServoDriverESP32': 15.0, 'PololuD36V28': 3.0, 'PololuBigMOSFET': 4.0, 'PololuACS724': 2.0,
    'BusHub': 12.0, 'HC-SR04': 9.0, 'FuseHolder_ATC': 15.0, 'RockerSwitch_KCD1': 3.0,
    'XT60_socket': 4.0, 'BalanceSocket': 2.0,
    'Feather_ESP32-S3': 6.0, 'BNO085': 3.0, 'VL53L1X': 0.5, 'ELRS_RP1': 1.5, 'D2F_switch': 0.8,
    # brain module (counts as the top attachment)
    'RaspberryPi5_8GB': 46.0, 'Pi5_ActiveCooler': 21.0, 'OAK-D-Lite': 61.0, 'LD19': 47.0, 'D36V50F5': 7.0, 'microSD': 0.5,
}
JOINT_LIMITS = {'coxa': (-45, 45), 'femur': (-70, 10), 'tibia': (-120, 45)}   # Fusion sign: +femur = down, +tibia = fold in


def _board_matrix_from_step(w, h):
    """Pololu STEPs: board lies in XY from (0,0) to (w,h), PCB underside at z=0.
    Part frame: board centred on the origin, components on +Z."""
    m = adsk.core.Matrix3D.create()
    m.translation = adsk.core.Vector3D.create(cm(-w / 2), cm(-h / 2), 0)
    return m


def _driver_matrix_from_step():
    """Waveshare Servo Driver with ESP32 STEP: 65 x 30 board in XY with a corner
    near the origin.  Part frame: centred, long side along +Y."""
    m = adsk.core.Matrix3D.create()
    m.setWithCoordinateSystem(P(15.0, -32.5, 0), V(0, 1, 0), V(-1, 0, 0), V(0, 0, 1))
    return m


def build(out_dir=None, use_vendor=True):
    design = adsk.fusion.Design.cast(app.activeProduct)
    design.designType = adsk.fusion.DesignTypes.ParametricDesignType
    root = design.rootComponent
    ensure_params(design)
    if REBUILD:
        for k in range(root.asBuiltJoints.count - 1, -1, -1):
            root.asBuiltJoints.item(k).deleteMe()
        for k in range(root.rigidGroups.count - 1, -1, -1):
            root.rigidGroups.item(k).deleteMe()
        for k in range(root.occurrences.count - 1, -1, -1):
            root.occurrences.item(k).deleteMe()
    g = Geo(design)
    B = Body(design, g)
    log, printed, cache, vendor_used, counts = [], {}, {}, {}, {}
    I3 = adsk.core.Matrix3D.create
    BLACK = ('Plastic - Matte (Black)', 'Plastic - Glossy (Black)')
    ORANGE = ('Plastic - Matte (Orange)', 'Plastic - Glossy (Orange)', 'Plastic - Matte (Red)')
    BLUE = ('Plastic - Matte (Blue)', 'Plastic - Glossy (Blue)')
    METAL = ('Aluminum - Satin', 'Steel - Satin')

    def place(name, make, m, look):
        counts[name] = counts.get(name, 0) + 1
        if name in cache:
            return add_occ(root, cache[name], m)
        occ = new_comp(root, name, m)
        res = make()
        if isinstance(res, dict):
            build_printed(occ.component, res, g, name, log)
            printed[name] = (occ.component, res['bed'])
        else:
            add_body(occ.component, res, name)
        paint(occ.component, design, list(look))
        cache[name] = occ.component
        return occ

    def place_vendor(name, key, step_m, envelope, m, look):
        counts[name] = counts.get(name, 0) + 1
        if name in cache:
            return add_occ(root, cache[name], m)
        occ = new_comp(root, name, m)
        comp = occ.component
        path = vendor_path(key) if use_vendor else None
        if path:
            im = app.importManager
            n0 = comp.occurrences.count
            im.importToTarget(im.createSTEPImportOptions(path), comp)
            for k in range(n0, comp.occurrences.count):
                comp.occurrences.item(k).transform = step_m
            vendor_used[name] = VENDOR[key]
        else:
            add_body(comp, envelope(), name)
            paint(comp, design, list(look))
        cache[name] = comp
        return occ

    def board_env(w, h, t=8.0):
        return lambda: box(-w / 2, w / 2, -h / 2, h / 2, 0, t)

    def hang(x0, y0, w, h):
        """Board hanging upside-down from the electronics carrier bosses."""
        return frame((x0 + w / 2, y0 + h / 2, Y_PP[0] - 7.0), (1, 0, 0), (0, 0, -1))

    body_occs, plates = [], {}
    for which in ('top', 'bottom'):
        for half in ('front', 'rear'):
            nm = 'Plate_%s_%s' % (which.capitalize(), half.capitalize())
            plates[nm] = place(nm, lambda w=which, h=half: B.plate(w, h), I3(), BLACK)
            body_occs.append(plates[nm])
    for nm, fn in (('BatteryTray', lambda: part_battery_tray(B)), ('ElectronicsCarrier', lambda: part_electronics_carrier(B)),
                   ('PowerPanel', lambda: part_power_panel(B)), ('FrontSensorBracket', lambda: part_front_bracket(B, design)),
                   ('QR_Receiver', lambda: part_qr_receiver(B)), ('ControlPod', lambda: part_control_pod(B)),
                   ('ControlPodLid', lambda: part_control_pod_lid(B)), ('ImuPod', lambda: part_imu_pod(B))):
        body_occs.append(place(nm, fn, I3(), ORANGE))
    # the turret example and the brain module share the top mount: the brain is shown,
    # the turret example is kept (hidden) for its STL and sim mesh
    turret = [place('QR_AdapterPlate_Example', lambda: part_qr_adapter(B), I3(), BLUE),
              place('STS3032', sts3032_envelope, frame((0, -10, g.z_top + QR['base_h'] + 5 + STS3032['H'] - 4), (1, 0, 0), (0, 0, 1)), BLUE)]
    for o in turret:
        o.isLightBulbOn = False
    body_occs += turret

    # electronics
    zb = Y_MP[1]
    bt = B.battery
    body_occs.append(place('Battery_3S_4000', lambda: box(-bt['x'] / 2, bt['x'] / 2, bt['y0'], bt['y0'] + bt['y'], zb + 2.0, zb + 2.0 + bt['z']),
                           I3(), ('Plastic - Matte (Yellow)', 'Plastic - Glossy (Yellow)')))
    body_occs.append(place_vendor('ServoDriverESP32', 'ServoDriverESP32', _driver_matrix_from_step(), board_env(30, 65, 9),
                                  frame((B.driver_c[0], B.driver_c[1], zb + 4.0), (1, 0, 0), (0, 0, 1)), BLUE))
    slots = B.board_slots()
    body_occs.append(place_vendor('PololuBigMOSFET', 'PololuBigMOSFET', _board_matrix_from_step(20.32, 22.86), board_env(20.3, 22.9, 4),
                                  hang(*slots[0]), BLUE))
    ax0, ay0, aw, ah = B.acs_slot
    body_occs.append(place_vendor('PololuACS724', 'PololuACS724', _board_matrix_from_step(20.32, 17.78), board_env(20.3, 17.8, 3.3),
                                  frame((ax0 + aw / 2, ay0 + ah / 2, zb + 4.0), (1, 0, 0), (0, 0, 1)), BLUE))
    body_occs.append(place_vendor('PololuD36V28F5', 'PololuD36V28', _board_matrix_from_step(17.78, 20.32), board_env(17.8, 20.3, 7.7),
                                  hang(*slots[1]), BLUE))
    # rev B2 control electronics: Feather ESP32-S3 + ELRS receiver in the pod, BNO085 under the
    # belly, VL53L1X on the front bracket (vendor STEPs are placed in their own frames)
    I_ = adsk.core.Matrix3D.create()
    body_occs.append(place_vendor('Feather_ESP32-S3', 'Feather', I_, feather_env, feather_m(B), BLUE))
    body_occs.append(place_vendor('BNO085_IMU', 'BNO085', I_, lambda: box(0, 25.4, 0, 22.86, 0, 4.55), bno_m(B), BLUE))
    body_occs.append(place_vendor('VL53L1X_ToF', 'VL53L1X', I_, lambda: box(0, 12.7, 0, 17.78, 0, 2.2), tof_board_m(), BLUE))
    rx, ry, rw, rh = B.rp1
    body_occs.append(place('ELRS_RP1_Receiver', lambda: box(rx - rw / 2, rx + rw / 2, ry - rh / 2, ry + rh / 2, Y_PP[1] + 2.0, Y_PP[1] + 5.5),
                           I3(), BLACK))

    def hub():
        b_ = box(-9, 9, -9, 9, 0, 1.6)
        for (dx, dy) in Harness.HUB_PLUGS:
            U(b_, box(dx - 4.0, dx + 4.0, dy - 2.25, dy + 2.25, 1.59, 11.575))
        return b_
    for k, (hx, hy) in enumerate(B.hubs):
        m = frame((hx, hy, zb + 4.0), (1 if k == 0 else -1, 0, 0), (0, 0, 1))
        body_occs.append(place('BusHub_5x5264', hub, m, ('Plastic - Matte (Green)',)))
    x0, x1, ya, yb_ = B.xt60
    body_occs.append(place('XT60_pair', lambda: box(x0, x1, ya, yb_, zb, zb + 8.0), I3(), ('Plastic - Matte (Yellow)',)))
    fx0, fx1, fy0, fy1 = B.fuse
    body_occs.append(place('FuseHolder_ATC_20A', lambda: box(fx0, fx1, fy0, fy1, zb, zb + 11), I3(), BLACK))
    zc, yf = pmm(design, 'frontMount_y'), pmm(design, 'frontMount_z')

    def sonar():
        b_ = box(-22.5, 22.5, yf - 4.6, yf - 3, zc - 10, zc + 10)
        for x in (-13, 13):
            U(b_, cyl((x, yf - 3, zc), (x, yf + 9, zc), 8.0))
        return b_
    body_occs.append(place('HC-SR04', sonar, I3(), METAL))
    zt = Y_PP[1]
    body_occs.append(place('RockerSwitch_KCD1', lambda: box(9.0, 21.0, -134, -121.2, zt + 3.2, zt + 20.5), I3(), BLACK))
    body_occs.append(place('XT60_socket', lambda: box(-9.8, 5.8, -130, -121.2, zt + 6.4, zt + 14.2), I3(), ('Plastic - Matte (Yellow)',)))
    body_occs.append(place('BalanceSocket_XH4', lambda: box(-22.8, -13.2, -128, -121.2, zt + 8.4, zt + 13.6), I3(), BLACK))

    # wiring harness (body): one component per cable, plus the servo plugs
    H = Harness(B, g)
    H.sonar_y, H.sonar_z = yf, zc
    cables, plugs = H.body()
    WIRE = ('Plastic - Matte (Black)', 'Plastic - Glossy (Black)')
    PLUG = ('Plastic - Matte (White)', 'Plastic - Glossy (White)', 'Plastic - Matte (Yellow)')
    for name, r, pts in cables:
        body_occs.append(place('Harness ' + name, lambda pts=pts, r=r: pipe(pts, r), I3(), WIRE))
    for name, temp in plugs:
        body_occs.append(place(name, lambda t=temp: t, I3(), PLUG))
    harness_report = [dict(cable=n, d_mm=2 * r, length_mm=round(sum(math.dist(a, b) for a, b in zip(p[:-1], p[1:])), 0)) for n, r, p in cables]

    # brain module (autonomy): on the top mount instead of the turret example
    brain_occs = []
    z0b = brain_floor_z(g) + 5.0
    for nm, fn in (('BrainBase', lambda: part_brain_base(B)), ('BrainLid', lambda: part_brain_lid(B)),
                   ('BrainCamMount', lambda: part_brain_cam_mount(B))):
        brain_occs.append(place(nm, fn, I3(), ('Plastic - Matte (Grey)', 'Plastic - Matte (Gray)', 'Plastic - Matte (Blue)')))
    px_, py_ = BR['pi']
    pi_m = frame((px_, py_, z0b + BR['pi_so']), (1, 0, 0), (0, 0, 1))
    brain_occs.append(place_vendor('RaspberryPi5', 'RPi5', I_, pi5_envelope, pi_m, ('Plastic - Matte (Green)',)))
    brain_occs.append(place('Pi5_ActiveCooler', pi5_cooler, pi_m, METAL))
    brain_occs.append(place_vendor('OAK-D-Lite', 'OAKDLite', I_, oak_envelope, oak_m(), METAL))
    bxb, byb = BR['buck']
    brain_occs.append(place_vendor('PololuD36V50F5', 'PololuD36V50', I_, d36v50_env,
                                   frame((bxb, byb, z0b + BR['buck_so']), (1, 0, 0), (0, 0, 1)), BLUE))
    brain_occs.append(place('LD19_LiDAR', ld19_envelope, frame((BR['ld19'][0], BR['ld19'][1], BR['top'] + 2.0), (1, 0, 0), (0, 0, 1)), BLACK))
    brain_cables = H.brain()
    for name, r, pts in brain_cables:
        brain_occs.append(place('Harness ' + name, lambda pts=pts, r=r: pipe(pts, r), I3(), WIRE))
    harness_report += [dict(cable=n, d_mm=2 * r, length_mm=round(sum(math.dist(a, b) for a, b in zip(p[:-1], p[1:])), 0), module='brain')
                       for n, r, p in brain_cables]
    body_occs += brain_occs

    # legs
    servo_env = st_envelope
    leg_groups, lines = [], {}
    for lg in B.legs:
        i = lg['i']
        d = lg['dir']
        coxa_m = frame((lg['X'], lg['Y'], 0), (d[0], d[1], 0), (0, 0, 1))
        cs = place_vendor('ST3215', 'ST3215', step_to_L(), servo_env, B.coxa_servo_m(lg), BLACK)
        cx = place('CoxaMain', lambda: part_coxa_main(g), coxa_m, ORANGE)
        cc = place('CoxaCap', lambda: part_coxa_cap(g), coxa_m, ORANGE)
        fs = place_vendor('STS3250', 'ST3215', step_to_L(), servo_env, chain(femur_servo_m(g), coxa_m), BLACK)
        femur_m = chain(frame((g.Lc, 0, 0), (1, 0, 0), (0, 0, 1)), coxa_m)
        fe = place('Femur', lambda: part_femur(g), femur_m, ORANGE)
        ts = place_vendor('ST3215', 'ST3215', step_to_L(), servo_env, chain(tibia_servo_m(g), femur_m), BLACK)
        tibia_m = chain(frame((g.Lf, 0, 0), (1, 0, 0), (0, 0, 1)), femur_m)
        ti = place('Tibia', lambda: part_tibia(g), tibia_m, ORANGE)
        bo = place('FootBoot_TPU', lambda: part_foot_boot(g), tibia_m, BLACK)
        fsw = place('FootSwitch_D2F', d2f_envelope, chain(d2f_frame(g), tibia_m), BLACK)
        sd = B.coxa_cable_side[i]
        tag = '' if sd > 0 else ' (mirror side)'
        LH = H.leg(sd)
        h_body, h_coxa, h_femur, h_tibia = [], [], [], []
        hx_, hy_, hz_ = Harness.COXA_HUB
        h_coxa.append(place('CoxaBusHub_3x5264', coxa_hub_env, chain(frame((hx_, hy_, hz_), (1, 0, 0), (0, 0, 1)), coxa_m), ('Plastic - Matte (Green)',)))
        for grp, m_, lst in (('body', coxa_m, h_body), ('coxa', coxa_m, h_coxa), ('femur', femur_m, h_femur), ('tibia', tibia_m, h_tibia)):
            for (n_, r_, p_) in LH[grp]:
                lst.append(place('Harness ' + n_ + tag, lambda p_=p_, r_=r_: pipe(p_, r_), m_, WIRE))
            for (n_, t_) in LH['plugs_' + grp]:
                lst.append(place(n_ + tag, lambda t_=t_: t_, m_, PLUG))
        body_occs += h_body
        if not lines:
            lines['Coxa'] = axis_line(cache['CoxaMain'], (0, 0, -40), (0, 0, 55))
            lines['Femur'] = axis_line(cache['Femur'], (0, -35, 0), (0, 35, 0))
            lines['Tibia'] = axis_line(cache['Tibia'], (0, -35, 0), (0, 35, 0))
        body_occs.append(cs)
        if i == 0:
            for grp in ('body', 'coxa', 'femur', 'tibia'):
                for n_, r_, p_ in LH[grp]:
                    harness_report.append(dict(cable=n_ + ' (x6)', d_mm=2 * r_, length_mm=round(sum(math.dist(a, b) for a, b in zip(p_[:-1], p_[1:])), 0)))
        leg_groups.append((i, [cx, cc, fs] + h_coxa, [fe, ts] + h_femur, [ti, bo, fsw] + h_tibia))

    plates['Plate_Top_Rear'].isGrounded = True
    rigid(root, body_occs, 'Body group')
    for (i, gc, gf, gt) in leg_groups:
        rigid(root, gc, 'Leg%d coxa group' % i)
        rigid(root, gf, 'Leg%d femur group' % i)
        rigid(root, gt, 'Leg%d tibia group' % i)
        lg = B.legs[i]
        fixed = plates['Plate_Top_Front'] if lg['Y'] > B.seam_top else plates['Plate_Top_Rear']
        revolute(root, gc[0], fixed, lines['Coxa'], gc[0], *JOINT_LIMITS['coxa'], 'Leg%d coxa' % i)
        revolute(root, gf[0], gc[0], lines['Femur'], gf[0], *JOINT_LIMITS['femur'], 'Leg%d femur' % i)
        revolute(root, gt[0], gf[0], lines['Tibia'], gt[0], *JOINT_LIMITS['tibia'], 'Leg%d tibia' % i)

    # mass estimate: printed volume x 1.24 g/cm3 x 0.6 effective fill (PLA/PETG), TPU 1.21 x 0.8
    printed_g = 0.0
    brain_printed_g = 0.0
    for row in log:
        q = counts.get(row['part'], 1)
        if row['part'] == 'QR_AdapterPlate_Example':
            continue
        if row['part'].startswith('Brain'):
            row['qty'] = q
            row['est_g_each'] = round(row['volume_cm3'] * 1.24 * 0.6, 1)
            brain_printed_g += row['est_g_each']
            continue
        rho, fill = (1.21, 0.8) if 'TPU' in row['part'] else (1.24, 0.6)
        row['qty'] = q
        row['est_g_each'] = round(row['volume_cm3'] * rho * fill, 1)
        printed_g += row['est_g_each'] * q
    bought_g = (12 * MASS_G['ST3215'] + 6 * MASS_G['STS3250'] + MASS_G['Battery_3S_4000'] + MASS_G['ServoDriverESP32']
                + MASS_G['PololuD36V28'] + MASS_G['PololuBigMOSFET'] + MASS_G['PololuACS724'] + MASS_G['BusHub']
                + MASS_G['HC-SR04'] + MASS_G['FuseHolder_ATC'] + MASS_G['RockerSwitch_KCD1'] + MASS_G['XT60_socket']
                + MASS_G['BalanceSocket'] + MASS_G['Feather_ESP32-S3'] + MASS_G['BNO085'] + MASS_G['VL53L1X']
                + MASS_G['ELRS_RP1'] + 6 * MASS_G['D2F_switch'])
    hardware_g = 120.0   # M3/M2 screws, ~170 inserts, washers, TPU grommets
    wiring_g = 125.0     # 18 bus-servo leads, 14 AWG power, XT60s, balance lead, 6 foot leads, sensor leads
    total_g = printed_g + bought_g + hardware_g + wiring_g
    brain_g = (brain_printed_g + MASS_G['RaspberryPi5_8GB'] + MASS_G['Pi5_ActiveCooler'] + MASS_G['OAK-D-Lite'] + MASS_G['LD19']
               + MASS_G['D36V50F5'] + MASS_G['microSD'] + 25.0)   # + cables and screws
    report = dict(parts=log, harness=harness_report, joints=root.asBuiltJoints.count, occurrences=root.allOccurrences.count,
                  vendor_models=vendor_used, use_vendor=use_vendor,
                  mass_g=dict(printed=round(printed_g), bought=round(bought_g), hardware=hardware_g, wiring=wiring_g, total=round(total_g),
                              brain_module=round(brain_g), brain_module_printed=round(brain_printed_g)),
                  frames=dict(top_plate_top=Y_PP[1], top_plate_bottom=Y_PP[0], bottom_plate_top=Y_MP[1], bottom_plate_bottom=Y_MP[0],
                              arm_planes=dict(plus=Y_PA, minus=Y_MA), leg_centre_plane=Y_LEG,
                              coxa_lowest=-38.5, attachment_datum=Y_PP[1] + QR['base_h'] + 5.0))
    if out_dir:
        with open(os.path.join(out_dir, 'build_report.json'), 'w') as f:
            json.dump(report, f, indent=1)
    return report, printed


def export_stls(printed, stl_dir):
    em = adsk.fusion.Design.cast(app.activeProduct).exportManager
    out = []
    for nm, (comp, bed) in printed.items():
        body = comp.bRepBodies.item(0)
        fn = os.path.join(stl_dir, nm + '.stl')
        opt = em.createSTLExportOptions(body, fn)
        opt.meshRefinement = adsk.fusion.MeshRefinementSettings.MeshRefinementHigh
        opt.isBinaryFormat = True
        em.execute(opt)
        out.append(dict(part=nm, file=fn, bed=list(bed)))
    with open(os.path.join(stl_dir, 'export_manifest.json'), 'w') as f:
        json.dump([{'part': o['part'], 'bed': o['bed']} for o in out], f, indent=1)
    return out


MESH_GROUPS = {'Body group': 'body', 'Leg0 coxa group': 'coxa', 'Leg0 femur group': 'femur', 'Leg0 tibia group': 'tibia'}
MESH_SPLIT = {'QR_Receiver': 'qr_receiver', 'QR_AdapterPlate_Example': 'qr_adapter_example', 'STS3032': 'qr_adapter_example',
              'HC-SR04': 'sonar_example',
              'BrainBase': 'brain_module', 'BrainLid': 'brain_module', 'BrainCamMount': 'brain_module', 'RaspberryPi5': 'brain_module',
              'OAK-D-Lite': 'brain_module', 'PololuD36V50F5': 'brain_module', 'LD19_LiDAR': 'brain_module',
              'Pi5_ActiveCooler': 'brain_module'}


def export_mesh_raw(out):
    """Per-body STLs (component coordinates) + world transforms for
    cad/export_sim_meshes.py.  Leg 0 only; every joint must be at 0."""
    design = adsk.fusion.Design.cast(app.activeProduct)
    root = design.rootComponent
    em = design.exportManager
    os.makedirs(out, exist_ok=True)
    for f in os.listdir(out):
        if f.endswith('.stl'):
            os.remove(os.path.join(out, f))
    occ_xf, ref = {}, {}
    k = 0

    def walk(o, parent_m, group):
        nonlocal k
        m = o.transform.copy()
        if parent_m is not None:
            m.transformBy(parent_m)
        for b in o.component.bRepBodies:
            fn = '%s__%s__%d.stl' % (group, o.component.name.replace(' ', '_').replace('/', '_')[:40], k)
            k += 1
            opt = em.createSTLExportOptions(b, os.path.join(out, fn))
            opt.meshRefinement = adsk.fusion.MeshRefinementSettings.MeshRefinementLow
            opt.isBinaryFormat = True
            em.execute(opt)
            occ_xf[fn] = [m.getCell(r, c) for r in range(4) for c in range(4)]
        for c in o.component.occurrences:
            walk(c, m, group)

    for rg in root.rigidGroups:
        g = MESH_GROUPS.get(rg.name)
        if g is None:
            continue
        for o in rg.occurrences:
            name = o.component.name
            if name.startswith('Harness') or name.startswith('Plug'):
                continue
            gg = MESH_SPLIT.get(name, g) if g == 'body' else g
            if name in ('CoxaMain', 'Femur', 'Tibia') and g in ('coxa', 'femur', 'tibia'):
                ref[g] = [o.transform.getCell(r, c) for r in range(4) for c in range(4)]
            walk(o, None, gg)
    with open(os.path.join(out, 'refs.json'), 'w') as f:
        json.dump({'ref': ref, 'occ': occ_xf}, f)
    return len(occ_xf)


# ---------------------------------------------------------------------------
# notebook / doc snapshots
# ---------------------------------------------------------------------------
SNAPSHOTS = {
    # name: (eye, target, fit)    world mm, Fusion frame (X right, Y forward, Z up)
    'hexapod_mk1_iso':         ((420, -520, 380), (0, 0, -20), True),
    'hexapod_mk1_front':       ((0, 700, 0), (0, 0, -20), True),
    'hexapod_mk1_top':         ((0, 0.01, 800), (0, 0, 0), True),
    'hexapod_mk1_underside':   ((260, -320, -520), (0, 0, -20), True),
    'hexapod_mk1_leg_closeup': ((400, 420, 140), (110, 140, -40), False),
}


def save_snapshots(out_dir, names=None, w=1600, h=1000):
    """Save the doc / notebook renders of the active design to out_dir (PNG).

    Run inside Fusion after build(); then run docs/notebook/build_notebook.py.
    """
    app = adsk.core.Application.get()
    vp = app.activeViewport
    os.makedirs(out_dir, exist_ok=True)
    saved = []
    for name, (eye, tgt, fit) in SNAPSHOTS.items():
        if names and name not in names:
            continue
        cam = vp.camera
        cam.isSmoothTransition = False
        cam.cameraType = adsk.core.CameraTypes.PerspectiveCameraType
        cam.eye = adsk.core.Point3D.create(*(v / 10.0 for v in eye))      # Fusion API is in cm
        cam.target = adsk.core.Point3D.create(*(v / 10.0 for v in tgt))
        cam.upVector = adsk.core.Vector3D.create(0, 0, 1) if name != 'hexapod_mk1_top' else adsk.core.Vector3D.create(0, 1, 0)
        cam.isFitView = fit
        vp.camera = cam
        vp.refresh()
        path = os.path.join(out_dir, name + '.png')
        vp.saveAsImageFile(path, w, h)
        saved.append(path)
    return saved
