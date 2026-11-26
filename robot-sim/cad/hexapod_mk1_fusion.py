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
    return dict(solid=s, tools=tools, bed=(0, 0, -1), fillet_axis=(0, 0, 1), edge_break=True)


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
    return dict(solid=s, tools=tools, bed=(1, 0, 0), fillet_axis=(1, 0, 0), edge_break=True)


def part_foot_boot(g):
    z_tip = -(g.Lt - 3.0 - 8.0)
    yc = g.yc
    outer = U(box(-16, 16, yc - 11, yc + 11, z_tip, z_tip + 12), cyl((-16, yc, z_tip), (16, yc, z_tip), 11.0))
    inner = U(box(-14.2, 14.2, yc - 8.2, yc + 8.2, z_tip, z_tip + 13), cyl((-14.2, yc, z_tip), (14.2, yc, z_tip), 8.2))
    return dict(solid=outer, tools=[inner], bed=(0, 0, 1), fillet_axis=None, chamfer=False)


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
    return dict(solid=s, tools=tools, bed=(0, 1, 0), fillet_axis=(1, 0, 0), fillet_r=3.0,
                fillet_skip=lambda p: p[2] < zt + 1.0, edge_break=True)


def dovetail(z_bot, z_top, hw_bot, hw_top, y0, y1):
    s = box(-hw_bot, hw_bot, y0, y1, z_bot, z_top)
    for sx in (1, -1):
        p0 = (sx * hw_bot, 0, z_bot)
        dz = z_top - z_bot
        D(s, halfspace(p0, (sx * dz, 0, (hw_bot - hw_top))))
    return s


QR = dict(base_h=8.0, floor=2.0, hw_bot=14.0, hw_top=10.0, key_x=(4.0, 8.0), key_h=2.0,
          y_rear=-44.0, y_front=20.0, stop=4.0, lock=(24.0, -36.0))


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
    # J1 attachment port: JST-XH 6-pin (GND, GND, VBAT, VBAT, +5V, BUS DATA), through to the pass-through
    tools.append(box(-8.9, 8.9, yr - 10.2, yr - 4.0, z0 - 1, zt + 1))
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
        cables.append(('Sonar', R_SIG, [(0, yf - 5.0, zc), (0, yf - 9, zc - 4), (0, 123, -2.5), (-8, 123, -5.0)]))
        return cables, plugs

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
        out['plugs_body'] = [('Plug coxa servo', plug_c)]
        out['plugs_coxa'] = [('Plug femur', plug_f)]
        out['plugs_femur'] = [('Plug tibia', plug_to)]
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
}


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
    return 0


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
                   ('QR_Receiver', lambda: part_qr_receiver(B))):
        body_occs.append(place(nm, fn, I3(), ORANGE))
    body_occs.append(place('QR_AdapterPlate_Example', lambda: part_qr_adapter(B), I3(), BLUE))
    body_occs.append(place('STS3032', sts3032_envelope,
                           frame((0, -10, g.z_top + QR['base_h'] + 5 + STS3032['H'] - 4), (1, 0, 0), (0, 0, 1)), BLUE))

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
        sd = B.coxa_cable_side[i]
        tag = '' if sd > 0 else ' (mirror side)'
        LH = H.leg(sd)
        h_body, h_coxa, h_femur = [], [], []
        hx_, hy_, hz_ = Harness.COXA_HUB
        h_coxa.append(place('CoxaBusHub_3x5264', coxa_hub_env, chain(frame((hx_, hy_, hz_), (1, 0, 0), (0, 0, 1)), coxa_m), ('Plastic - Matte (Green)',)))
        for grp, m_, lst in (('body', coxa_m, h_body), ('coxa', coxa_m, h_coxa), ('femur', femur_m, h_femur)):
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
            for grp in ('body', 'coxa', 'femur'):
                for n_, r_, p_ in LH[grp]:
                    harness_report.append(dict(cable=n_ + ' (x6)', d_mm=2 * r_, length_mm=round(sum(math.dist(a, b) for a, b in zip(p_[:-1], p_[1:])), 0)))
        leg_groups.append((i, [cx, cc, fs] + h_coxa, [fe, ts] + h_femur, [ti, bo]))

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
    for row in log:
        q = counts.get(row['part'], 1)
        if row['part'] == 'QR_AdapterPlate_Example':
            continue
        rho, fill = (1.21, 0.8) if 'TPU' in row['part'] else (1.24, 0.6)
        row['qty'] = q
        row['est_g_each'] = round(row['volume_cm3'] * rho * fill, 1)
        printed_g += row['est_g_each'] * q
    bought_g = (12 * MASS_G['ST3215'] + 6 * MASS_G['STS3250'] + MASS_G['Battery_3S_4000'] + MASS_G['ServoDriverESP32']
                + MASS_G['PololuD36V28'] + MASS_G['PololuBigMOSFET'] + MASS_G['PololuACS724'] + MASS_G['BusHub']
                + MASS_G['HC-SR04'] + MASS_G['FuseHolder_ATC'] + MASS_G['RockerSwitch_KCD1'] + MASS_G['XT60_socket']
                + MASS_G['BalanceSocket'])
    hardware_g = 110.0   # M3/M2 screws, 150 inserts, washers
    wiring_g = 90.0      # 18 bus-servo leads, 14 AWG power, XT60s, balance lead
    total_g = printed_g + bought_g + hardware_g + wiring_g
    report = dict(parts=log, harness=harness_report, joints=root.asBuiltJoints.count, occurrences=root.allOccurrences.count,
                  vendor_models=vendor_used, use_vendor=use_vendor,
                  mass_g=dict(printed=round(printed_g), bought=round(bought_g), hardware=hardware_g, wiring=wiring_g, total=round(total_g)),
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
              'HC-SR04': 'sonar_example'}


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
