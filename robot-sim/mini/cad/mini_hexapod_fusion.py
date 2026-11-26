"""Hexapod Mini - parametric generator for Autodesk Fusion.

A remote-control-only mini hexapod for under US$200 (see ../docs/DESIGN.md).
18 x MG90S micro servos, ESP32-DevKitC + 2 x PCA9685, 2S 18650 Li-ion pack with a
2S BMS, a fuse and a rocker switch, two 5 V / 5 A bucks (one per side), and a
Bluetooth gamepad (Bluepad32).

Run it inside Fusion (Utilities > Scripts and Add-Ins, or through the Fusion MCP
server).  build() works in its own document: it activates the open document whose
root component is called "Hexapod Mini" or, if there is none, creates a new design
document.  It never touches the "Hexapod Mk1" documents.

It follows the Mk1 generator (robot-sim/cad/hexapod_mk1_fusion.py):
  * every solid is made from TemporaryBRep primitives (boxes, cylinders, spheres),
  * one component per unique part, user parameters for the main dimensions,
  * as-built revolute joints at every coxa, femur and tibia axis, with limits,
    and one rigid group per moving link,
  * printed parts get concave-edge fillets, a bed-side chamfer (elephant foot)
    and edge breaks, then the holes are cut (teardrops when a hole is horizontal
    in the print),
  * every wire is modelled ("Harness ..." components), with plugs.

Frames
------
World (Fusion): +X = robot RIGHT, +Y = FORWARD, +Z = UP, origin in the body centre
at the height of the femur axes ("hip plane").  Simulator: x_sim = -X, y_sim = Z,
z_sim = Y, the same as Mk1.
Leg frame L (one per leg): origin on the coxa axis in the hip plane, +X out along
the leg (coxa at 0), +Z up, +Y = left of the leg (Z x X).  This is the sim's leg
frame turned to Fusion axes (sim: +X out, +Y up, +Z forward).
Femur frame F: origin on the femur axis, +X along the femur.  Tibia frame T: origin
on the tibia axis, +X along the femur at the modelled pose, the tibia points -Z.
Servo frame S (MG90S): origin on the output axis on the case bottom face, +Z along
the shaft to the horn, +X along the case towards the cable end.

Modelled pose: coxa 0, femur horizontal (sim femur 0), tibia straight down (sim
tibia -90, servo 90).  Fusion joint signs: coxa + = counter-clockwise from above,
femur + = DOWN (= -sim femur), tibia + = fold in (= -(sim tibia + 90)).

All dimensions are millimetres; the helpers convert to Fusion's centimetres.
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
DOC_NAME = 'Hexapod Mini'
FAST = False      # True: skip fillets / chamfers / edge breaks (quick geometry checks)


def cm(v):
    return v / 10.0


def P(x, y, z):
    return adsk.core.Point3D.create(cm(x), cm(y), cm(z))


def V(x, y, z):
    v = adsk.core.Vector3D.create(x, y, z)
    v.normalize()
    return v


# ----------------------------------------------------------------------------
# Parameters (names follow robot-sim/mini/firmware/mini_hexapod/config.h)
# ----------------------------------------------------------------------------
PARAMS = [
    ('leg_coxa', '30 mm', 'mm', 'coxa axis to femur axis (firmware LEG_COXA)'),
    ('leg_femur', '44 mm', 'mm', 'femur axis to tibia axis (LEG_FEMUR)'),
    ('leg_tibia', '60 mm', 'mm', 'tibia axis to the foot tip (LEG_TIBIA)'),
    ('standHeight', '45 mm', 'mm', 'hip plane above the floor when walking (STAND_HEIGHT)'),
    ('footReach', '58 mm', 'mm', 'coxa axis to foot when standing (FOOT_REACH); stance window 52-61 mm (middle legs), 52-70 mm (corners)'),
    ('fit_clearance', '0.25 mm', 'mm', 'clearance per side, servo pockets'),
    ('first_layer_chamfer', '0.4 mm', 'mm', 'elephant-foot chamfer on bed-side edges'),
    ('root_fillet', '1.5 mm', 'mm', 'fillet on inside corners along the print-vertical axis'),
]
# legs FR, MR, RR, FL, ML, RL (sim order 0..5): coxa axis (X, Y) and leg direction (deg from +X)
MOUNTS = [(48.0, 72.0, 40.0), (60.0, 0.0, 0.0), (48.0, -72.0, -40.0),
          (-48.0, 72.0, 140.0), (-60.0, 0.0, 180.0), (-48.0, -72.0, -140.0)]
for i, (x, y, d) in enumerate(MOUNTS):
    PARAMS.append(('mount_%d_x' % i, '%g mm' % x, 'mm', 'coxa axis %d, robot right' % i))
    PARAMS.append(('mount_%d_y' % i, '%g mm' % y, 'mm', 'coxa axis %d, forward' % i))
    PARAMS.append(('mount_%d_dir' % i, '%g deg' % d, 'deg', 'leg %d direction from +X' % i))


def ensure_params(design):
    up = design.userParameters
    for name, expr, units, comment in PARAMS:
        if up.itemByName(name) is None:
            up.add(name, adsk.core.ValueInput.createByString(expr), units, comment)


def pmm(design, name):
    return design.userParameters.itemByName(name).value * 10.0


def pdeg(design, name):
    return math.degrees(design.userParameters.itemByName(name).value)


# ----------------------------------------------------------------------------
# Temporary-BRep primitives (mm) - same helpers as Mk1
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
    a = _vec([p1[k] - p0[k] for k in range(3)])
    u = _vec(up)
    w = _cross(a, u)
    L = math.sqrt(sum((p1[k] - p0[k]) ** 2 for k in range(3)))
    mid = [(p0[k] + p1[k]) / 2 + u[k] * r / math.sqrt(2) for k in range(3)]
    d = _vec([u[k] + w[k] for k in range(3)])
    roof = obox(mid, a, d, L, r, r)
    return U(cyl(p0, p1, r), roof)


def hole(p0, p1, d, up=None):
    """Round hole, or a teardrop when the hole is horizontal in the print ('up' = print up)."""
    if up is None:
        return cyl(p0, p1, d / 2)
    a = _vec([p1[k] - p0[k] for k in range(3)])
    if abs(sum(a[k] * up[k] for k in range(3))) > 0.9:
        return cyl(p0, p1, d / 2)
    return teardrop(p0, p1, d / 2, up)


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


def xf(body, m):
    TB.transform(body, m)
    return body


def apply_m(m, p):
    q = P(*p)
    q.transformBy(m)
    return (q.x * 10.0, q.y * 10.0, q.z * 10.0)


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


def prism_poly_convex(pts, z0, z1):
    """Convex polygon (CCW list of (x, y)) extruded from z0 to z1, as halfspace intersections."""
    b = box(-500, 500, -500, 500, z0, z1)
    n = len(pts)
    for k in range(n):
        (ax, ay), (bx, by) = pts[k], pts[(k + 1) % n]
        nx, ny = (by - ay), -(bx - ax)          # outward normal for a CCW polygon
        D(b, halfspace((ax, ay, 0), (nx, ny, 0)))
    return b


def minkowski_prism(pts, r, z0, z1):
    """Convex polygon grown by r (convex hull of circles of radius r on the vertices)."""
    b = prism_poly_convex(pts, z0, z1)
    n = len(pts)
    for k in range(n):
        (ax, ay), (bx, by) = pts[k], pts[(k + 1) % n]
        U(b, cyl((ax, ay, z0), (ax, ay, z1), r))
        L = math.hypot(bx - ax, by - ay)
        d = ((bx - ax) / L, (by - ay) / L, 0)
        nrm = (d[1], -d[0], 0)
        c = ((ax + bx) / 2 + nrm[0] * r / 2, (ay + by) / 2 + nrm[1] * r / 2, (z0 + z1) / 2)
        U(b, obox(c, d, nrm, L, r, z1 - z0))
    return b


# ----------------------------------------------------------------------------
# MG90S micro servo (typical datasheet dimensions; confirm with calipers and the
# fit coupon, BUILD.md step 1).  Servo frame S, see the module docstring.
# ----------------------------------------------------------------------------
SV = dict(front=-5.9, back=16.9, half_w=6.1, case_top=22.7, tab_z=(15.7, 18.2), tab_x=(-10.7, 21.7),
          tab_holes=(-8.4, 19.4), boss_r=5.9, boss_top=26.7, horn_hub=(26.7, 29.0), horn=(29.0, 31.0),
          horn_len=12.5, horn_w=2.3, cable_z=3.0)
MG90S_G = 13.4
FIT = 0.25


def mg90s_envelope():
    b = box(SV['front'], SV['back'], -SV['half_w'], SV['half_w'], 0, SV['case_top'])
    tabs = box(SV['tab_x'][0], SV['tab_x'][1], -SV['half_w'], SV['half_w'], *SV['tab_z'])
    for hx in SV['tab_holes']:
        D(tabs, cyl((hx, 0, SV['tab_z'][0] - 1), (hx, 0, SV['tab_z'][1] + 1), 1.0))
    U(b, tabs, cyl((0, 0, SV['case_top']), (0, 0, SV['boss_top']), SV['boss_r']),
      cyl((6.0, 0, SV['case_top']), (6.0, 0, SV['case_top'] + 1.8), 2.8))
    return b


def horn_envelope():
    """Single-arm horn, arm along +X of the horn frame (= servo frame turned so the arm
    points along the driven link)."""
    b = cyl((0, 0, SV['horn_hub'][0]), (0, 0, SV['horn_hub'][1]), 3.5)
    U(b, cyl((0, 0, SV['horn'][0]), (0, 0, SV['horn'][1]), 3.5),
      box(0, SV['horn_len'], -SV['horn_w'], SV['horn_w'], *SV['horn']),
      cyl((SV['horn_len'], 0, SV['horn'][0]), (SV['horn_len'], 0, SV['horn'][1]), SV['horn_w']))
    return b


# ----------------------------------------------------------------------------
# Geometry constants (mm).  Stack along the femur/tibia axes (leg +Y):
#   idler arm [-20,-17] | 0.5 | cradle plate [-16.5,-14] | servo bottom -14 ... horn top 17 | horn arm [17,20]
# ----------------------------------------------------------------------------
Y_SB = -14.0                 # servo bottom face (femur and tibia servos)
Y_CR = (-16.5, -14.0)        # proximal part's cradle plate (carries the idler pin hole)
Y_ID = (-20.0, -17.0)        # distal part's idler arm
Y_HA = (17.0, 20.0)          # distal part's horn arm
Y_TP = (Y_SB + SV['tab_z'][0] - 3.0, Y_SB + SV['tab_z'][0])   # tab plate under the tabs: [-1.3, 1.7]
Z_FLOOR = -19.0              # coxa servo bottom = tray floor top
Z_DECK = (Z_FLOOR + SV['tab_z'][1], Z_FLOOR + SV['tab_z'][1] + 2.5)   # deck sits on the coxa servo tabs: [-0.8, 1.7]
Z_UA = (Z_FLOOR + SV['horn'][1], Z_FLOOR + SV['horn'][1] + 3.0)     # coxa upper arm on the coxa horn: [12, 15]
Z_LA = (-25.0, -22.0)        # coxa lower (pivot) arm under the floor
FLOOR_T = 2.5
Z_BELLY = -24.5              # belly (battery bay) floor top = hatch top
HATCH_T = 2.5
TRAY_R = 14.0                # tray edge radius round each coxa axis (servo tab corner is at 12.3)
SWEEP_R = 14.5               # coxa link web sweeps outside this radius (half plane beyond the axis)
CUT_R = 45.0                 # ... out to this radius (the coxa link reaches 41 mm)
WEB_X = (15.5, 18.5)         # coxa link web (leg frame X)
BULK_X = (17.0, 20.0)        # femur bulkhead (femur frame X)
FEM_H = 9.0                  # femur half-height (z_f) - the femur prints on its z_f = -9 face
TIB_W = 8.0                  # tibia arm half-width (x_t)
TIB_WEB = (-31.0, -26.0)     # tibia web (z_t)
R_SERVO, R_PWR, R_SIG = 1.0, 0.9, 0.65   # servo lead (3-core, ~2 mm), 20 AWG silicone, 26 AWG Dupont


class Geo:
    def __init__(self, design):
        self.coxa = pmm(design, 'leg_coxa')
        self.femur = pmm(design, 'leg_femur')
        self.tibia = pmm(design, 'leg_tibia')
        self.fil = pmm(design, 'root_fillet')
        self.cham = pmm(design, 'first_layer_chamfer')
        self.mounts = [(pmm(design, 'mount_%d_x' % i), pmm(design, 'mount_%d_y' % i), pdeg(design, 'mount_%d_dir' % i)) for i in range(6)]

    def leg_m(self, i):
        x, y, d = self.mounts[i]
        a = math.radians(d)
        return frame((x, y, 0), (math.cos(a), math.sin(a), 0), (0, 0, 1))

    def fem_local(self):
        return frame((self.coxa, 0, 0), (1, 0, 0), (0, 0, 1))

    def tib_local(self):
        return frame((self.femur, 0, 0), (1, 0, 0), (0, 0, 1))


# servo placements (in the parent link's frame)
def coxa_servo_m():          # leg frame: shaft up, cable end inward
    return frame((0, 0, Z_FLOOR), (-1, 0, 0), (0, 0, 1))


def femur_servo_m(g):        # leg frame: shaft along +Y, cable end down
    return frame((g.coxa, Y_SB, 0), (0, 0, -1), (0, 1, 0))


def tibia_servo_m(g):        # femur frame: shaft along +Y, cable end towards the femur axis
    return frame((g.femur, Y_SB, 0), (-1, 0, 0), (0, 1, 0))


def coxa_horn_m():           # coxa frame (= leg frame): arm along +X
    return frame((0, 0, Z_FLOOR), (1, 0, 0), (0, 0, 1))


def femur_horn_m():          # femur frame: arm along +X_F
    return frame((0, Y_SB, 0), (1, 0, 0), (0, 1, 0))


def tibia_horn_m():          # tibia frame: arm along -Z_T (down the tibia)
    return frame((0, Y_SB, 0), (0, 0, -1), (0, 1, 0))


def servo_point(m, p):
    return apply_m(m, p)


def near_servo(m, margin=2.0):
    """Fillet keep-out: True for points within `margin` of the MG90S (case, tabs, boss,
    horn) placed with matrix m (part frame).  Fillets there would touch the servo."""
    inv = m.copy()
    inv.invert()

    def f(p):
        x, y, z = apply_m(inv, p)
        if SV['tab_x'][0] - margin <= x <= SV['tab_x'][1] + margin and abs(y) <= SV['half_w'] + margin and -margin <= z <= SV['tab_z'][1] + margin:
            return True
        if math.hypot(x, y) <= 3.5 + margin and SV['case_top'] - margin <= z <= SV['horn'][1] + margin:
            return True
        return SV['front'] - margin <= x <= SV['horn_len'] + 2.5 + margin and abs(y) <= SV['half_w'] + margin and SV['horn'][0] - margin <= z <= SV['horn'][1] + margin
    return f


def any_of(*fs):
    return lambda p: any(f(p) for f in fs)


# ----------------------------------------------------------------------------
# Printed leg parts
# ----------------------------------------------------------------------------
def part_coxa(g):
    """Coxa link (leg frame).  A C-shape round the tray edge: upper arm on the coxa horn,
    web outside the tray, lower arm with the pivot pin under the floor, and a block that
    holds the femur servo (case pocket, tab seat, cradle plate with the femur idler pin hole).
    Prints on its -Y face (y = -16.5)."""
    y0 = Y_CR[0]
    fx = g.coxa
    s = box(-9.0, 32.0, y0, 12.5, *Z_UA)                                  # upper arm
    U(s, box(WEB_X[0], WEB_X[1], y0, 12.5, Z_LA[0], Z_UA[1]))           # web
    U(s, box(-7.0, WEB_X[1], y0, 8.0, *Z_LA))                            # lower arm
    U(s, box(WEB_X[1], fx + 8.0, y0, Y_TP[1], -24.0, 11.0))              # femur servo block
    U(s, box(WEB_X[1], fx + 2.0, y0, Y_TP[1], 11.0, Z_UA[0]))            # block -> upper arm filler
    up = (0, 1, 0)
    sm = femur_servo_m(g)
    cav = box(SV['front'] - FIT, SV['back'] + FIT, -SV['half_w'] - FIT, SV['half_w'] + FIT, 0, 30)   # case pocket (servo frame)
    xf(cav, sm)
    tools = [cav,
             box(fx - 1.6, fx + 1.6, -17.0, -9.4, -21.2, -16.0),          # femur servo lead channel
             box(WEB_X[1] + 1.0, fx - SV['half_w'] - FIT - 1.0, -13.5, Y_TP[1] + 0.1, -16.0, 4.0),   # lightening pocket
             hole((fx, Y_CR[0] - 1, 0), (fx, Y_CR[1] + 0.01, 0), 3.4, up),  # femur idler pin (M3 spins here)
             hole((0, 0, Z_LA[0] - 1), (0, 0, Z_LA[1] + 1), 2.8, up),     # coxa pivot pin (M3 self-taps here)
             hole((0, 0, Z_UA[0] - 0.1), (0, 0, Z_UA[1] + 0.1), 5.5, up)]  # coxa horn screw access
    for hx in (6.0, 9.0, 12.0):                                            # coxa horn screw holes
        tools.append(hole((hx, 0, Z_UA[0] - 0.1), (hx, 0, Z_UA[1] + 0.1), 1.5, up))
    for tx in SV['tab_holes']:                                             # femur servo tab screws (M2 self-tap)
        p = servo_point(sm, (tx, 0, SV['tab_z'][0]))
        tools.append(cyl((p[0], p[1] - 6.0, p[2]), (p[0], p[1] + 0.1, p[2]), 0.8))
    return dict(solid=s, tools=tools, bed=(0, -1, 0), fillet_axis=(0, 1, 0), edge_break=True,
                fillet_skip=any_of(near_servo(sm), near_servo(coxa_horn_m())))


def _d_arm(y0, y1, r, xlen, h):
    """Arm plate in the x-z plane from y0 to y1: disc r on the axis plus a box out to xlen."""
    a = cyl((0, y0, 0), (0, y1, 0), r)
    U(a, box(0, xlen, y0, y1, -h, h))
    return a


def part_femur(g):
    """Femur (femur frame).  Horn arm and idler arm round the femur servo, a bulkhead,
    and a cradle for the tibia servo (rails, tab plate, cradle plate with the tibia idler
    pin hole, and a skin on the -Y side).  Prints on its z_f = -9 face."""
    L = g.femur
    h = FEM_H
    xe = L + 12.5
    s = _d_arm(Y_HA[0], Y_HA[1], h, BULK_X[1], h)                         # horn arm
    U(s, _d_arm(Y_ID[0], Y_ID[1], h, BULK_X[1], h))                       # idler arm
    U(s, box(BULK_X[0], BULK_X[1], Y_ID[0], Y_HA[1], -h, h))              # bulkhead
    U(s, box(BULK_X[0], xe, Y_CR[0], Y_CR[1], -h, h))                     # cradle plate
    U(s, box(BULK_X[0], xe, Y_CR[0], Y_TP[1], 6.6, h))                    # upper rail
    U(s, box(BULK_X[0], xe, Y_CR[0], Y_TP[1], -h, -6.6))                  # lower rail
    U(s, box(BULK_X[0], xe, Y_TP[0], Y_TP[1], -h, h))                     # tab plate
    skin = box(BULK_X[0], L - 11.5, Y_ID[0], Y_CR[0], 1.5, h)             # -Y skin: upper half, clear of the tibia idler arm
    U(skin, box(BULK_X[0], 24.0, Y_ID[0], Y_CR[0], -h, h))                # ... and the root of the lower half
    U(s, skin)
    up = (0, 0, 1)
    tm = tibia_servo_m(g)
    cut = box(SV['front'] - 0.3, SV['back'] + 0.3, -SV['half_w'] - 0.3, SV['half_w'] + 0.3, SV['tab_z'][0] - 4.0, SV['tab_z'][0] + 0.1)
    xf(cut, tm)
    tools = [cut,
             box(22.0, 26.0, Y_ID[0] - 0.1, Y_SB + 0.1, -2.2, 2.2),       # tibia lead slot through skin + cradle
             hole((0, Y_ID[0] - 0.1, 0), (0, Y_ID[1] + 0.1, 0), 2.8, up),  # femur idler pin (self-taps here)
             hole((L, Y_CR[0] - 0.1, 0), (L, Y_CR[1] + 0.1, 0), 3.4, up),  # tibia idler pin (spins here)
             hole((0, Y_HA[0] - 0.1, 0), (0, Y_HA[1] + 0.1, 0), 5.5, up)]  # femur horn screw access
    for hx in (6.0, 9.0, 12.0):
        tools.append(hole((hx, Y_HA[0] - 0.1, 0), (hx, Y_HA[1] + 0.1, 0), 1.5, up))
    for tx in SV['tab_holes']:
        p = servo_point(tm, (tx, 0, SV['tab_z'][0]))
        tools.append(cyl((p[0], p[1] - 6.0, p[2]), (p[0], p[1] + 0.1, p[2]), 0.8))
    return dict(solid=s, tools=tools, bed=(0, 0, -1), fillet_axis=(0, 0, 1), edge_break=True,
                fillet_skip=any_of(near_servo(tm), near_servo(femur_horn_m())))


def _t_arm(y0, y1):
    a = cyl((0, y0, 0), (0, y1, 0), TIB_W)
    U(a, box(-TIB_W, TIB_W, y0, y1, TIB_WEB[0], 0))
    return a


def part_tibia(g):
    """Tibia (tibia frame).  Horn arm and idler arm round the tibia servo, a web below the
    servo's sweep, and a tapered leg to a round foot.  Prints on its x_t = -8 face."""
    T = g.tibia
    s = _t_arm(*Y_HA)
    U(s, _t_arm(*Y_ID))
    U(s, box(-TIB_W, TIB_W, Y_ID[0], Y_HA[1], *TIB_WEB))
    U(s, box(-TIB_W, 2.0, -5.0, 5.0, -(T - 4.5), TIB_WEB[0] + 0.01))
    U(s, sphere((0, 0, -(T - 4.5)), 4.5))
    up = (1, 0, 0)
    tools = [hole((0, Y_ID[0] - 0.1, 0), (0, Y_ID[1] + 0.1, 0), 2.8, up),
             hole((0, Y_HA[0] - 0.1, 0), (0, Y_HA[1] + 0.1, 0), 5.5, up)]
    for hz in (6.0, 9.0, 12.0):
        tools.append(hole((0, Y_HA[0] - 0.1, -hz), (0, Y_HA[1] + 0.1, -hz), 1.5, up))
    return dict(solid=s, tools=tools, bed=(-1, 0, 0), fillet_axis=(1, 0, 0), edge_break=True, fillet_skip=near_servo(tibia_horn_m()))


# ----------------------------------------------------------------------------
# Body: tray, deck, battery hatch
# ----------------------------------------------------------------------------
BELLY = dict(x=30.0, y=58.0, wall=2.0, cham=10.0)
PCA = dict(L=62.2, W=25.4, t=1.6, z0=4.7, xR=4.0, y0=-39.1)      # PCA-R origin (inner long edge, rear end)
ESP = dict(x=13.95, y0=27.0, y1=81.4, z0=7.7, t=1.6)              # flipped ESP32-DevKitC (pins up)
BUCK = dict(x=(4.0, 24.2), y=(-54.5, -39.3), z=(-7.6, -0.8))     # Mini560 (right); left = mirrored placement
BMS = dict(x=22.5, y=(41.0, 57.0), z=(-4.3, -0.8))
HOLDER = dict(x=20.5, y=38.5, z=(Z_BELLY, -4.0))
SWITCH = dict(c=(18.0, -64.5), bezel=(15.0, 21.0, 2.0), body=(13.0, 19.0, 15.5))
FUSE = dict(x=(-25.0, 5.0), y=(-82.0, -70.0), z=(Z_FLOOR, -10.0))
DIVIDER = dict(x=(-28.0, -24.0), y=(36.0, 51.0), z=(Z_DECK[1], Z_DECK[1] + 4.0))
DIODE = dict(x=-20.0, y=(54.5, 64.0), r=2.65)
DECK_BOSS = [(27.5, 50.0), (-27.5, 50.0), (27.5, -50.0), (-27.5, -50.0)]
HATCH_BOSS = [(27.5, 0.0), (-27.5, 0.0)]


def belly_prism(hx, hy, cham, z0, z1):
    pts = [(hx, hy - cham), (hx - cham, hy), (-hx + cham, hy), (-hx, hy - cham),
           (-hx, -hy + cham), (-hx + cham, -hy), (hx - cham, -hy), (hx, -hy + cham)]
    pts = [(x, y) for (x, y) in pts]
    # CCW order
    pts = [pts[0], pts[1], pts[2], pts[3], pts[4], pts[5], pts[6], pts[7]]
    return prism_poly_convex(pts, z0, z1)


class Body:
    def __init__(self, g):
        self.g = g
        self.centres = [(x, y) for (x, y, d) in g.mounts]
        self.poly = sorted(self.centres, key=lambda p: math.atan2(p[1], p[0]))     # CCW round the body centre

    def outline(self, z0, z1, r=TRAY_R):
        """Tray outline: hull of circles r round the coxa axes, minus each coxa link's sweep
        (the half plane beyond the axis, outside SWEEP_R)."""
        b = minkowski_prism(self.poly, r, z0, z1)
        for (x, y, d) in self.g.mounts:
            a = math.radians(d)
            n = (math.cos(a), math.sin(a), 0)
            cut = I(cyl((x, y, z0 - 1), (x, y, z1 + 1), CUT_R), halfspace((x, y, 0), n))
            D(cut, cyl((x, y, z0 - 2), (x, y, z1 + 2), SWEEP_R))
            D(b, cut)
        return b

    def cavity(self, z0, z1):
        b = minkowski_prism(self.poly, TRAY_R - 1.2, z0, z1)
        for (x, y, d) in self.g.mounts:
            a = math.radians(d)
            n = (math.cos(a), math.sin(a), 0)
            cut = I(cyl((x, y, z0 - 1), (x, y, z1 + 1), CUT_R), halfspace((x - 2.0 * n[0], y - 2.0 * n[1], 0), n))
            D(cut, cyl((x, y, z0 - 2), (x, y, z1 + 2), SWEEP_R - 1.7))
            D(b, cut)
        return b

    def coxa_servo_world(self, i):
        return chain(coxa_servo_m(), self.g.leg_m(i))

    def leg_pt(self, i, p):
        return apply_m(self.g.leg_m(i), p)


def part_tray(b):
    """Tray (world frame): floor, perimeter wall up to the deck, servo locating rings, the
    battery belly (open at the bottom, closed by the hatch), deck and hatch bosses."""
    zf0 = Z_FLOOR - FLOOR_T
    s = b.outline(zf0, Z_DECK[0])
    D(s, b.cavity(Z_FLOOR, Z_DECK[0] + 1))
    bx, by, bw, bc = BELLY['x'], BELLY['y'], BELLY['wall'], BELLY['cham']
    U(s, belly_prism(bx + bw, by + bw, bc, Z_BELLY, Z_FLOOR))
    D(s, belly_prism(bx, by, bc - 0.8, Z_BELLY - 1, Z_FLOOR + 0.01))
    for i in range(6):
        m = b.coxa_servo_world(i)
        ring = box(SV['front'] - FIT - 1.5, SV['back'] + FIT + 1.5, -SV['half_w'] - FIT - 1.5, SV['half_w'] + FIT + 1.5, 0, 5.0)
        D(ring, box(SV['front'] - FIT, SV['back'] + FIT, -SV['half_w'] - FIT, SV['half_w'] + FIT, -1, 6))
        D(ring, box(12.0, 30.0, -4.0, 4.0, -1, 6))                         # lead exit
        xf(ring, m)
        U(s, ring)
    for (x, y) in DECK_BOSS:
        U(s, cyl((x, y, Z_BELLY), (x, y, Z_DECK[0]), 3.0))
    for (x, y) in HATCH_BOSS:
        U(s, cyl((x, y, Z_BELLY), (x, y, Z_FLOOR), 3.0))
    tools = [box(x0, x1, y0, y1, zf0 - 0.1, Z_FLOOR + 0.1) for (x0, x1, y0, y1) in FLOOR_WINDOWS]
    for i in range(6):
        x, y = b.centres[i]
        tools.append(cyl((x, y, zf0 - 1), (x, y, Z_FLOOR + 0.1), 1.7))     # coxa pivot pin hole (M3 spins)
    for (x, y) in DECK_BOSS:
        tools.append(cyl((x, y, Z_DECK[0] - 10), (x, y, Z_DECK[0] + 0.1), 1.25))
    for (x, y) in HATCH_BOSS:
        tools.append(cyl((x, y, Z_BELLY - 0.1), (x, y, Z_BELLY + 5), 1.25))
    return dict(solid=s, tools=tools, bed=(0, 0, -1), fillet_axis=(0, 0, 1), fillet2=False,
                fillet_skip=any_of(*[near_servo(b.coxa_servo_world(i)) for i in range(6)]))


def pca_holes(side):
    out = []
    for (lx, ly) in ((3.0, 3.0), (22.4, 3.0), (3.0, 59.2), (22.4, 59.2)):
        out.append(pca_world(side, (lx, ly, 0)))
    return out


def pca_world(side, p):
    """PCA9685 local (x across from the inner long edge, y along from the rear end, z up
    from the board bottom) -> world.  side +1 = right board (0x40), -1 = left (0x41,
    turned 180 deg about Z)."""
    lx, ly, lz = p
    if side > 0:
        return (PCA['xR'] + lx, PCA['y0'] + ly, PCA['z0'] + lz)
    return (-PCA['xR'] - lx, PCA['y0'] + PCA['L'] - ly, PCA['z0'] + lz)


def part_deck(b):
    """Deck (world): sits on the coxa servo tabs and clamps them.  Carries the two PCA9685s
    (standoffs), the flipped ESP32 (pedestal + guides), the fuse is under it, the rocker
    switch through it."""
    z0, z1 = Z_DECK
    s = b.outline(z0, z1)
    for side in (1, -1):
        for (x, y, z) in pca_holes(side):
            U(s, cyl((x, y, z1), (x, y, PCA['z0']), 2.6))
    U(s, box(-9.0, 9.0, ESP['y0'] + 0.5, 44.5, z1, ESP['z0'] - 3.1))      # pedestal under the module
    for sx in (1, -1):
        U(s, box(*sorted((sx * 14.3, sx * 16.3)), 30.0, 78.0, z1, ESP['z0'] + 2.8))   # guides
    tools = []
    for i in range(6):
        m = b.coxa_servo_world(i)
        cut = box(SV['front'] - 0.3, SV['back'] + 0.3, -SV['half_w'] - 0.3, SV['half_w'] + 0.3, SV['tab_z'][1] - 0.1, 40)
        tools.append(xf(cut, m))
        for tx in SV['tab_holes']:
            p = apply_m(m, (tx, 0, 0))
            tools.append(cyl((p[0], p[1], z0 - 0.1), (p[0], p[1], z1 + 0.1), 1.2))
        p = b.leg_pt(i, (-24.0, 0, 0))
        tools.append(cyl((p[0], p[1], z0 - 0.1), (p[0], p[1], z1 + 0.1), 2.3))       # coxa servo lead
    for side in (1, -1):
        for (x, y, z) in pca_holes(side):
            tools.append(cyl((x, y, z1), (x, y, PCA['z0'] + 0.1), 1.0))
    for (x, y) in DECK_BOSS:
        tools.append(cyl((x, y, z0 - 0.1), (x, y, z1 + 0.1), 1.7))
    for (x, y) in WIRE_HOLES:
        tools.append(cyl((x, y, z0 - 0.1), (x, y, z1 + 0.1), 1.5))
    for (x0, x1, y0, y1) in DECK_WINDOWS:
        tools.append(box(x0, x1, y0, y1, z0 - 0.1, z1 + 0.1))
    cx, cy = SWITCH['c']
    bw, bl, bh = SWITCH['body']
    tools.append(box(cx - bw / 2 - 0.2, cx + bw / 2 + 0.2, cy - bl / 2 - 0.2, cy + bl / 2 + 0.2, z0 - 0.1, z1 + 0.1))
    return dict(solid=s, tools=tools, bed=(0, 0, -1), fillet_axis=(0, 0, 1), fillet2=False)


WIRE_HOLES = [(1.8, -10.5), (1.8, -5.5), (-1.8, -10.5), (-1.8, -5.5), (-26.0, 30.0)]
# lightening windows (x0, x1, y0, y1): under each PCA9685, under the ESP32, over the fuse
DECK_WINDOWS = [(8.5, 24.9, -32.6, 16.6), (-24.9, -8.5, -32.6, 16.6), (-11.0, 11.0, 47.0, 75.0), (-22.0, 8.0, -80.0, -44.0),
                (29.0, 40.0, 12.0, 40.0), (29.0, 40.0, -40.0, -12.0), (-40.0, -29.0, 12.0, 40.0), (-40.0, -29.0, -40.0, -12.0)]
FLOOR_WINDOWS = [(36.0, 50.0, 12.0, 40.0), (36.0, 50.0, -40.0, -12.0), (-50.0, -36.0, 12.0, 40.0), (-50.0, -36.0, -40.0, -12.0),
                 (-20.0, 20.0, 62.0, 81.0)]


def part_hatch(b):
    """Battery hatch: closes the belly, carries the 2 x 18650 holder (two M3 screws)."""
    bx, by, bw, bc = BELLY['x'], BELLY['y'], BELLY['wall'], BELLY['cham']
    s = belly_prism(bx + bw, by + bw, bc, Z_BELLY - HATCH_T, Z_BELLY)
    tools = [box(-8, 8, by + bw - 4, by + bw + 1, Z_BELLY - HATCH_T - 0.1, Z_BELLY + 0.1),   # finger notch
             box(-13, 13, -18, 18, Z_BELLY - HATCH_T - 0.1, Z_BELLY + 0.1)]                   # window under the holder
    for (x, y) in HATCH_BOSS:
        tools.append(cyl((x, y, Z_BELLY - HATCH_T - 0.1), (x, y, Z_BELLY + 0.1), 1.7))
    for y in (-25.0, 25.0):
        tools.append(cyl((0, y, Z_BELLY - HATCH_T - 0.1), (0, y, Z_BELLY + 0.1), 1.7))
    return dict(solid=s, tools=tools, bed=(0, 0, -1), fillet_axis=None, fillet2=False)


# ----------------------------------------------------------------------------
# Bought-part envelopes (world frame unless stated)
# ----------------------------------------------------------------------------
def pca_env(side):
    def w(x0, x1, y0, y1, z0, z1):
        a = pca_world(side, (x0, y0, z0))
        c = pca_world(side, (x1, y1, z1))
        return box(min(a[0], c[0]), max(a[0], c[0]), min(a[1], c[1]), max(a[1], c[1]), a[2], c[2])
    b = w(0, PCA['W'], 0, PCA['L'], 0, PCA['t'])
    U(b, w(17.78, 25.4, 10.8, 51.4, PCA['t'], 4.1))                       # servo header base
    U(b, w(9.08, 24.32, 0, 2.54, PCA['t'], 4.1))                          # rear 6-pin header base
    U(b, w(9.08, 24.32, 59.66, 62.2, PCA['t'], 4.1))                      # front 6-pin header base
    U(b, w(0, 7.5, 26.1, 36.1, PCA['t'], 11.6))                           # power terminal block
    U(b, w(10.0, 16.0, 28.0, 38.0, PCA['t'], 3.2))                        # PCA9685 chip
    return b


def pca_channel_y(k):
    return 50.13 - 2.54 * k                                               # local y of channel k


def servo_plug(side, k):
    a = pca_world(side, (17.78, pca_channel_y(k) - 1.25, 4.1))
    c = pca_world(side, (25.4, pca_channel_y(k) + 1.25, 18.1))
    return box(min(a[0], c[0]), max(a[0], c[0]), min(a[1], c[1]), max(a[1], c[1]), a[2], c[2])


def plug_top(side, k):
    return pca_world(side, (21.59, pca_channel_y(k), 18.1))


I2C_PINS = ['GND', 'OE', 'SCL', 'SDA', 'VCC', 'V+']


def i2c_pin(side, end, j):
    """end 'front' or 'rear' (world), pin j -> local point on the header base top."""
    lx = 16.7 + (j - 2.5) * 2.54
    if (end == 'front') == (side > 0):
        ly = 60.93
    else:
        ly = 1.27
    return pca_world(side, (lx, ly, 4.1))


def dupont(p, up=14.0):
    """Female Dupont housing standing on a pin base at p (top face at p z + up)."""
    return box(p[0] - 1.25, p[0] + 1.25, p[1] - 1.25, p[1] + 1.25, p[2], p[2] + up)


def esp_pin(row, i):
    """ESP32-DevKitC pin i (1 = USB end) of row 'A' (x -12.7: 3V3 ... 5V side) or 'B'
    (x +12.7: GND ... CLK side), on the flipped board's header base top."""
    x = -12.7 if row == 'A' else 12.7
    return (x, ESP['y1'] - 2.3 - 2.54 * (i - 1), ESP['z0'] + ESP['t'] + 2.5)


def esp_env():
    z0, t = ESP['z0'], ESP['t']
    b = box(-ESP['x'], ESP['x'], ESP['y0'], ESP['y1'], z0, z0 + t)
    U(b, box(-9.0, 9.0, ESP['y0'], ESP['y0'] + 18.0, z0 - 3.1, z0))        # WROOM module (under: board is flipped)
    U(b, box(-4.0, 4.0, ESP['y1'] - 4.9, ESP['y1'] + 1.0, z0 - 2.6, z0))   # micro-USB socket
    for x in (-12.7, 12.7):
        U(b, box(x - 1.27, x + 1.27, esp_pin('A', 19)[1] - 1.27, esp_pin('A', 1)[1] + 1.27, z0 + t, z0 + t + 2.5))
    return b


def buck_env(side):
    x0, x1 = BUCK['x']
    if side < 0:
        x0, x1 = -x1, -x0
    b = box(x0, x1, BUCK['y'][0], BUCK['y'][1], BUCK['z'][1] - 1.6, BUCK['z'][1])
    U(b, box(x0 + 2.0, x1 - 2.2, BUCK['y'][0] + 4.5, BUCK['y'][1] - 1.7, BUCK['z'][0], BUCK['z'][1] - 1.6))
    return b


def buck_pad(side, name, k=0):
    """Pad points on the buck board's underside (IN pads on the rear edge, OUT pads on the
    front edge).  The left buck is the same module in the same orientation, moved across."""
    off = {'IN+': 2.5, 'IN-': 9.0, 'OUT+': 13.0, 'OUT-': 17.0}[name] + 2.3 * k
    zb = BUCK['z'][1] - 1.6 - 0.2
    if name.startswith('IN'):
        y = BUCK['y'][0] + 1.5
    else:
        y = BUCK['y'][1] - 1.5
    x0 = BUCK['x'][0] if side > 0 else -BUCK['x'][1]
    return (x0 + off, y, zb)


def bms_env():
    return box(-BMS['x'], BMS['x'], BMS['y'][0], BMS['y'][1], *BMS['z'])


def bms_pad(name, k=0):
    x = {'B+': -18.0, 'BM': -6.0, 'P-': 6.0, 'B-': 18.0}[name]
    return (x + (2.0 if k == 2 else 0.0), BMS['y'][0] + 1.0 + 3.0 * (k if k < 2 else 2), BMS['z'][0] - 0.1)


def holder_env():
    b = box(-HOLDER['x'], HOLDER['x'], -HOLDER['y'], HOLDER['y'], *HOLDER['z'])
    return b


def switch_env():
    cx, cy = SWITCH['c']
    w, l, h = SWITCH['bezel']
    bw, bl, bh = SWITCH['body']
    b = box(cx - w / 2, cx + w / 2, cy - l / 2, cy + l / 2, Z_DECK[1], Z_DECK[1] + h)
    U(b, box(cx - 5.0, cx + 5.0, cy - 7.0, cy + 7.0, Z_DECK[1] + h, Z_DECK[1] + h + 4.5))          # rocker
    U(b, box(cx - bw / 2, cx + bw / 2, cy - bl / 2, cy + bl / 2, Z_DECK[0] - bh, Z_DECK[1]))
    return b


def fuse_env():
    return box(FUSE['x'][0], FUSE['x'][1], FUSE['y'][0], FUSE['y'][1], *FUSE['z'])


def divider_env():
    return box(DIVIDER['x'][0], DIVIDER['x'][1], DIVIDER['y'][0], DIVIDER['y'][1], *DIVIDER['z'])


def diode_env():
    return cyl((DIODE['x'], DIODE['y'][0], Z_DECK[1] + DIODE['r']), (DIODE['x'], DIODE['y'][1], Z_DECK[1] + DIODE['r']), DIODE['r'])


MASS_G = {'MG90S': 13.4, 'Horn': 0.6, 'PCA9685': 9.0, 'ESP32_DevKitC': 9.5, 'Mini560': 3.0, 'BMS_2S': 4.0,
          'Holder_2x18650_with_cells': 16.0 + 2 * 46.5, 'FuseHolder_ATC_mini': 6.0, 'RockerSwitch_KCD1': 4.0,
          'Diode_1N5822': 1.0, 'Divider_100k_47k': 0.5}


# ----------------------------------------------------------------------------
# Harness.  Lanes: servo leads leave the PCA plugs straight up, then run at a fixed
# height per joint (coxa 25, femur 27.5, tibia 30 mm) to their leg, so no two leads
# cross at the same height.
# ----------------------------------------------------------------------------
LANE = {'coxa': 25.0, 'femur': 27.5, 'tibia': 30.0}
# PCA channel per (leg, joint); right board 0x40 takes legs 0-2, left board 0x41 legs 3-5.
CHANNEL = {0: (0, 1, 2), 1: (6, 7, 8), 2: (15, 14, 13), 3: (15, 14, 13), 4: (9, 8, 7), 5: (0, 1, 2)}
SPLICE_Y = {'femur': -2.0, 'tibia': 2.0}
UA_TOP = Z_UA[1] + R_SERVO + 0.1


def coxa_seg(joint, g):
    """Servo lead on the coxa link (leg frame), from the splice above the coxa axis."""
    fx = g.coxa
    if joint == 'femur':
        z = LANE['femur']
        return [(0, -2.0, z), (0, -2.0, UA_TOP), (20.0, -2.0, UA_TOP), (20.0, -18.5, UA_TOP), (20.0, -18.5, -19.5),
                (fx, -18.5, -19.5), (fx, -11.0, -19.5), (fx, -11.0, -17.1)]
    z = LANE['tibia']
    return [(0, 2.0, z), (0, 2.0, UA_TOP), (22.5, 2.0, UA_TOP), (22.5, -21.6, UA_TOP), (22.5, -21.6, 6.0), (fx, -21.6, 6.0)]


def femur_seg(g):
    """Tibia servo lead on the femur (femur frame), from the splice at the femur joint."""
    return [(0, -21.6, 6.0), (24.0, -21.6, 6.0), (24.0, -21.6, 0.0), (24.0, -11.0, 0.0), (g.femur - SV['back'] + 0.2, -11.0, 0.0)]


def body_servo_leads(b):
    """Body segments of all 18 servo leads (world): (name, radius, points)."""
    out = []
    for i in range(6):
        side = 1 if i < 3 else -1
        lm = b.g.leg_m(i)
        for j, joint in enumerate(('coxa', 'femur', 'tibia')):
            k = CHANNEL[i][j]
            t = plug_top(side, k)
            z = LANE[joint]
            if joint == 'coxa':
                h = apply_m(lm, (-24.0, 0, 0))
                ex = apply_m(lm, (-(SV['back'] - 0.2), 0, Z_FLOOR + SV['cable_z']))
                ex2 = apply_m(lm, (-24.0, 0, Z_FLOOR + SV['cable_z']))
                pts = [t, (t[0], t[1], z), (h[0], h[1], z), (h[0], h[1], ex2[2]), ex2, ex]
            else:
                sp = apply_m(lm, (0, SPLICE_Y[joint], z))
                pts = [t, (t[0], t[1], z), sp]
            out.append(('Leg%d %s lead' % (i, joint), R_SERVO, pts, (side, k)))
    return out


def power_wires():
    """Every power and signal wire in the body (world): (name, radius, points)."""
    W = []
    zt = Z_DECK[1]
    # battery holder -> BMS
    W.append(('B1 holder + to BMS B+', R_PWR, [(-8, HOLDER['y'] + 0.1, -14), (-8, 42, -14), (-18, 42, -14), bms_pad('B+')]))
    W.append(('B2 holder - to BMS B-', R_PWR, [(8, HOLDER['y'] + 0.1, -14), (8, 42, -14), (18, 42, -14), bms_pad('B-')]))
    W.append(('B3 holder mid-tap to BMS BM', R_PWR, [(0, -HOLDER['y'] - 0.1, -14), (0, -40.5, -14), (22.3, -40.5, -14), (22.3, 46, -14),
                                                    (22.3, 46, -7), (-6, 46, -7), (-6, 42, -7), bms_pad('BM')]))
    # BMS P+ (= B+) -> fuse -> switch -> bucks
    p = bms_pad('B+', 1)
    W.append(('F1 BMS P+ to fuse', R_PWR, [p, (p[0], p[1], -10), (-22.3, p[1], -10), (-22.3, -76, -10), (-27.0, -76, -10),
                                            (-27.0, -76, -14.5), (FUSE['x'][0] + 0.2, -76, -14.5)]))
    W.append(('F2 fuse to switch', R_PWR, [(FUSE['x'][1] - 0.2, -76, -14.5), (8.0, -76, -14.5), (8.0, -76, -17.8), (18.0, -76, -17.8),
                                            (18.0, -68.5, -17.8), (18.0, -68.5, Z_DECK[0] - SWITCH['body'][2] + 0.1)]))
    a = buck_pad(1, 'IN+')
    W.append(('F3 switch to buck R IN+', R_PWR, [(18.0, -60.5, Z_DECK[0] - SWITCH['body'][2] + 0.1), (18.0, -60.5, -17.8), (18.0, -57, -17.8),
                                                 (a[0], -57, -17.8), (a[0], a[1], -17.8), a]))
    a, c = buck_pad(1, 'IN+', 1), buck_pad(-1, 'IN+', 1)
    W.append(('F4 buck R IN+ to buck L IN+', R_PWR, [a, (a[0], a[1], -9.0), (a[0], -51.0, -9.0), (c[0], -51.0, -9.0), (c[0], c[1], -9.0), c]))
    # BMS P- -> buck R IN- -> buck L IN-
    p = bms_pad('P-')
    a = buck_pad(1, 'IN-', 1)
    W.append(('G1 BMS P- to buck R IN-', R_PWR, [p, (p[0], p[1], -6), (p[0], 39.8, -6), (p[0], 39.8, -10), (22.3, 39.8, -10), (22.3, -53.6, -10),
                                                 (a[0], -53.6, -10), (a[0], a[1], -10), a]))
    a, c = buck_pad(1, 'IN-'), buck_pad(-1, 'IN-')
    W.append(('G2 buck R IN- to buck L IN-', R_PWR, [a, (a[0], a[1], -20.0), (c[0], c[1], -20.0), c]))
    # buck outputs -> PCA power terminals (up through the deck between the boards)
    zt_e = PCA['z0'] + 4.3
    for side, tag in ((1, 'R'), (-1, 'L')):
        vp = pca_world(side, (0.2, 28.6, 4.3))
        gn = pca_world(side, (0.2, 33.6, 4.3))
        hx = 1.8 * side
        o_p, o_g = buck_pad(side, 'OUT+'), buck_pad(side, 'OUT-')
        zr = -2.5
        # route the output whose terminal is further back first, at a y that clears the other
        first, second = ((o_p, vp, 'OUT+ to V+'), (o_g, gn, 'OUT- to GND'))
        if vp[1] > gn[1]:
            first, second = second, first
        (pa, ta, na), (pb, tb, nb) = first, second
        W.append(('O%s buck %s %s' % (tag, tag, na), R_PWR, [pa, (pa[0], pa[1], zr), (pa[0], ta[1] - 2.0, zr), (hx, ta[1] - 2.0, zr), (hx, ta[1], zr),
                                                            (hx, ta[1], zt_e), (ta[0], ta[1], zt_e)]))
        W.append(('O%s buck %s %s' % (tag, tag, nb), R_PWR, [pb, (pb[0], pb[1], zr), (pb[0], tb[1] + 2.0, zr), (hx, tb[1] + 2.0, zr), (hx, tb[1], zr),
                                                            (hx, tb[1], zt_e), (tb[0], tb[1], zt_e)]))
    # ESP32 power: PCA-R V+ terminal -> Schottky diode -> ESP32 5V pin (A1)
    vp = pca_world(1, (0.2, 28.6, 6.7))
    yd0, yd1 = DIODE['y']
    zd = Z_DECK[1] + DIODE['r']
    W.append(('P1a PCA-R V+ to diode', R_SIG, [vp, (0.0, vp[1], vp[2]), (0.0, 24.5, vp[2]), (DIODE['x'], 24.5, vp[2]), (DIODE['x'], 24.5, zt + 0.7),
                                               (DIODE['x'], yd0 - 1.0, zt + 0.7), (DIODE['x'], yd0 - 1.0, zd), (DIODE['x'], yd0 + 0.2, zd)]))
    e5 = esp_pin('A', 1)
    W.append(('P1b diode to ESP32 5V', R_SIG, [(DIODE['x'], yd1 - 0.2, zd), (DIODE['x'], e5[1], zd), (DIODE['x'], e5[1], 27.0), (e5[0], e5[1], 27.0),
                                               (e5[0], e5[1], e5[2] + 14.0)]))
    # ESP32 -> PCA-R front header (GND, OE, SCL, SDA on row B; VCC from 3V3 on row A)
    rowB = {'GND': 19, 'SCL': 17, 'SDA': 14, 'OE': 7}
    lane = {'GND': (27.0, 24.0), 'SCL': (28.5, 25.5), 'SDA': (30.0, 27.0), 'OE': (31.5, 28.5)}
    for nm in ('GND', 'OE', 'SCL', 'SDA'):
        e = esp_pin('B', rowB[nm])
        q = i2c_pin(1, 'front', I2C_PINS.index(nm))
        z, yt = lane[nm]
        W.append(('E %s ESP32 to PCA-R' % nm, R_SIG, [(e[0], e[1], e[2] + 14.0), (e[0], e[1], z), (e[0], yt, z), (q[0], yt, z), (q[0], q[1], z),
                                                      (q[0], q[1], q[2] + 14.0)]))
    e = esp_pin('A', 19)
    q = i2c_pin(1, 'front', I2C_PINS.index('VCC'))
    W.append(('E VCC ESP32 3V3 to PCA-R', R_SIG, [(e[0], e[1], e[2] + 14.0), (e[0], e[1], 33.0), (e[0], 30.0, 33.0), (q[0], 30.0, 33.0), (q[0], q[1], 33.0),
                                                   (q[0], q[1], q[2] + 14.0)]))
    # PCA-R rear header -> PCA-L rear header (not V+: each board has its own buck)
    for j, nm in enumerate(I2C_PINS[:5]):
        a = i2c_pin(1, 'rear', j)
        c = i2c_pin(-1, 'rear', j)
        z = 24.0 + 1.6 * j
        W.append(('C %s PCA-R to PCA-L' % nm, R_SIG, [(a[0], a[1], a[2] + 14.0), (a[0], a[1], z), (c[0], c[1], z), (c[0], c[1], c[2] + 14.0)]))
    # battery sense: BMS P+ -> 100k/47k divider -> GPIO34 (A15), divider foot -> GND (A6)
    p = bms_pad('B+', 2)
    hx, hy = WIRE_HOLES[4]
    dz = (DIVIDER['z'][0] + DIVIDER['z'][1]) / 2
    dxc = (DIVIDER['x'][0] + DIVIDER['x'][1]) / 2
    W.append(('S1 BMS P+ to divider', R_SIG, [p, (p[0], p[1], -5.5), (p[0], 40.0, -5.5), (p[0], 40.0, -2.5), (hx, 40.0, -2.5), (hx, hy, -2.5),
                                              (hx, hy, dz), (dxc, DIVIDER['y'][0] - 0.2, dz)]))
    e = esp_pin('A', 15)
    W.append(('S2 divider to GPIO34', R_SIG, [(dxc, DIVIDER['y'][1] + 0.2, DIVIDER['z'][0] + 0.9), (dxc, 56.0, DIVIDER['z'][0] + 0.9),
                                              (dxc, 56.0, 28.0), (e[0], e[1], 28.0), (e[0], e[1], e[2] + 14.0)]))
    e = esp_pin('A', 6)
    W.append(('S3 divider to ESP32 GND', R_SIG, [(dxc, DIVIDER['y'][1] + 0.2, DIVIDER['z'][1] - 0.9), (dxc, 52.5, DIVIDER['z'][1] - 0.9),
                                                 (dxc, 52.5, 29.5), (dxc, e[1], 29.5), (e[0], e[1], 29.5), (e[0], e[1], e[2] + 14.0)]))
    return W


def dupont_ends():
    """Dupont housings (world) on the ESP32 and PCA 6-pin headers."""
    out = []
    for nm, i in (('GND', 19), ('SCL', 17), ('SDA', 14), ('OE', 7)):
        out.append(('Plug ESP32 B%d %s' % (i, nm), dupont(esp_pin('B', i))))
    for nm, i in (('3V3', 19), ('GPIO34', 15), ('GND', 6), ('5V', 1)):
        out.append(('Plug ESP32 A%d %s' % (i, nm), dupont(esp_pin('A', i))))
    for j, nm in enumerate(I2C_PINS[:5]):
        out.append(('Plug PCA-R front %s' % nm, dupont(i2c_pin(1, 'front', j))))
        out.append(('Plug PCA-R rear %s' % nm, dupont(i2c_pin(1, 'rear', j))))
        out.append(('Plug PCA-L rear %s' % nm, dupont(i2c_pin(-1, 'rear', j))))
    return out


# ----------------------------------------------------------------------------
# Fusion plumbing (from Mk1)
# ----------------------------------------------------------------------------
def edge_mid(e):
    ok, t0, t1 = e.evaluator.getParameterExtents()
    ok, p = e.evaluator.getPointAtParameter((t0 + t1) / 2)
    return p


def concave_edges(body, axis, skip=None, any_geom=False):
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
    n = 0
    for e in list(bed_edges(body, bed)):
        one = adsk.core.ObjectCollection.create()
        one.add(e)
        inp = comp.features.chamferFeatures.createInput2()
        inp.chamferEdgeSets.addEqualDistanceChamferEdgeSet(one, adsk.core.ValueInput.createByReal(cm(dist * 0.6)), False)
        try:
            comp.features.chamferFeatures.add(inp)
            n += 1
        except Exception:
            pass
    return n


def add_body(comp, temp, name):
    bf = comp.features.baseFeatures.add()
    bf.startEdit()
    comp.bRepBodies.add(temp, bf)
    bf.finishEdit()
    bf.name = name
    body = comp.bRepBodies.item(comp.bRepBodies.count - 1)
    body.name = name
    return body


def build_printed(comp, spec, g, name, log):
    body = add_body(comp, spec['solid'], name)
    nf = 0
    if FAST:
        spec = dict(spec, fillet_axis=None, fillet2=False, chamfer=False, edge_break=False)
    if spec.get('fillet_axis') is not None:
        nf = try_fillet(comp, concave_edges(body, spec['fillet_axis'], spec.get('fillet_skip')), spec.get('fillet_r', g.fil))
        body = comp.bRepBodies.item(0)
    if spec.get('fillet2', True):
        nf += try_fillet(comp, concave_edges(body, None, spec.get('fillet_skip'), any_geom=True), spec.get('fillet2_r', 0.8))
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
        bed = spec['bed']
        ne = try_fillet(comp, bed_edges(body, (-bed[0], -bed[1], -bed[2])), 0.6)
        body = comp.bRepBodies.item(0)
    body.name = name
    if comp.bRepBodies.count != 1:
        raise RuntimeError('%s has %d bodies (expected 1)' % (name, comp.bRepBodies.count))
    bb = body.boundingBox
    ext = [(bb.maxPoint.x - bb.minPoint.x) * 10, (bb.maxPoint.y - bb.minPoint.y) * 10, (bb.maxPoint.z - bb.minPoint.z) * 10]
    log.append(dict(part=name, bed=spec['bed'], extents_mm=[round(v, 1) for v in ext], volume_cm3=round(body.volume, 2),
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


# Fusion joint limits (deg).  Coxa +-25 = firmware limit (the gait uses <= 16; neighbouring legs
# are clear of each other up to 25, the body up to 40 - see the sweep).  sim femur -35..+55 -> Fusion -55..+35 (+ = down);
# sim tibia -130..-40 -> Fusion +40..-50 (+ = fold in).
JOINT_LIMITS = {'coxa': (-25, 25), 'femur': (-55, 35), 'tibia': (-50, 40)}


def get_design():
    """Activate (or create) the Hexapod Mini design document."""
    for k in range(app.documents.count):
        doc = app.documents.item(k)
        try:
            des = adsk.fusion.Design.cast(doc.products.itemByProductType('DesignProductType'))
        except Exception:
            des = None
        if des and des.rootComponent.name == DOC_NAME:
            doc.activate()
            return des
        if des and des.attributes.itemByName('HexapodMini', 'doc'):
            doc.activate()
            return des
    doc = app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
    des = adsk.fusion.Design.cast(doc.products.itemByProductType('DesignProductType'))
    des.attributes.add('HexapodMini', 'doc', DOC_NAME)
    try:
        des.rootComponent.name = DOC_NAME
    except Exception:
        pass
    return des


def build(out_dir=None):
    design = get_design()
    design.designType = adsk.fusion.DesignTypes.ParametricDesignType
    root = design.rootComponent
    ensure_params(design)
    for k in range(root.asBuiltJoints.count - 1, -1, -1):
        root.asBuiltJoints.item(k).deleteMe()
    for k in range(root.rigidGroups.count - 1, -1, -1):
        root.rigidGroups.item(k).deleteMe()
    for k in range(root.occurrences.count - 1, -1, -1):
        root.occurrences.item(k).deleteMe()
    g = Geo(design)
    B = Body(g)
    log, printed, cache, counts, harness_report = [], {}, {}, {}, []
    I3 = adsk.core.Matrix3D.create
    ORANGE = ('Plastic - Matte (Orange)', 'Plastic - Glossy (Orange)', 'Plastic - Matte (Red)')
    GREY = ('Plastic - Matte (Gray)', 'Plastic - Matte (Grey)', 'Plastic - Glossy (Gray)')
    BLUE = ('Plastic - Matte (Blue)', 'Plastic - Glossy (Blue)')
    BLACK = ('Plastic - Matte (Black)', 'Plastic - Glossy (Black)')
    GREEN = ('Plastic - Matte (Green)', 'Plastic - Glossy (Green)')
    RED = ('Plastic - Matte (Red)', 'Plastic - Glossy (Red)')

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

    body_occs = []
    tray = place('Tray', lambda: part_tray(B), I3(), GREY)
    tray.isGrounded = True
    body_occs.append(tray)
    body_occs.append(place('Deck', lambda: part_deck(B), I3(), GREY))
    body_occs.append(place('BatteryHatch', lambda: part_hatch(B), I3(), GREY))
    body_occs.append(place('PCA9685_R_0x40', lambda: pca_env(1), I3(), BLUE))
    body_occs.append(place('PCA9685_L_0x41', lambda: pca_env(-1), I3(), BLUE))
    body_occs.append(place('ESP32_DevKitC', esp_env, I3(), BLACK))
    body_occs.append(place('Mini560_R', lambda: buck_env(1), I3(), BLUE))
    body_occs.append(place('Mini560_L', lambda: buck_env(-1), I3(), BLUE))
    body_occs.append(place('BMS_2S', bms_env, I3(), GREEN))
    body_occs.append(place('Holder_2x18650_with_cells', holder_env, I3(), BLACK))
    body_occs.append(place('RockerSwitch_KCD1', switch_env, I3(), RED))
    body_occs.append(place('FuseHolder_ATC_mini', fuse_env, I3(), RED))
    body_occs.append(place('Divider_100k_47k', divider_env, I3(), BLACK))
    body_occs.append(place('Diode_1N5822', diode_env, I3(), BLACK))

    def harness(name, r, pts, group):
        occ = new_comp(root, 'Harness ' + name)
        add_body(occ.component, pipe(pts, r), 'Harness ' + name)
        paint(occ.component, design, list(BLACK))
        group.append(occ)
        harness_report.append(dict(cable=name, d_mm=2 * r, length_mm=round(sum(math.dist(a, b) for a, b in zip(pts[:-1], pts[1:])), 0)))
        return occ

    def plug(name, temp, group):
        occ = new_comp(root, name)
        add_body(occ.component, temp, name)
        paint(occ.component, design, list(BLACK))
        group.append(occ)

    for (name, r, pts, (side, k)) in body_servo_leads(B):
        harness(name + ' (body)', r, pts, body_occs)
        plug('Plug servo %s ch%d' % ('R' if side > 0 else 'L', k), servo_plug(side, k), body_occs)
    for (name, r, pts) in power_wires():
        harness(name, r, pts, body_occs)
    for (name, temp) in dupont_ends():
        plug(name, temp, body_occs)

    leg_groups = []
    lines = {}
    for i in range(6):
        lm = g.leg_m(i)
        fm = chain(g.fem_local(), lm)
        tm = chain(g.tib_local(), fm)
        body_occs.append(place('MG90S', mg90s_envelope, chain(coxa_servo_m(), lm), BLUE))
        cx = place('Coxa', lambda: part_coxa(g), lm, ORANGE)
        if 'Coxa' not in lines:
            lines['Coxa'] = axis_line(cache['Coxa'], (0, 0, -30), (0, 0, 20))
        gc = [cx, place('MG90S', mg90s_envelope, chain(femur_servo_m(g), lm), BLUE),
              place('Horn', horn_envelope, chain(coxa_horn_m(), lm), GREY)]
        for joint in ('femur', 'tibia'):
            pts = [apply_m(lm, p) for p in coxa_seg(joint, g)]
            harness('Leg%d %s lead (coxa)' % (i, joint), R_SERVO, pts, gc)
        fe = place('Femur', lambda: part_femur(g), fm, ORANGE)
        if 'Femur' not in lines:
            lines['Femur'] = axis_line(cache['Femur'], (0, -25, 0), (0, 25, 0))
        gf = [fe, place('MG90S', mg90s_envelope, chain(tibia_servo_m(g), fm), BLUE),
              place('Horn', horn_envelope, chain(femur_horn_m(), fm), GREY)]
        harness('Leg%d tibia lead (femur)' % i, R_SERVO, [apply_m(fm, p) for p in femur_seg(g)], gf)
        ti = place('Tibia', lambda: part_tibia(g), tm, ORANGE)
        if 'Tibia' not in lines:
            lines['Tibia'] = axis_line(cache['Tibia'], (0, -25, 0), (0, 25, 0))
        gt = [ti, place('Horn', horn_envelope, chain(tibia_horn_m(), tm), GREY)]
        leg_groups.append((i, gc, gf, gt))

    rigid(root, body_occs, 'Body group')
    for (i, gc, gf, gt) in leg_groups:
        rigid(root, gc, 'Leg%d coxa group' % i)
        rigid(root, gf, 'Leg%d femur group' % i)
        rigid(root, gt, 'Leg%d tibia group' % i)
        revolute(root, gc[0], tray, lines['Coxa'], gc[0], *JOINT_LIMITS['coxa'], 'Leg%d coxa' % i)
        revolute(root, gf[0], gc[0], lines['Femur'], gf[0], *JOINT_LIMITS['femur'], 'Leg%d femur' % i)
        revolute(root, gt[0], gf[0], lines['Tibia'], gt[0], *JOINT_LIMITS['tibia'], 'Leg%d tibia' % i)

    # mass: printed volume x 1.24 g/cm3 x 0.55 effective fill (PLA, 3 walls, 25 % gyroid)
    printed_g = 0.0
    for row in log:
        q = counts.get(row['part'], 1)
        row['qty'] = q
        row['est_g_each'] = round(row['volume_cm3'] * 1.24 * 0.55, 1)
        printed_g += row['est_g_each'] * q
    bought_g = (18 * MASS_G['MG90S'] + 18 * MASS_G['Horn'] + 2 * MASS_G['PCA9685'] + MASS_G['ESP32_DevKitC'] + 2 * MASS_G['Mini560']
                + MASS_G['BMS_2S'] + MASS_G['Holder_2x18650_with_cells'] + MASS_G['FuseHolder_ATC_mini'] + MASS_G['RockerSwitch_KCD1']
                + MASS_G['Diode_1N5822'] + MASS_G['Divider_100k_47k'])
    hardware_g = 18.0     # ~60 M2 screws + horn screws, 18 M3 pin screws, 6 M3 deck/hatch screws, M2 nuts
    wiring_g = 14.0       # 20 AWG silicone, Dupont jumpers, heat-shrink (servo leads are counted with the servos)
    total_g = printed_g + bought_g + hardware_g + wiring_g
    report = dict(parts=log, harness=harness_report, joints=root.asBuiltJoints.count, occurrences=root.allOccurrences.count,
                  mass_g=dict(printed=round(printed_g), bought=round(bought_g), hardware=hardware_g, wiring=wiring_g, total=round(total_g)),
                  geometry=dict(coxa=g.coxa, femur=g.femur, tibia=g.tibia, mounts=g.mounts, joint_limits_fusion=JOINT_LIMITS,
                                deck=Z_DECK, floor=Z_FLOOR, belly_floor=Z_BELLY - HATCH_T, lowest_leg_point=Z_LA[0]),
                  channels={str(i): CHANNEL[i] for i in range(6)})
    if out_dir:
        with open(_L(os.path.join(out_dir, 'build_report.json')), 'w') as f:
            json.dump(report, f, indent=1)
    return report, printed


def _L(path):
    path = os.path.abspath(path)
    if os.name == 'nt' and not path.startswith('\\\\?\\'):
        return '\\\\?\\' + path
    return path


def export_stls(printed, stl_dir):
    """Raw STL export (component frame).  Then run
    python robot-sim/cad/orient_stl.py <raw> robot-sim/mini/cad/stl <raw>/export_manifest.json
    to put them in print orientation."""
    em = adsk.fusion.Design.cast(app.activeProduct).exportManager
    out = []
    for nm, (comp, bed) in printed.items():
        body = comp.bRepBodies.item(0)
        fn = os.path.join(stl_dir, nm + '.stl')
        opt = em.createSTLExportOptions(body, fn)
        opt.meshRefinement = adsk.fusion.MeshRefinementSettings.MeshRefinementHigh
        opt.isBinaryFormat = True
        em.execute(opt)
        out.append(dict(part=nm, bed=list(bed)))
    with open(os.path.join(stl_dir, 'export_manifest.json'), 'w') as f:
        json.dump(out, f, indent=1)
    return out


def save_snapshots(out_dir, w=1600, h=1000):
    """Renders for the docs.  Sets the camera explicitly with +Z up (a new Fusion document
    may be Y-up, which turns the standard views on their side) and hides the joint glyphs."""
    design = adsk.fusion.Design.cast(app.activeProduct)
    design.rootComponent.isJointsFolderLightBulbOn = False
    vp = app.activeViewport
    views = [('mini_iso', (0.55, 0.62, 0.5), (0, 0, -12), 330),
             ('mini_front', (0, 1, 0), (0, 0, -12), 300),
             ('mini_top', (0, 0, 1), (0, 0, 0), 300),
             ('mini_leg', (0.25, -1, 0.3), (95, -15, -15), 150)]
    for nm, eye, tgt, dist in views:
        cam = vp.camera
        cam.isSmoothTransition = False
        cam.cameraType = adsk.core.CameraTypes.OrthographicCameraType
        cam.target = P(*tgt)
        cam.eye = P(*[tgt[k] + eye[k] * dist for k in range(3)])
        d = _vec(eye)
        up = (0, 1, 0) if abs(d[2]) > 0.99 else _vec([0 - d[2] * d[0], 0 - d[2] * d[1], 1 - d[2] * d[2]])   # +Z, made square to the view
        cam.upVector = adsk.core.Vector3D.create(*up)
        cam.viewExtents = cm(dist) * 0.9
        vp.camera = cam
        vp.refresh()
        vp.saveAsImageFile(os.path.join(out_dir, nm + '.png'), w, h)


def run(_context: str):
    build()
