"""Hexapod Mk1 - parametric generator for Autodesk Fusion.

Run it inside Fusion (Utilities > Scripts and Add-Ins, or through the Fusion MCP
server).  It builds the design "Hexapod Mk1" from scratch in the active design:
one component per unique part, user parameters named after the fields in
robot-sim/src/robots/hexapod.config.js, revolute joints at every coxa, femur and
tibia axis, and servo / electronics envelopes.

To change the robot: edit the user parameters (Modify > Change Parameters),
then run this script again with REBUILD = True.  It reads the parameters back,
deletes the old geometry and rebuilds every part.  Keep hexapod.config.js in
step so the simulator stays the digital twin.

Frames
------
Robot frame (Fusion world): +X = robot RIGHT, +Y = FORWARD, +Z = UP, origin at
the body centre half way between the plates.
Simulator frame (three.js): +X = robot LEFT, +Y = UP, +Z = FORWARD.
So  X_fusion = -x_sim,  Y_fusion = z_sim,  Z_fusion = y_sim.

Leg-part frames: origin on the joint axis, +X along the leg, +Z up, +Y = left of
the leg (Z cross X).  The leg's centre plane is Y = 0.

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
    ('body_length', '260 mm', 'mm', 'config body.length - core hexagon length (plate grows lobes around the coxa servos)'),
    ('body_width', '140 mm', 'mm', 'config body.width - core hexagon width'),
    ('body_thickness', '5 mm', 'mm', 'config body.thickness - plate thickness'),
    ('body_plateGap', '45 mm', 'mm', 'config body.plateGap - clear gap between the plates'),
    ('body_lobeRadius', '21 mm', 'mm', 'config body.lobeRadius - plate lobe radius around each coxa axis'),
    ('leg_coxa', '45 mm', 'mm', 'config leg.coxa - coxa axis to femur axis'),
    ('leg_femur', '75 mm', 'mm', 'config leg.femur - femur axis to tibia axis'),
    ('leg_tibia', '120 mm', 'mm', 'config leg.tibia - tibia axis to foot contact (incl. 3 mm TPU boot)'),
    ('leg_footReach', '130 mm', 'mm', 'config leg.footReach'),
    ('standHeight', '90 mm', 'mm', 'config standHeight'),
    ('topMount_y', '13 mm', 'mm', 'config topMount[1] - attachment datum above the top plate'),
    ('topMount_z', '-10 mm', 'mm', 'config topMount[2] - attachment datum, sim z (forward)'),
    ('frontMount_y', '-12 mm', 'mm', 'config frontMount[1] - sonar centre height'),
    ('frontMount_z', '135 mm', 'mm', 'config frontMount[2] - sonar bracket front'),
    ('fit_clearance', '0.25 mm', 'mm', 'clearance per side for servo pockets and drop-in fits'),
    ('slide_clearance', '0.2 mm', 'mm', 'clearance per side for the dovetail and jigsaw seams'),
    ('insert_hole', '4.0 mm', 'mm', 'hole for M3 x 4 x 5 heat-set inserts'),
    ('clear_hole', '3.4 mm', 'mm', 'M3 clearance hole'),
    ('bearing_pocket', '10.1 mm', 'mm', '623ZZ (3x10x4) press-fit pocket'),
    ('horn_pcd', '18 mm', 'mm', 'bolt circle of the 25T aluminium servo horn (check yours)'),
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
# Purchased-part envelopes
# ----------------------------------------------------------------------------
# MG996R (TowerPro drawing, nominal; measure yours).  Servo frame: shaft = +Z,
# origin on the shaft axis in the plane of the underside of the mounting tabs,
# case extends toward +X (shaft is 10 mm from the case end).
MG996R = dict(L=40.7, W=19.7, shaft_off=10.0, below_tab=26.6, above_tab=10.0, tab_t=2.5,
              tab_len=54.5, tab_pitch=49.5, tab_w=10.0, tab_hole=4.5, boss_d=13.0,
              boss_top=12.8, spline_d=5.9, spline_top=16.3, horn_top=17.3, mass=55.0)
MG90S = dict(L=22.8, W=12.2, shaft_off=5.9, below_tab=15.9, above_tab=6.8, tab_t=2.5,
             tab_len=32.5, tab_pitch=27.8, tab_w=0.0, tab_hole=2.2, boss_d=11.6,
             boss_top=9.0, spline_d=4.8, spline_top=12.6, horn_top=13.6, mass=13.4)


def servo_env(s):
    L, W, o = s['L'], s['W'], s['shaft_off']
    body = box(-o, L - o, -W / 2, W / 2, -s['below_tab'], s['above_tab'])
    c = L / 2 - o
    tabs = box(c - s['tab_len'] / 2, c + s['tab_len'] / 2, -W / 2, W / 2, 0, s['tab_t'])
    for sx in (-1, 1):
        for sy in ((-1, 1) if s['tab_w'] else (0,)):
            px, py = c + sx * s['tab_pitch'] / 2, sy * s['tab_w'] / 2
            D(tabs, cyl((px, py, -1), (px, py, 4), s['tab_hole'] / 2))
    U(body, tabs)
    U(body, cyl((0, 0, s['above_tab'] - 0.5), (0, 0, s['boss_top']), s['boss_d'] / 2))
    U(body, cyl((0, 0, s['boss_top']), (0, 0, s['spline_top']), s['spline_d'] / 2))
    return body


def servo_tab_holes(s):
    """(x, y) of the tab screw holes in the servo frame."""
    c = s['L'] / 2 - s['shaft_off']
    out = []
    for sx in (-1, 1):
        for sy in ((-1, 1) if s['tab_w'] else (0,)):
            out.append((c + sx * s['tab_pitch'] / 2, sy * s['tab_w'] / 2))
    return out


def horn_env(s, pcd):
    """25T round aluminium horn: hub on the spline, 3 mm disc, 4 x M3 on the PCD."""
    top = s['horn_top']
    b = cyl((0, 0, s['boss_top']), (0, 0, top - 3), 4.5)
    U(b, cyl((0, 0, top - 3), (0, 0, top), 12))
    D(b, cyl((0, 0, s['boss_top'] - 0.1), (0, 0, s['spline_top']), s['spline_d'] / 2 + 0.05))
    for k in range(4):
        a = math.radians(45 + 90 * k)
        x, y = pcd / 2 * math.cos(a), pcd / 2 * math.sin(a)
        D(b, cyl((x, y, top - 4), (x, y, top + 1), 1.25))
    return b


def bearing_623():
    """623ZZ 3 x 10 x 4, axis along Z, centred on the origin."""
    return D(cyl((0, 0, -2), (0, 0, 2), 5.0), cyl((0, 0, -3), (0, 0, 3), 1.5))


# ----------------------------------------------------------------------------
# Printed leg parts
# ----------------------------------------------------------------------------
class Geo:
    """Shared leg-chain numbers, derived from the parameters."""

    def __init__(self, d):
        self.T = pmm(d, 'body_thickness')
        self.GAP = pmm(d, 'body_plateGap')
        self.z_top = self.GAP / 2 + self.T          # top face of the top plate
        self.z_bot = -self.GAP / 2 - self.T         # bottom face of the bottom plate
        self.Lc = pmm(d, 'leg_coxa')
        self.Lf = pmm(d, 'leg_femur')
        self.Lt = pmm(d, 'leg_tibia')
        self.lobe = pmm(d, 'body_lobeRadius')
        self.fit = pmm(d, 'fit_clearance')
        self.slide = pmm(d, 'slide_clearance')
        self.ins = pmm(d, 'insert_hole')
        self.clr = pmm(d, 'clear_hole')
        self.brg = pmm(d, 'bearing_pocket')
        self.pcd = pmm(d, 'horn_pcd')
        self.cham = pmm(d, 'first_layer_chamfer')
        self.fil = pmm(d, 'root_fillet')
        s = MG996R
        # Servo held in a cradle with its shaft along +Y: back wall 5 mm, then
        # the case, tab seat at y_tab, horn top at y_horn.  Everything is
        # shifted so the leg centre plane (between horn arm and idler arm) is y=0.
        back = 5.0
        washer = 0.5
        arm_horn = 5.5
        arm_idler = 6.0
        t_bottom = back + 0.3
        t_tab = t_bottom + s['below_tab']
        t_horn = t_tab + s['horn_top']
        idler_out = -washer - arm_idler
        horn_out = t_horn + arm_horn
        tc = (idler_out + horn_out) / 2
        self.y_back = -tc
        self.y_bottom = t_bottom - tc
        self.y_tab = t_tab - tc
        self.y_horn = t_horn - tc
        self.y_horn_out = horn_out - tc
        self.y_idler_in = -washer - tc
        self.y_idler_out = idler_out - tc
        # coxa horn top above the top plate
        self.z_coxa_horn = self.z_top + s['horn_top']
        self.z_coxa_lower_top = self.z_bot - washer


def cradle(g, ax, along, x_ax):
    """Servo cradle block + pocket tool, servo shaft at (ax, ., 0) along +Y.

    along = -1 : servo case runs toward -Z (coxa); along = 'x-' : toward -X (femur).
    Returns (solid, tools, tab_points_xz)."""
    s = MG996R
    W, f = s['W'], g.fit
    near, far = s['shaft_off'], s['L'] - s['shaft_off']
    c_off = s['L'] / 2 - s['shaft_off']
    holes = [(c_off - s['tab_pitch'] / 2), (c_off + s['tab_pitch'] / 2)]
    if along == 'z-':
        solid = box(ax - W / 2 - 3.0, ax + W / 2 + 3.0, g.y_back, g.y_tab,
                    -(c_off + s['tab_pitch'] / 2) - 4.9, -(c_off - s['tab_pitch'] / 2) + 4.6)
        pocket = box(ax - W / 2 - f, ax + W / 2 + f, g.y_bottom - 0.3, g.y_tab + 2,
                     -far - f, near + f)
        wire = box(ax - 4, ax + 4, g.y_bottom - 0.3, g.y_bottom + 8, -far - 12, -far + 1)
        tabs = [(ax + sy * s['tab_w'] / 2, -h) for h in holes for sy in (-1, 1)]
    else:  # 'x-'
        solid = box(ax - (c_off + s['tab_pitch'] / 2) - 4.9, ax - (c_off - s['tab_pitch'] / 2) + 5.6,
                    g.y_back, g.y_tab, -13.0, 13.0)
        pocket = box(ax - far - f, ax + near + f, g.y_bottom - 0.3, g.y_tab + 2,
                     -W / 2 - f, W / 2 + f)
        wire = box(ax - far - 12, ax - far + 1, g.y_bottom - 0.3, g.y_bottom + 8, -4, 4)
        tabs = [(ax - h, sy * s['tab_w'] / 2) for h in holes for sy in (-1, 1)]
    tools = [pocket, wire]
    for (x, z) in tabs:
        tools.append(cyl((x, g.y_tab + 1, z), (x, g.y_tab - 6, z), g.ins / 2))
    # idler insert in the back wall, coaxial with the shaft
    tools.append(cyl((ax, g.y_back - 1, 0), (ax, g.y_back + 4.8, 0), g.ins / 2))
    return solid, tools


def horn_arm_holes(g, cx, cz, y0, y1, up):
    t = [hole((cx, y0 - 1, cz), (cx, y1 + 1, cz), 9.0, up)]
    for k in range(4):
        a = math.radians(45 + 90 * k)
        x, z = cx + g.pcd / 2 * math.cos(a), cz + g.pcd / 2 * math.sin(a)
        t.append(hole((x, y0 - 1, z), (x, y1 + 1, z), 3.3, up))
    return t


def idler_arm_holes(g, cx, cz, y_in, y_out, up):
    """Bearing pocket from the inner face (4 mm), 7.4 mm counterbore outside."""
    sgn = 1 if y_out > y_in else -1
    t = [hole((cx, y_in - sgn * 0.5, cz), (cx, y_in + sgn * 4.0, cz), g.brg, up)]
    t.append(hole((cx, y_in + sgn * 3.5, cz), (cx, y_out + sgn * 1.0, cz), 7.4, up))
    return t


def part_coxa(g):
    """Coxa: U clevis round the coxa servo (horn on top, 623 idler below the
    bottom plate) + cradle for the femur servo.  Prints on its flat back (-Y)."""
    Lc = g.Lc
    zt0, zt1 = g.z_coxa_horn, g.z_coxa_horn + 5.0
    zb1 = g.z_coxa_lower_top
    zb0 = zb1 - 6.0
    xw0, xw1 = Lc - 21, Lc - 11
    solid, tools = cradle(g, Lc, 'z-', None)
    web = box(xw0, xw1, g.y_back, g.y_tab, solid_top(solid) - 0.01, zt1)
    U(solid, web)
    for (z0, z1) in ((zt0, zt1), (zb0, zb1)):
        arm = box(0, xw1, g.y_back, g.y_tab, z0, z1)
        U(arm, cyl((0, 0, z0), (0, 0, z1), 15.0))
        U(arm, box(-15, 0, g.y_back, 0, z0, z1))
        U(solid, arm)
    up = (0, 1, 0)
    # horn bolts through the top arm (vertical in the robot = horizontal in the print)
    tools.append(hole((0, 0, zt0 - 1), (0, 0, zt1 + 1), 9.0, up))
    for k in range(4):
        a = math.radians(45 + 90 * k)
        x, y = g.pcd / 2 * math.cos(a), g.pcd / 2 * math.sin(a)
        tools.append(hole((x, y, zt0 - 1), (x, y, zt1 + 1), 3.3, up))
    # 623 idler in the bottom arm: pocket from the top face, counterbore below
    tools.append(hole((0, 0, zb1 + 0.5), (0, 0, zb1 - 4.0), g.brg, up))
    tools.append(hole((0, 0, zb1 - 3.5), (0, 0, zb0 - 1), 7.4, up))
    return dict(solid=solid, tools=tools, bed=(0, -1, 0), fillet_axis=(0, 1, 0))


def solid_top(b):
    return b.boundingBox.maxPoint.z * 10.0


def part_femur(g):
    """Femur: clevis round the femur servo + cradle for the tibia servo.
    Prints lying on its side (-Z face down); holes are teardrops."""
    Lf = g.Lf
    h = 13.0
    solid, tools = cradle(g, Lf, 'x-', None)
    x_r0, x_r1 = 26.0, 34.0   # riser ends 3.4 mm short of the tibia servo tab (fillet r3 fits)
    for (y0, y1) in ((g.y_horn, g.y_horn_out), (g.y_idler_out, g.y_idler_in)):
        arm = U(octagon_y(0, 0, h, y0, y1), box(0, x_r1, y0, y1, -h, h))
        U(solid, arm)
    U(solid, box(x_r0, x_r1, g.y_tab - 0.01, g.y_horn_out, -h, h))
    U(solid, box(x_r0, x_r1, g.y_idler_out, g.y_back + 0.01, -h, h))
    # fill between the risers and the cradle so it is one solid block
    U(solid, box(x_r0, 35.01, g.y_back, g.y_tab, -h, h))
    up = (0, 0, 1)
    tools += horn_arm_holes(g, 0, 0, g.y_horn, g.y_horn_out, up)
    tools += idler_arm_holes(g, 0, 0, g.y_idler_in, g.y_idler_out, up)
    return dict(solid=solid, tools=tools, bed=(0, 0, -1), fillet_axis=(0, 0, 1))


def part_tibia(g):
    """Tibia: clevis round the tibia servo, tapered A-frame shank, rounded
    foot for a TPU boot.  Prints on its side (-X face down)."""
    h = 12.0
    yo = g.y_horn_out
    z_tip = -(g.Lt - 3.0 - 8.0)   # boot is 3 mm, foot radius 8 mm
    solid = None
    for (y0, y1) in ((g.y_horn, g.y_horn_out), (g.y_idler_out, g.y_idler_in)):
        arm = U(octagon_y(0, 0, h, y0, y1), box(-h, h, y0, y1, -40, 0))
        solid = arm if solid is None else U(solid, arm)
    U(solid, box(-h, h, -yo, yo, -46, -36))
    shank = box(-h, h, -yo, yo, z_tip + 14, -46)
    for s in (1, -1):
        p0 = (0, s * yo, -46)
        p1 = (0, s * 8.0, z_tip + 14)
        dy, dz = p1[1] - p0[1], p1[2] - p0[2]
        n = (0, s * abs(dz), -abs(dy)) if s > 0 else (0, -abs(dz), -abs(dy))
        D(shank, halfspace(p0, n))
    U(solid, shank)
    U(solid, box(-h, h, -8, 8, z_tip, z_tip + 14.01))
    U(solid, cyl((-h, 0, z_tip), (h, 0, z_tip), 8.0))
    D(solid, box(-h - 1, h + 1, -16, 16, -66, -50))     # lightening window
    up = (1, 0, 0)
    tools = horn_arm_holes(g, 0, 0, g.y_horn, g.y_horn_out, up)
    tools += idler_arm_holes(g, 0, 0, g.y_idler_in, g.y_idler_out, up)
    return dict(solid=solid, tools=tools, bed=(-1, 0, 0), fillet_axis=(1, 0, 0))


def part_foot_boot(g):
    """TPU boot over the tibia tip (adds 3 mm, grips the floor)."""
    z_tip = -(g.Lt - 3.0 - 8.0)
    outer = U(box(-14, 14, -11, 11, z_tip, z_tip + 12), cyl((-14, 0, z_tip), (14, 0, z_tip), 11.0))
    inner = U(box(-12.2, 12.2, -8.2, 8.2, z_tip, z_tip + 13), cyl((-12.2, 0, z_tip), (12.2, 0, z_tip), 8.2))
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
        self.standoffs = [(32, 60), (-32, 60), (36, 0), (-36, 0), (32, -60), (-32, -60)]
        # attachment grid: 20 mm pitch, centred on the attachment datum
        self.grid = []
        for x in (-20, 0, 20):
            for y in range(-70, 71, 20):
                if x == 0 and y == -50:
                    continue  # cable pass-through
                self.grid.append((x, y))
        self.pass_through = (0, -55, 30, 14)
        self.carrier_screws = [(0, 62), (0, 124)]
        self.bracket_inserts = [(18, 124), (-18, 124)]
        self.panel_screw = (35, -120)
        self.tray_inserts = [(25, -30), (-25, -30), (25, -110), (-25, -110)]
        # two PCA9685 boards (62.5 x 25.4, ~M2.5 holes 55.9 x 19.0 - check yours)
        self.pca = [(-14.3, 39.25), (14.3, 39.25)]

    def outline(self, z0, z1):
        L2, W2 = self.L / 2, self.W / 2
        b = box(-W2 * 1.15, W2 * 1.15, -L2, L2, z0, z1)
        for sx in (1, -1):
            for sy in (1, -1):
                p0 = (sx * W2 * 0.75, sy * L2, 0)
                p1 = (sx * W2 * 1.15, sy * 0.03 * 1000, 0)
                dx, dy = p1[0] - p0[0], p1[1] - p0[1]
                n = _vec([dy, -dx, 0])
                if n[0] * sx < 0:
                    n = [-n[0], -n[1], 0]
                D(b, halfspace(p0, n))
        for lg in self.legs:
            U(b, cyl((lg['X'], lg['Y'], z0), (lg['X'], lg['Y'], z1), self.g.lobe))
        return b

    def keys(self, seam, toward, z0, z1, grow=0.0):
        """T keys (neck 8 x 6, head 16 x 6) sticking out of the seam toward -Y (toward=-1) or +Y."""
        out = None
        for cx in (-33, 33):
            n0, n1 = sorted((seam, seam + toward * 6))
            h0, h1 = sorted((seam + toward * 6, seam + toward * 12))
            k = U(box(cx - 4 - grow, cx + 4 + grow, n0 - (grow if toward < 0 else 0.01), n1 + (grow if toward > 0 else 0.01), z0 - 1 if grow else z0, z1 + 1 if grow else z1),
                  box(cx - 8 - grow, cx + 8 + grow, h0 - grow, h1 + grow, z0 - 1 if grow else z0, z1 + 1 if grow else z1))
            out = k if out is None else U(out, k)
        return out

    def plate(self, which, half):
        g = self.g
        if which == 'top':
            z0, z1, seam, toward = g.GAP / 2, g.z_top, self.seam_top, -1
        else:
            z0, z1, seam, toward = g.z_bot, -g.GAP / 2, self.seam_bot, 1
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
        for (x, y) in self.standoffs:
            tools.append(cyl((x, y, zz[0]), (x, y, zz[1]), g.clr / 2))
        if which == 'top':
            s = MG996R
            for lg in self.legs:
                dx, dy = lg['dir'][0], lg['dir'][1]
                c = s['L'] / 2 - s['shaft_off']
                cx, cy = lg['X'] - dx * c, lg['Y'] - dy * c
                tools.append(obox((cx, cy, (z0 + z1) / 2), (dx, dy, 0), (-dy, dx, 0),
                                  s['L'] + 2 * g.fit, s['W'] + 2 * g.fit, 20))
                for (hx, hy) in servo_tab_holes(s):
                    # servo +X points inward (-dir), servo +Y = Z x X
                    wx = lg['X'] - dx * hx + dy * hy
                    wy = lg['Y'] - dy * hx - dx * hy
                    tools.append(cyl((wx, wy, zz[0]), (wx, wy, zz[1]), g.ins / 2))
            for (x, y) in self.grid + self.bracket_inserts:
                tools.append(cyl((x, y, zz[0]), (x, y, zz[1]), g.ins / 2))
            for (x, y) in self.carrier_screws + [self.panel_screw]:
                tools.append(cyl((x, y, zz[0]), (x, y, zz[1]), g.clr / 2))
            px, py, pw, ph = self.pass_through
            pt = box(px - pw / 2 + ph / 2, px + pw / 2 - ph / 2, py - ph / 2, py + ph / 2, *zz)
            U(pt, cyl((px - pw / 2 + ph / 2, py, zz[0]), (px - pw / 2 + ph / 2, py, zz[1]), ph / 2))
            U(pt, cyl((px + pw / 2 - ph / 2, py, zz[0]), (px + pw / 2 - ph / 2, py, zz[1]), ph / 2))
            tools.append(pt)
        else:
            for lg in self.legs:
                tools.append(cyl((lg['X'], lg['Y'], zz[0]), (lg['X'], lg['Y'], zz[1]), g.ins / 2))
                dx, dy = lg['dir'][0], lg['dir'][1]
                bx, by = lg['X'] - dx * 36, lg['Y'] - dy * 36
                for s_ in (-1, 1):
                    cx, cy = bx - dy * 5 * s_, by + dx * 5 * s_
                    tools.append(obox((cx, cy, (z0 + z1) / 2), (dx, dy, 0), (-dy, dx, 0), 4.5, 2.2, 20))
            for (x, y) in self.tray_inserts:
                tools.append(cyl((x, y, zz[0]), (x, y, zz[1]), g.ins / 2))
            tools.append(cyl((self.panel_screw[0], self.panel_screw[1], zz[0]),
                             (self.panel_screw[0], self.panel_screw[1], zz[1]), g.clr / 2))
            # UBEC cooling slots
            for y in (-12, -6, 0, 6):
                tools.append(box(-18, 18, y - 1.5, y + 1.5, *zz))
            # PCA9685 bosses (M2.5 self-tapping)
            for (cx, cy) in self.pca:
                for sx in (-1, 1):
                    for sy in (-1, 1):
                        x, y = cx + sx * 9.5, cy + sy * 27.95
                        if (y > seam) == (half == 'front'):
                            U(o, cyl((x, y, z1 - 0.01), (x, y, z1 + 4), 3.0))
                        tools.append(cyl((x, y, z1 - 3), (x, y, z1 + 5), 1.1))
        tools = [t for t in tools if t is not None]
        bed = (0, 0, -1)
        return dict(solid=o, tools=tools, bed=bed, fillet_axis=(0, 0, 1), fillet_r=4.0,
                    fillet_skip=lambda p: abs(p[1] - seam) < 14)


def part_battery_tray(b):
    g = b.g
    z0 = -g.GAP / 2
    y0, y1 = -128, -16
    s = box(-21, 21, y0, y1, z0, z0 + 2.5)
    for sx in (-1, 1):
        U(s, box(18.5, 21, y0, y1, z0, z0 + 29.5) if sx > 0 else box(-21, -18.5, y0, y1, z0, z0 + 29.5))
    U(s, box(-21, 21, y1 - 2.5, y1, z0, z0 + 29.5))
    for (x, y) in b.tray_inserts:
        U(s, box(18.5, 29, y - 4, y + 4, z0, z0 + 3) if x > 0 else box(-29, -18.5, y - 4, y + 4, z0, z0 + 3))
    tools = [box(-6, 6, y1 - 4, y1 + 1, z0 + 8, z0 + 20)]   # battery lead exit
    for (x, y) in b.tray_inserts:
        tools.append(cyl((x, y, z0 - 1), (x, y, z0 + 5), g.clr / 2))
    for yc in (-44, -100):                                   # strap slots, 22 mm strap
        tools.append(box(-22.5, 22.5, yc - 11, yc + 11, z0 + 3.5, z0 + 6.5))
    return dict(solid=s, tools=tools, bed=(0, 0, -1), fillet_axis=(0, 1, 0), fillet_r=1.0)


def part_carrier(b):
    """Controller carrier: hangs under the top plate, holds a 50 x 70 perfboard
    (ESP32 on female headers, buck converter, I2C + power distribution)."""
    g = b.g
    zt = g.GAP / 2
    s = box(-27, 27, 56, 130, zt - 3, zt)
    pts = [(x, y) for x in (-23, 23) for y in (60, 126)]
    for (x, y) in pts:
        U(s, cyl((x, y, zt - 8), (x, y, zt - 2.99), 3.5))
    tools = [cyl((x, y, zt - 9), (x, y, zt - 3.5), g.ins / 2) for (x, y) in pts]
    for (x, y) in b.carrier_screws:
        tools.append(cyl((x, y, zt - 10), (x, y, zt + 1), g.ins / 2))
    tools.append(box(-15, 15, 72, 116, zt - 4, zt + 1))      # airflow / weight window
    return dict(solid=s, tools=tools, bed=(0, 0, 1), fillet_axis=None)


def part_power_panel(b):
    g = b.g
    x0, x1, y0, y1 = 24, 46, -128, -124
    s = box(x0, x1, y0, y1, -g.GAP / 2, g.GAP / 2)
    px, py = b.panel_screw
    U(s, box(px - 6, px + 6, y1 - 0.01, -114, g.GAP / 2 - 5, g.GAP / 2))
    U(s, box(px - 6, px + 6, y1 - 0.01, -114, -g.GAP / 2, -g.GAP / 2 + 5))
    tools = [box(35 - 6.6, 35 + 6.6, y0 - 1, y1 + 1, -4.7, 14.7)]  # KCD1 rocker 19.2 x 13
    for z in (g.GAP / 2 + 1, -g.GAP / 2 - 1):
        zz = (z, z - 5.5) if z > 0 else (z, z + 5.5)
        tools.append(cyl((px, py, zz[0]), (px, py, zz[1]), g.ins / 2))
    return dict(solid=s, tools=tools, bed=(0, -1, 0), fillet_axis=(1, 0, 0), fillet_r=2.0)


def part_front_bracket(b, d):
    g = b.g
    zc = pmm(d, 'frontMount_y')
    yf = pmm(d, 'frontMount_z')
    zt = g.z_top
    s = box(-25, 25, 116, yf - 3, zt, zt + 3)
    U(s, box(-25, 25, yf - 3, yf, zc - 12, zt + 3))
    up = (0, -1, 0)
    tools = []
    for x in (-13, 13):                               # HC-SR04 transducers (26 mm apart)
        tools.append(cyl((x, yf - 4, zc), (x, yf + 1, zc), 8.4))
    for x in (-21, 21):                               # PCB holes (42 x 17), M2 self-tapping
        for z in (zc - 8.5, zc + 8.5):
            tools.append(cyl((x, yf - 4, z), (x, yf + 1, z), 0.9))
    tools.append(box(-8, 8, yf - 4, yf + 1, zc + 10.5, zc + 20.5))  # ESP32 USB access
    for (x, y) in b.bracket_inserts:
        tools.append(hole((x, y, zt - 1), (x, y, zt + 4), g.clr, up))
    return dict(solid=s, tools=tools, bed=(0, 1, 0), fillet_axis=(1, 0, 0), fillet_r=3.0,
                fillet_skip=lambda p: p[2] < zt + 1.0)


def dovetail(z_bot, z_top, hw_bot, hw_top, y0, y1):
    s = box(-hw_bot, hw_bot, y0, y1, z_bot, z_top)
    for sx in (1, -1):
        p0 = (sx * hw_bot, 0, z_bot)
        dz, dx = z_top - z_bot, (hw_top - hw_bot)
        n = (sx * dz, 0, (hw_bot - hw_top))
        D(s, halfspace(p0, n))
    return s


QR = dict(z0=None, base_h=8.0, floor=2.0, hw_bot=14.0, hw_top=10.0, key_x=(4.0, 8.0), key_h=2.0,
          y_rear=-44.0, y_front=20.0, stop=4.0, lock=(24.0, -36.0))


def part_qr_receiver(b):
    g = b.g
    z0 = g.z_top
    zf = z0 + QR['floor']
    zt = z0 + QR['base_h']
    yr, yfr = QR['y_rear'], QR['y_front']
    s = box(-30, 30, yr, yfr, z0, zt)
    U(s, box(-26, 26, yr - 14, yr + 0.01, z0, zt))             # connector bay
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
    # J1 (JST-XH 8) and J2 (JST-XH 6) header pockets, open through to the pass-through
    tools.append(box(-24, -24 + 22.8, yr - 10.2, yr - 4.0, z0 - 1, zt + 1))
    tools.append(box(1.5, 1.5 + 17.8, yr - 10.2, yr - 4.0, z0 - 1, zt + 1))
    return dict(solid=s, tools=tools, bed=(0, 0, -1), fillet_axis=None)


def part_qr_adapter(b):
    """Example attachment adapter: dovetail rail with keyway + 70 x 62 plate
    with an M3-insert grid (20 mm pitch).  Prints plate-face down."""
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
    return dict(solid=rail, tools=tools, bed=(0, 0, 1), fillet_axis=None)


# ----------------------------------------------------------------------------
# Fusion plumbing
# ----------------------------------------------------------------------------
def edge_mid(e):
    ok, t0, t1 = e.evaluator.getParameterExtents()
    ok, p = e.evaluator.getPointAtParameter((t0 + t1) / 2)
    return p


def concave_edges(body, axis, skip=None):
    out = adsk.core.ObjectCollection.create()
    for e in body.edges:
        if e.faces.count != 2:
            continue
        geo = e.geometry
        if geo.objectType != adsk.core.Line3D.classType():
            continue
        d = geo.startPoint.vectorTo(geo.endPoint)
        if d.length < 0.15:
            continue
        d.normalize()
        if axis is not None and abs(d.x * axis[0] + d.y * axis[1] + d.z * axis[2]) < 0.99:
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
    body.name = name
    if comp.bRepBodies.count != 1:
        raise RuntimeError('%s has %d bodies (expected 1)' % (name, comp.bRepBodies.count))
    bb = body.boundingBox
    ext = [(bb.maxPoint.x - bb.minPoint.x) * 10, (bb.maxPoint.y - bb.minPoint.y) * 10, (bb.maxPoint.z - bb.minPoint.z) * 10]
    vol = body.volume  # cm3
    log.append(dict(part=name, bed=spec['bed'], extents_mm=[round(v, 1) for v in ext], volume_cm3=round(vol, 2),
                    fillets=nf, chamfer_edges=nc))
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
def build(out_dir=None):
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
    log = []
    printed = {}
    cache = {}

    def place(name, make, m, look):
        """First call creates the component (make() -> temp body or spec dict),
        later calls add another occurrence of the same component."""
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

    I3 = adsk.core.Matrix3D.create
    BLACK = ('Plastic - Matte (Black)', 'Plastic - Glossy (Black)')
    ORANGE = ('Plastic - Matte (Orange)', 'Plastic - Glossy (Orange)', 'Plastic - Matte (Red)')
    BLUE = ('Plastic - Matte (Blue)', 'Plastic - Glossy (Blue)')
    METAL = ('Aluminum - Satin', 'Steel - Satin')

    body_occs = []
    plates = {}
    for which in ('top', 'bottom'):
        for half in ('front', 'rear'):
            nm = 'Plate_%s_%s' % (which.capitalize(), half.capitalize())
            plates[nm] = place(nm, lambda w=which, h=half: B.plate(w, h), I3(), BLACK)
            body_occs.append(plates[nm])
    for nm, fn in (('BatteryTray', lambda: part_battery_tray(B)), ('ControllerCarrier', lambda: part_carrier(B)),
                   ('PowerPanel', lambda: part_power_panel(B)), ('FrontSensorBracket', lambda: part_front_bracket(B, design)),
                   ('QR_Receiver', lambda: part_qr_receiver(B))):
        body_occs.append(place(nm, fn, I3(), ORANGE))
    body_occs.append(place('QR_AdapterPlate_Example', lambda: part_qr_adapter(B), I3(), BLUE))
    body_occs.append(place('MG90S', lambda: servo_env(MG90S),
                           frame((0, -10, g.z_top + QR['base_h'] + 5 + MG90S['below_tab']), (1, 0, 0), (0, 0, 1)), BLUE))

    # purchased envelopes in the body
    for (x, y) in B.standoffs:
        m = I3()
        m.translation = adsk.core.Vector3D.create(cm(x), cm(y), 0)
        body_occs.append(place('Standoff_M3x45_FF', lambda: cyl((0, 0, -g.GAP / 2), (0, 0, g.GAP / 2), 2.75), m, METAL))
    zb, zt = -g.GAP / 2, g.GAP / 2
    body_occs.append(place('Battery_2S_2200', lambda: box(-17, 17, -125, -20, zb + 2.5, zb + 27.5), I3(),
                           ('Plastic - Matte (Yellow)', 'Plastic - Glossy (Yellow)')))
    body_occs.append(place('UBEC', lambda: box(-22.5, 22.5, -14, 6, zb + 0.5, zb + 16.5), I3(), BLUE))
    for k, (cx, cy) in enumerate(B.pca):
        body_occs.append(place('PCA9685_%d' % k, lambda cx=cx, cy=cy: box(cx - 12.7, cx + 12.7, cy - 31.25, cy + 31.25, zb + 4, zb + 5.6), I3(), BLUE))
    body_occs.append(place('Perfboard_50x70', lambda: box(-25, 25, 58, 128, zt - 9.6, zt - 8), I3(), ('Plastic - Matte (Green)',)))
    body_occs.append(place('ESP32_DevKitC', lambda: box(-14, 14, 72, 126, zt - 23.2, zt - 18.1), I3(), BLACK))
    zc, yf = pmm(design, 'frontMount_y'), pmm(design, 'frontMount_z')

    def sonar():
        b_ = box(-22.5, 22.5, yf - 4.6, yf - 3, zc - 10, zc + 10)
        for x in (-13, 13):
            U(b_, cyl((x, yf - 3, zc), (x, yf + 9, zc), 8.0))
        return b_
    body_occs.append(place('HC-SR04', sonar, I3(), METAL))
    body_occs.append(place('RockerSwitch_KCD1', lambda: box(28.6, 41.4, -132, -118, -4.5, 14.5), I3(), BLACK))

    # legs
    s = MG996R
    lines = {}
    joints = 0
    leg_groups = []
    for lg in B.legs:
        i = lg['i']
        dx, dy = lg['dir'][0], lg['dir'][1]
        coxa_m = frame((lg['X'], lg['Y'], 0), (dx, dy, 0), (0, 0, 1))
        cservo_m = frame((lg['X'], lg['Y'], g.z_top), (-dx, -dy, 0), (0, 0, 1))
        cs = place('MG996R', lambda: servo_env(s), cservo_m, BLACK)
        ch = place('Horn_25T', lambda: horn_env(s, g.pcd), cservo_m, METAL)
        cx = place('Coxa', lambda: part_coxa(g), coxa_m, ORANGE)
        cb = place('Bearing_623ZZ', bearing_623, chain(frame((0, 0, g.z_coxa_lower_top - 2), (1, 0, 0), (0, 0, 1)), coxa_m), METAL)
        fservo_l = frame((g.Lc, g.y_tab, 0), (0, 0, -1), (0, 1, 0))
        fs = place('MG996R', None, chain(fservo_l, coxa_m), BLACK)
        fh = place('Horn_25T', None, chain(fservo_l, coxa_m), METAL)
        femur_m = chain(frame((g.Lc, 0, 0), (1, 0, 0), (0, 0, 1)), coxa_m)
        fe = place('Femur', lambda: part_femur(g), femur_m, ORANGE)
        fb = place('Bearing_623ZZ', None, chain(frame((0, g.y_idler_in - 2, 0), (1, 0, 0), (0, 1, 0)), femur_m), METAL)
        tservo_l = frame((g.Lf, g.y_tab, 0), (-1, 0, 0), (0, 1, 0))
        ts = place('MG996R', None, chain(tservo_l, femur_m), BLACK)
        th = place('Horn_25T', None, chain(tservo_l, femur_m), METAL)
        tibia_m = chain(frame((g.Lf, 0, 0), (1, 0, 0), (0, 0, 1)), femur_m)
        ti = place('Tibia', lambda: part_tibia(g), tibia_m, ORANGE)
        tb = place('Bearing_623ZZ', None, chain(frame((0, g.y_idler_in - 2, 0), (1, 0, 0), (0, 1, 0)), tibia_m), METAL)
        bo = place('FootBoot_TPU', lambda: part_foot_boot(g), tibia_m, BLACK)
        if not lines:
            lines['Coxa'] = axis_line(cache['Coxa'], (0, 0, -40), (0, 0, 55))
            lines['Femur'] = axis_line(cache['Femur'], (0, -35, 0), (0, 35, 0))
            lines['Tibia'] = axis_line(cache['Tibia'], (0, -35, 0), (0, 35, 0))
        body_occs.append(cs)
        leg_groups.append((i, [cx, ch, cb, fs], [fe, fh, fb, ts], [ti, th, tb, bo]))

    plates['Plate_Top_Rear'].isGrounded = True
    rigid(root, body_occs, 'Body group')
    for (i, gc, gf, gt) in leg_groups:
        rigid(root, gc, 'Leg%d coxa group' % i)
        rigid(root, gf, 'Leg%d femur group' % i)
        rigid(root, gt, 'Leg%d tibia group' % i)
        lg = B.legs[i]
        fixed = plates['Plate_Top_Front'] if lg['Y'] > B.seam_top else plates['Plate_Top_Rear']
        # limits from the interference sweep (Fusion sign: +femur = down, +tibia = fold in)
        revolute(root, gc[0], fixed, lines['Coxa'], gc[0], -45, 45, 'Leg%d coxa' % i)
        revolute(root, gf[0], gc[0], lines['Femur'], gf[0], -90, 35, 'Leg%d femur' % i)
        revolute(root, gt[0], gf[0], lines['Tibia'], gt[0], -120, 45, 'Leg%d tibia' % i)
        joints += 3

    report = dict(parts=log, joints=joints, occurrences=root.allOccurrences.count,
                  geo={k: round(v, 2) for k, v in vars(g).items() if isinstance(v, float)})
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
        out.append(dict(part=nm, file=fn, bed=bed))
    return out
