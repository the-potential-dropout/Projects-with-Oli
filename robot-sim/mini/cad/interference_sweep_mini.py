"""Hexapod Mini interference sweep (run inside Fusion, after mini_hexapod_fusion.build()).

Uses Fusion's own interference analysis (Design.analyzeInterference).  For each
mode it moves the legs through the poses below and records every pair of
components that overlap, with the overlap volume in mm^3:

  MODE = 'neutral'        everything, modelled pose
  MODE = '0' ... '5'      one leg (plus the body) through POSES
  MODE = 'pairs'          neighbouring legs swung towards each other to the firmware coxa limit (25 deg)

Run it with MODE set (the MCP wrapper sets it), it writes OUT as JSON.  The
intended contacts (a cable end in its plug or on its terminal, the splice of one
cable at a joint) are filtered by intended() here and in the notebook.
Fusion joint values: coxa + = CCW from above, femur + = down, tibia + = fold in.
"""
import adsk.core
import adsk.fusion
import math
import json
import re

OUT = r'C:\Users\admin\AppData\Local\Temp\hex_mini_sweep.json'
MODE = 'neutral'

# (coxa, femur, tibia) in Fusion degrees.  Limits: coxa +-25 (body clear to +-40), femur -55..35, tibia -50..40.
POSES = [(-40, 0, 0), (40, 0, 0),
         (0, -55, 0), (0, -25, 0), (0, 20, 0), (0, 35, 0),
         (0, 0, -50), (0, 0, 25), (0, 0, 40),
         (0, -55, 40), (0, -55, -50), (0, -30, 40), (0, 35, 40), (0, 35, -50), (0, 20, 25), (0, 10, 40),
         (-40, -55, 40), (40, -55, 40), (-40, 35, -50), (40, 35, -50)]
PAIRS = [((0, 1), (-25, 25)), ((1, 2), (-25, 25)), ((3, 4), (25, -25)), ((4, 5), (25, -25)),
         ((0, 3), (25, -25)), ((2, 5), (-25, 25))]

BOUGHT_TERMINALS = ('MG90S', 'PCA9685', 'ESP32', 'Mini560', 'BMS_2S', 'Holder_2x18650', 'RockerSwitch', 'FuseHolder', 'Divider', 'Diode')


def family(n):
    return re.sub(r' \((body|coxa|femur)\)$', '', n)


def intended(a, b):
    names = (a, b)
    if a.startswith('Harness') and b.startswith('Harness') and family(a) == family(b):
        return True
    if any(n.startswith('Plug') for n in names) and any(n.startswith('Harness') for n in names):
        return True
    if any(n.startswith('Harness') for n in names) and any(n.startswith(BOUGHT_TERMINALS) for n in names):
        return True
    if any(n.startswith('Plug') for n in names) and any(n.startswith(('PCA9685', 'ESP32')) for n in names):
        return True
    return False


def top_name(e):
    o = getattr(e, 'assemblyContext', None)
    if o is None:
        return getattr(e, 'name', '?')
    while o.assemblyContext is not None:
        o = o.assemblyContext
    return o.component.name


def hits(design, occs):
    res = design.analyzeInterference(design.createInterferenceInput(occs))
    d = {}
    for r in res:
        a, b = sorted((top_name(r.entityOne), top_name(r.entityTwo)))
        v = r.interferenceBody.volume * 1000
        if v < 0.05 or intended(a, b):
            continue
        k = a + '/' + b
        d[k] = round(d.get(k, 0) + v, 2)
    return d


def group_occs(root, legs):
    occs = adsk.core.ObjectCollection.create()
    for rg in root.rigidGroups:
        if rg.name == 'Body group' or any(rg.name.startswith('Leg%d ' % L) for L in legs):
            for o in rg.occurrences:
                occs.add(o)
    return occs


def set_leg(J, L, c, f, t):
    J['Leg%d coxa' % L].jointMotion.rotationValue = math.radians(c)
    J['Leg%d femur' % L].jointMotion.rotationValue = math.radians(f)
    J['Leg%d tibia' % L].jointMotion.rotationValue = math.radians(t)


def main(mode):
    app = adsk.core.Application.get()
    design = adsk.fusion.Design.cast(app.activeProduct)
    root = design.rootComponent
    J = {j.name: j for j in root.asBuiltJoints}
    for j in J.values():                       # sweep past the firmware limits
        lim = j.jointMotion.rotationLimits
        lim.isMinimumValueEnabled = False
        lim.isMaximumValueEnabled = False
    out = {}
    if mode == 'neutral':
        occs = adsk.core.ObjectCollection.create()
        for o in root.occurrences:
            occs.add(o)
        out['neutral'] = hits(design, occs)
    elif mode == 'pairs':
        for (a, b), (ca, cb) in PAIRS:
            occs = group_occs(root, (a, b))
            for fem, tib in ((0, 0), (-55, 40), (35, -50)):
                set_leg(J, a, ca, fem, tib)
                set_leg(J, b, cb, fem, tib)
                out['legs %d+%d c%d/%d f%d t%d' % (a, b, ca, cb, fem, tib)] = hits(design, occs)
            set_leg(J, a, 0, 0, 0)
            set_leg(J, b, 0, 0, 0)
    else:
        L = int(mode)
        occs = group_occs(root, (L,))
        for (c, f, t) in POSES:
            set_leg(J, L, c, f, t)
            out['c%d f%d t%d' % (c, f, t)] = hits(design, occs)
        set_leg(J, L, 0, 0, 0)
    return out


def run(_context: str):
    out = main(MODE)
    json.dump(out, open(OUT, 'w'), indent=1)
    print(json.dumps(out, indent=0)[:6000])
