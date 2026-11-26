import adsk.core, adsk.fusion, math, json, traceback

OUT = r'C:\Users\admin\AppData\Local\Temp\hex_sweep.json'


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
        k = '/'.join(sorted((top_name(r.entityOne), top_name(r.entityTwo))))
        d[k] = round(d.get(k, 0) + r.interferenceBody.volume * 1000, 1)
    return d


def main(mode):
    app = adsk.core.Application.get()
    design = adsk.fusion.Design.cast(app.activeProduct)
    root = design.rootComponent
    out = {}
    if mode == 'neutral':
        occs = adsk.core.ObjectCollection.create()
        for o in root.occurrences:
            occs.add(o)
        out['neutral'] = hits(design, occs)
    else:
        J = {j.name: j for j in root.asBuiltJoints}
        for j in J.values():
            l = j.jointMotion.rotationLimits
            l.isMinimumValueEnabled = False
            l.isMaximumValueEnabled = False
        occs = adsk.core.ObjectCollection.create()
        legs = mode.split(',')
        for rg in root.rigidGroups:
            if any(rg.name.startswith('Leg%s ' % L) for L in legs) or rg.name == 'Body group':
                for o in rg.occurrences:
                    occs.add(o)
        poses = []
        for c in (-45, 45):
            poses.append((c, 0, 0))
        for f in (-70, -30, 10):
            poses.append((0, f, 0))
        for t in (-120, -60, 30, 45):
            poses.append((0, 0, t))
        for (f, t) in ((-60, 45), (-30, 45), (30, -60), (10, 45), (-70, 45), (-70, -60), (10, -120)):
            poses.append((0, f, t))
        for (c, f, t) in poses:
            for L in legs:
                J['Leg%s coxa' % L].jointMotion.rotationValue = math.radians(c)
                J['Leg%s femur' % L].jointMotion.rotationValue = math.radians(f)
                J['Leg%s tibia' % L].jointMotion.rotationValue = math.radians(t)
            out['c%d f%d t%d' % (c, f, t)] = hits(design, occs)
        for L in legs:
            for n in ('coxa', 'femur', 'tibia'):
                J['Leg%s %s' % (L, n)].jointMotion.rotationValue = 0
    json.dump(out, open(OUT, 'w'), indent=1)


def run(_context: str):
    try:
        main(MODE)
    except Exception:
        json.dump({'error': traceback.format_exc()}, open(OUT, 'w'))
        raise
