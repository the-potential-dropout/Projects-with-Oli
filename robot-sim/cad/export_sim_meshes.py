"""Convert per-body STL exports from Fusion into GLB meshes in the simulator's frames.

Input: a folder of STLs named <group>__<component>__<n>.stl, exported from the
Fusion design with every joint at 0.  Fusion writes each body in its own
component's coordinates (mm), so refs.json also holds every occurrence's world
transform ("occ", keyed by file name) and the world transforms of leg 0's Coxa,
Femur and Tibia occurrences ("ref"), all as Fusion cm, row-major 4x4.

Output (robot-sim/cad/mesh/<group>.glb), units metres, +Y up:
  body, qr_receiver, qr_adapter_example, sonar_example
        origin = body centre (half way between the plates, femur-axis height),
        +Z forward, +X robot left.
  coxa  origin on the coxa axis at femur-axis height, leg along +X.
  femur origin on the femur axis, link along +X (zero angle = horizontal).
  tibia origin on the tibia axis, zero angle = straight along +X.
  coxa/femur/tibia: +Y up at zero angle; the servo horn side is -Z.

    python export_sim_meshes.py <raw_dir> <out_dir>
"""
import json
import math
import os
import struct
import sys

COLORS = {
    'Plate': (0.08, 0.08, 0.09), 'MG996R': (0.05, 0.05, 0.05), 'MG90S': (0.1, 0.25, 0.8),
    'Horn': (0.75, 0.75, 0.78), 'Standoff': (0.75, 0.75, 0.78), 'Battery': (0.95, 0.75, 0.1),
    'UBEC': (0.1, 0.25, 0.8), 'PCA9685': (0.1, 0.25, 0.8), 'Perfboard': (0.1, 0.5, 0.2),
    'ESP32': (0.05, 0.05, 0.05), 'HC-SR04': (0.75, 0.75, 0.78), 'Rocker': (0.05, 0.05, 0.05),
    'FootBoot': (0.05, 0.05, 0.05), 'QR_Adapter': (0.15, 0.3, 0.85),
}
PRINTED = (0.85, 0.3, 0.12)


def color(comp):
    for k, c in COLORS.items():
        if comp.startswith(k):
            return c
    return PRINTED


def read_stl(path):
    data = open(path, 'rb').read()
    n = struct.unpack_from('<I', data, 80)[0]
    return [struct.unpack_from('<12f', data, 84 + i * 50)[3:] for i in range(n)]


def fwd_rigid(m):
    """m: 16 floats row-major (cm). Returns function local_mm -> world_mm."""
    def f(p):
        return tuple(m[4 * r] * p[0] + m[4 * r + 1] * p[1] + m[4 * r + 2] * p[2] + m[4 * r + 3] * 10 for r in range(3))
    return f


def inv_rigid(m):
    """m: 16 floats row-major (cm). Returns function world_mm -> local_mm."""
    R = [[m[0], m[1], m[2]], [m[4], m[5], m[6]], [m[8], m[9], m[10]]]
    t = [m[3] * 10, m[7] * 10, m[11] * 10]

    def f(p):
        d = [p[k] - t[k] for k in range(3)]
        return tuple(sum(R[r][c] * d[r] for r in range(3)) for c in range(3))
    return f


def frame_fn(group, refs):
    if group in ('body', 'qr_receiver', 'qr_adapter_example', 'sonar_example'):
        return lambda p: (-p[0] / 1000, p[2] / 1000, p[1] / 1000)
    loc = inv_rigid(refs[group])
    if group in ('coxa', 'femur'):
        return lambda p: (lambda q: (q[0] / 1000, q[2] / 1000, -q[1] / 1000))(loc(p))
    if group == 'tibia':
        return lambda p: (lambda q: (-q[2] / 1000, q[0] / 1000, -q[1] / 1000))(loc(p))
    raise KeyError(group)


def write_glb(path, prims):
    """prims: list of (name, rgb, [triangles of 3 xyz])."""
    bin_ = bytearray()
    accessors, views, meshes_prims, materials = [], [], [], []
    for name, rgb, tris in prims:
        pos, nrm = [], []
        for a, b, c in tris:
            u = [b[k] - a[k] for k in range(3)]
            w = [c[k] - a[k] for k in range(3)]
            n = [u[1] * w[2] - u[2] * w[1], u[2] * w[0] - u[0] * w[2], u[0] * w[1] - u[1] * w[0]]
            ln = math.sqrt(sum(x * x for x in n)) or 1.0
            n = [x / ln for x in n]
            for p in (a, b, c):
                pos.append(p)
                nrm.append(n)
        for arr, kind in ((pos, 'POSITION'), (nrm, 'NORMAL')):
            off = len(bin_)
            for v in arr:
                bin_ += struct.pack('<3f', *v)
            views.append({'buffer': 0, 'byteOffset': off, 'byteLength': len(arr) * 12, 'target': 34962})
            acc = {'bufferView': len(views) - 1, 'componentType': 5126, 'count': len(arr), 'type': 'VEC3'}
            if kind == 'POSITION':
                acc['min'] = [min(v[k] for v in arr) for k in range(3)]
                acc['max'] = [max(v[k] for v in arr) for k in range(3)]
            accessors.append(acc)
        materials.append({'name': name, 'pbrMetallicRoughness': {'baseColorFactor': list(rgb) + [1.0],
                                                                  'metallicFactor': 0.1, 'roughnessFactor': 0.7}})
        meshes_prims.append({'attributes': {'POSITION': len(accessors) - 2, 'NORMAL': len(accessors) - 1},
                             'material': len(materials) - 1})
    while len(bin_) % 4:
        bin_ += b'\0'
    gltf = {'asset': {'version': '2.0', 'generator': 'Hexapod Mk1 export_sim_meshes.py'},
            'scene': 0, 'scenes': [{'nodes': [0]}],
            'nodes': [{'name': os.path.splitext(os.path.basename(path))[0], 'mesh': 0}],
            'meshes': [{'primitives': meshes_prims}], 'materials': materials,
            'accessors': accessors, 'bufferViews': views, 'buffers': [{'byteLength': len(bin_)}]}
    js = json.dumps(gltf, separators=(',', ':')).encode()
    while len(js) % 4:
        js += b' '
    with open(path, 'wb') as f:
        f.write(struct.pack('<III', 0x46546C67, 2, 12 + 8 + len(js) + 8 + len(bin_)))
        f.write(struct.pack('<II', len(js), 0x4E4F534A) + js)
        f.write(struct.pack('<II', len(bin_), 0x004E4942) + bytes(bin_))


def main(raw, out):
    d = json.load(open(os.path.join(raw, 'refs.json')))
    refs, occ = d['ref'], d['occ']
    groups = {}
    for fn in sorted(os.listdir(raw)):
        if fn.endswith('.stl'):
            g, comp, _ = fn[:-4].split('__')
            groups.setdefault(g, []).append((comp, os.path.join(raw, fn), fwd_rigid(occ[fn])))
    os.makedirs(out, exist_ok=True)
    summary = {}
    for g, items in groups.items():
        f = frame_fn(g, refs)
        prims, ntri = [], 0
        for comp, path, w in items:
            tris = [(f(w(t[0:3])), f(w(t[3:6])), f(w(t[6:9]))) for t in read_stl(path)]
            ntri += len(tris)
            prims.append((comp, color(comp), tris))
        write_glb(os.path.join(out, g + '.glb'), prims)
        pts = [p for _, _, ts in prims for t in ts for p in t]
        bb = [[round(min(p[k] for p in pts), 4) for k in range(3)], [round(max(p[k] for p in pts), 4) for k in range(3)]]
        summary[g] = {'triangles': ntri, 'bbox_m': bb, 'parts': sorted({c for c, _, _ in items})}
        print('%-20s %6d tris  bbox %s' % (g, ntri, bb))
    json.dump(summary, open(os.path.join(out, 'meshes.json'), 'w'), indent=1)


if __name__ == '__main__':
    main(sys.argv[1], sys.argv[2])
