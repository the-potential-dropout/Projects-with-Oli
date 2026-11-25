"""Put Fusion STL exports into print orientation and check bed fit.

Fusion exports each part in its component frame.  build_report.json /
export_manifest.json list the face that goes on the bed ("bed" = outward normal
of that face).  This script rotates each STL so that face points down (-Z),
drops it onto Z = 0, centres it on X/Y, and reports the size, volume and
whether it fits a 220 x 220 x 250 mm and a 180 x 180 x 180 mm bed.

    python orient_stl.py <raw_dir> <out_dir> <export_manifest.json>

Pure Python, no numpy.
"""
import json
import math
import os
import struct
import sys

ROT = {  # bed normal -> rotation that sends it to (0, 0, -1)
    (0, 0, -1): lambda x, y, z: (x, y, z),
    (0, 0, 1): lambda x, y, z: (x, -y, -z),
    (0, -1, 0): lambda x, y, z: (x, -z, y),
    (0, 1, 0): lambda x, y, z: (x, z, -y),
    (-1, 0, 0): lambda x, y, z: (-z, y, x),
    (1, 0, 0): lambda x, y, z: (z, y, -x),
}


def read_stl(path):
    with open(path, 'rb') as f:
        data = f.read()
    n = struct.unpack_from('<I', data, 80)[0]
    if 84 + n * 50 != len(data):
        raise ValueError('%s is not a binary STL' % path)
    tris = []
    for i in range(n):
        v = struct.unpack_from('<12f', data, 84 + i * 50)
        tris.append((v[3:6], v[6:9], v[9:12]))
    return tris


def write_stl(path, tris, name):
    with open(path, 'wb') as f:
        f.write(('Hexapod Mk1 ' + name)[:80].ljust(80).encode('ascii'))
        f.write(struct.pack('<I', len(tris)))
        for a, b, c in tris:
            u = [b[k] - a[k] for k in range(3)]
            w = [c[k] - a[k] for k in range(3)]
            n = [u[1] * w[2] - u[2] * w[1], u[2] * w[0] - u[0] * w[2], u[0] * w[1] - u[1] * w[0]]
            ln = math.sqrt(sum(x * x for x in n)) or 1.0
            f.write(struct.pack('<12fH', *(x / ln for x in n), *a, *b, *c, 0))


def main(raw_dir, out_dir, manifest):
    items = json.load(open(manifest))
    rows = []
    for it in items:
        name = it['part']
        tris = read_stl(os.path.join(raw_dir, name + '.stl'))
        rot = ROT[tuple(int(round(v)) for v in it['bed'])]
        tris = [tuple(rot(*p) for p in t) for t in tris]
        xs = [p[0] for t in tris for p in t]
        ys = [p[1] for t in tris for p in t]
        zs = [p[2] for t in tris for p in t]
        cx, cy, z0 = (min(xs) + max(xs)) / 2, (min(ys) + max(ys)) / 2, min(zs)
        tris = [tuple((p[0] - cx, p[1] - cy, p[2] - z0) for p in t) for t in tris]
        sx, sy, sz = max(xs) - min(xs), max(ys) - min(ys), max(zs) - min(zs)
        vol = 0.0
        for a, b, c in tris:
            vol += (a[0] * (b[1] * c[2] - b[2] * c[1]) - a[1] * (b[0] * c[2] - b[2] * c[0]) + a[2] * (b[0] * c[1] - b[1] * c[0])) / 6.0
        write_stl(os.path.join(out_dir, name + '.stl'), tris, name)
        fits = lambda bx, by, bz: (max(sx, sy) <= bx and min(sx, sy) <= by and sz <= bz)
        rows.append(dict(part=name, x=round(sx, 1), y=round(sy, 1), z=round(sz, 1), volume_cm3=round(abs(vol) / 1000, 2),
                         fits_220=fits(220, 220, 250), fits_180=fits(180, 180, 180), triangles=len(tris)))
    for r in rows:
        print('%-26s %6.1f x %6.1f x %6.1f mm  %7.2f cm3  220:%s  180:%s' % (
            r['part'], r['x'], r['y'], r['z'], r['volume_cm3'], 'ok' if r['fits_220'] else 'NO', 'ok' if r['fits_180'] else 'NO'))
    json.dump(rows, open(os.path.join(out_dir, 'print_sizes.json'), 'w'), indent=1)


if __name__ == '__main__':
    main(*sys.argv[1:4])
