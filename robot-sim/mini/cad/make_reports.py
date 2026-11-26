"""Refresh the generated tables in the Hexapod Mini docs from the CAD outputs.

    python robot-sim/mini/cad/make_reports.py [sweep_dir]

- merges the sweep results (sweep_<mode>.json from interference_sweep_mini.py) into
  cad/interference_report.json, when a sweep_dir is given;
- rewrites the blocks between <!-- gen:NAME --> and <!-- /gen:NAME --> in
  docs/DESIGN.md and docs/WIRING.md (or a {{NAME}} placeholder the first time):
  MASS_TOTAL, MASS_TABLE, LOAD_TABLE, SWEEP_TABLE, PARTS_TABLE, WIRE_TABLE.
Standard library only.
"""
import glob
import json
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
MINI = os.path.dirname(HERE)
LONG = '\\\\?\\'


def _L(p):
    p = os.path.abspath(p)
    return LONG + p if os.name == 'nt' and not p.startswith(LONG) else p


def rd(p):
    return json.load(open(_L(p), encoding='utf-8'))


def mass_table(rep):
    m = rep['mass_g']
    rows = ['*(generated from cad/build_report.json)*', '',
            '| Printed parts | Bought parts | Screws | Wiring | **Total** |', '| --- | --- | --- | --- | --- |',
            '| %d g | %d g | %d g | %d g | **%d g** |' % (m['printed'], m['bought'], m['hardware'], m['wiring'], m['total']), '',
            '| Printed part | Qty | Volume (cm³) | Est. mass each (g) |', '| --- | --- | --- | --- |']
    for p in rep['parts']:
        rows.append('| %s | %d | %.1f | %.1f |' % (p['part'], p['qty'], p['volume_cm3'], p['est_g_each']))
    return '\n'.join(rows)


def load_table(mass_kg):
    sys.path.insert(0, HERE)
    import io
    import contextlib
    import load_calc_mini
    with contextlib.redirect_stdout(io.StringIO()):
        res = load_calc_mini.main(mass_kg)
    rows = ['*(generated from cad/load_calc_mini.py, robot %.2f kg, MG90S datasheet torque %.2f kg·cm at 5.0 V)*' % (res['mass_kg'], res['rated_kgcm']), '',
            '| Case | Foot load (N) | Stance window (mm) | Coxa: required / margin | Femur: required / margin | Tibia: required / margin |',
            '| --- | --- | --- | --- | --- | --- |']
    for r in res['rows']:
        cell = lambda j: '%.2f kg·cm / **%.2f×**' % (r[j]['required_kgcm'], r[j]['margin'])
        rows.append('| %s | %.2f | %d–%d | %s | %s | %s |' % (r['case'], r['F_foot_N'], r['window_mm'][0], r['window_mm'][1], cell('coxa'), cell('femur'), cell('tibia')))
    return '\n'.join(rows), res


def sweep_table(rep):
    total = clean = 0
    bad = []
    for mode, poses in rep.items():
        for pose, hits in poses.items():
            total += 1
            if hits:
                bad.append((mode, pose, hits))
            else:
                clean += 1
    rows = ['*(generated from cad/interference_report.json)*', '',
            '**%d of %d poses clean** (Fusion interference analysis, intended contacts filtered).' % (clean, total), '',
            '| Mode | Poses | Clean |', '| --- | --- | --- |']
    for mode, poses in rep.items():
        rows.append('| %s | %d | %d |' % ({'neutral': 'modelled pose', 'pairs': 'neighbouring legs at the 25° coxa limit'}.get(mode, 'leg %s alone' % mode),
                                         len(poses), sum(1 for h in poses.values() if not h)))
    for mode, pose, hits in bad:
        rows.append('')
        rows.append('- %s, %s: %s' % (mode, pose, ', '.join('%s %.1f mm³' % kv for kv in hits.items())))
    return '\n'.join(rows)


def parts_table(rep, sizes):
    sz = {s['part']: s for s in sizes}
    rows = ['*(generated from cad/build_report.json and cad/stl/print_sizes.json)*', '',
            '| Part | Qty | Bed face | Printed size (mm) | Fillets | Bed-chamfer edges | Edge breaks | 180 bed |',
            '| --- | --- | --- | --- | --- | --- | --- | --- |']
    for p in rep['parts']:
        s = sz.get(p['part'], {})
        dims = '%.0f × %.0f × %.0f' % (s.get('x', 0), s.get('y', 0), s.get('z', 0)) if s else ''
        rows.append('| %s | %d | %s | %s | %d | %d | %d | %s |' % (p['part'], p['qty'], tuple(p['bed']), dims, p['fillets'], p['chamfer_edges'],
                                                             p['edge_breaks'], '✓' if s.get('fits_180') else '✗'))
    return '\n'.join(rows)


WIRE_DESC = [
    (r'^Leg(\d) (coxa|femur|tibia) lead', lambda m: ('MG90S leg %s %s → PCA9685 channel (WIRING §2)' % (m.group(1), m.group(2)), 'servo lead (3-core)')),
    (r'^B1', lambda m: ('holder + → BMS B+', '20 AWG red')),
    (r'^B2', lambda m: ('holder − → BMS B−', '20 AWG black')),
    (r'^B3', lambda m: ('holder mid-tap → BMS BM', '20 AWG')),
    (r'^F1', lambda m: ('BMS B+ (P+) → fuse', '20 AWG red')),
    (r'^F2', lambda m: ('fuse → switch', '20 AWG red')),
    (r'^F3', lambda m: ('switch → Mini560 R IN+', '20 AWG red')),
    (r'^F4', lambda m: ('Mini560 R IN+ → Mini560 L IN+', '20 AWG red')),
    (r'^G1', lambda m: ('BMS P− → Mini560 R IN−', '20 AWG black')),
    (r'^G2', lambda m: ('Mini560 R IN− → Mini560 L IN−', '20 AWG black')),
    (r'^O(R|L) buck \w OUT\+', lambda m: ('Mini560 %s OUT+ → PCA9685 %s V+' % (m.group(1), m.group(1)), '20 AWG red')),
    (r'^O(R|L) buck \w OUT-', lambda m: ('Mini560 %s OUT− → PCA9685 %s GND' % (m.group(1), m.group(1)), '20 AWG black')),
    (r'^P1a', lambda m: ('PCA9685 R V+ terminal → 1N5822 anode', '26 AWG Dupont')),
    (r'^P1b', lambda m: ('1N5822 cathode → ESP32 5V', '26 AWG Dupont')),
    (r'^E (\w+)', lambda m: ('ESP32 → PCA9685 R front header %s' % m.group(1), '26 AWG Dupont')),
    (r'^C (\w+)', lambda m: ('PCA9685 R rear header → PCA9685 L rear header %s' % m.group(1), '26 AWG Dupont')),
    (r'^S1', lambda m: ('BMS P+ → divider (100 kΩ)', '26 AWG')),
    (r'^S2', lambda m: ('divider mid → ESP32 GPIO34', '26 AWG Dupont')),
    (r'^S3', lambda m: ('divider foot (47 kΩ) → ESP32 GND', '26 AWG Dupont')),
]


def wire_table(rep):
    total = {}
    order = []
    for h in rep['harness']:
        base = re.sub(r' \((body|coxa|femur)\)$', '', h['cable'])
        if base not in total:
            order.append(base)
            total[base] = 0
        total[base] += h['length_mm']
    rows = ['| CAD name | From → to | Wire | Length (CAD route) |', '| --- | --- | --- | --- |']
    for base in order:
        desc, wire = base, ''
        for pat, f in WIRE_DESC:
            m = re.match(pat, base)
            if m:
                desc, wire = f(m)
                break
        rows.append('| %s | %s | %s | %d mm |' % (base, desc, wire, total[base]))
    rows.append('')
    rows.append('*(generated from cad/build_report.json: %d wires; the servo leads are the leads on the servos, the lengths show what each lead must reach - '
                'the MG90S lead is about 250 mm)*' % len(order))
    return '\n'.join(rows)


def put(text, name, block):
    pat = re.compile(r'<!-- gen:%s -->.*?<!-- /gen:%s -->' % (name, name), re.S)
    new = '<!-- gen:%s -->\n%s\n<!-- /gen:%s -->' % (name, block, name)
    if pat.search(text):
        return pat.sub(lambda m: new, text)
    return text.replace('{{%s}}' % name, new)


def main(sweep_dir=None):
    rep = rd(os.path.join(HERE, 'build_report.json'))
    if sweep_dir:
        out = {}
        for mode in ['neutral', 'pairs', '0', '1', '2', '3', '4', '5']:
            f = os.path.join(sweep_dir, 'sweep_%s.json' % mode)
            out[mode] = json.load(open(f)) if os.path.exists(f) else {}
        json.dump(out, open(_L(os.path.join(HERE, 'interference_report.json')), 'w'), indent=1)
    sweep = rd(os.path.join(HERE, 'interference_report.json'))
    sizes = rd(os.path.join(HERE, 'stl', 'print_sizes.json'))
    lt, res = load_table(rep['mass_g']['total'] / 1000.0)
    d = os.path.join(MINI, 'docs', 'DESIGN.md')
    t = open(_L(d), encoding='utf-8').read()
    t = t.replace('{{MASS_TOTAL}}', str(int(round(rep['mass_g']['total'], -1))))
    for name, block in (('MASS_TABLE', mass_table(rep)), ('LOAD_TABLE', lt), ('SWEEP_TABLE', sweep_table(sweep)), ('PARTS_TABLE', parts_table(rep, sizes))):
        t = put(t, name, block)
    open(_L(d), 'w', encoding='utf-8').write(t)
    w = os.path.join(MINI, 'docs', 'WIRING.md')
    t = open(_L(w), encoding='utf-8').read()
    t = put(t, 'WIRE_TABLE', wire_table(rep))
    open(_L(w), 'w', encoding='utf-8').write(t)
    print('mass %d g; femur margin (lone middle foot) %.2fx' % (rep['mass_g']['total'], res['rows'][0]['femur']['margin']))


if __name__ == '__main__':
    main(*sys.argv[1:2])
