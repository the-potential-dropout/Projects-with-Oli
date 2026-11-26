"""Compile the Hexapod Mk1 engineering design notebook into one HTML file.

    python robot-sim/docs/notebook/build_notebook.py [out.html]

- Entries: docs/notebook/entries/*.md, in file-name order (plain Markdown).
- Images (![alt](path), relative to the entry) are embedded as base64, so the
  output is a single self-contained file you can e-mail or print to PDF.
- Generated sections are filled from the current CAD outputs, so the numbers
  always match the repo:
    {{loads}}         servo torque table   (cad/load_calc.py)
    {{mass}}          mass breakdown       (cad/build_report.json)
    {{fillets}}       per-part fillets / chamfers / bed fit (build_report + stl/print_sizes.json)
    {{harness}}       cable list with lengths (build_report.json)
    {{interference}}  collision-sweep summary (cad/interference_report.json)
  and a final "Current design" gallery of docs/img/*.png.
- Standard library only.
"""
import base64
import html
import json
import os
import re
import sys
import datetime

HERE = os.path.dirname(os.path.abspath(__file__))
DOCS = os.path.dirname(HERE)
SIM = os.path.dirname(DOCS)
CAD = os.path.join(SIM, 'cad')


LONG = '\\\\?\\'

def _L(path):
    """Long-path-safe name on Windows (the repo can sit deeper than 260 chars)."""
    path = os.path.abspath(path)
    if os.name == 'nt' and not path.startswith(LONG):
        return LONG + path
    return path


def _read_json(*p):
    path = os.path.join(*p)
    return json.load(open(_L(path), encoding='utf-8')) if os.path.exists(_L(path)) else None


# ---------------------------------------------------------------------------
# generated sections (return Markdown)
# ---------------------------------------------------------------------------
def gen_loads():
    sys.path.insert(0, CAD)
    import load_calc
    import io
    import contextlib
    with contextlib.redirect_stdout(io.StringIO()):
        rows = load_calc.main()
    out = ['*(generated from cad/load_calc.py)*', '',
           '| Case | Joint | Servo | Required (kg·cm) | Rated | Margin vs rated | Margin vs stall |',
           '| --- | --- | --- | --- | --- | --- | --- |']
    for case, r in rows.items():
        for j in ('coxa', 'femur', 'tibia'):
            v = r[j]
            out.append('| %s | %s | %s | %.2f | %g | **%.2f×** | %.2f× |' % (
                case if j == 'coxa' else '', j, v['servo'], v['required_kgcm'], v['rated_kgcm'], v['margin_rated'], v['margin_stall']))
    return '\n'.join(out)


def gen_mass():
    r = _read_json(CAD, 'build_report.json')
    if not r:
        return '*(build_report.json missing: run the generator)*'
    m = r['mass_g']
    return '\n'.join(['*(generated from cad/build_report.json)*', '',
                      '| Printed | Bought parts | Hardware | Wiring | **Total** |', '| --- | --- | --- | --- | --- |',
                      '| %d g | %d g | %d g | %d g | **%d g** |' % (m['printed'], m['bought'], m['hardware'], m['wiring'], m['total'])])


def gen_fillets():
    r = _read_json(CAD, 'build_report.json')
    sizes = {p['part']: p for p in (_read_json(CAD, 'stl', 'print_sizes.json') or [])}
    if not r:
        return ''
    out = ['*(generated from cad/build_report.json and cad/stl/print_sizes.json)*', '',
           '| Part | Qty | Inside-corner fillets | Bed chamfer edges | Edge breaks | Printed size (mm) | 220 bed | 180 bed |',
           '| --- | --- | --- | --- | --- | --- | --- | --- |']
    for p in r['parts']:
        s = sizes.get(p['part'], {})
        dims = '%.0f × %.0f × %.0f' % (s.get('x', 0), s.get('y', 0), s.get('z', 0)) if s else ''
        out.append('| %s | %s | %d | %d | %d | %s | %s | %s |' % (
            p['part'], p.get('qty', 1), p['fillets'], p['chamfer_edges'], p.get('edge_breaks', 0), dims,
            '✓' if s.get('fits_220') else '✗', '✓' if s.get('fits_180') else '✗'))
    return '\n'.join(out)


def gen_harness():
    r = _read_json(CAD, 'build_report.json')
    if not r or 'harness' not in r:
        return ''
    out = ['*(generated from cad/build_report.json; add about 15 % for bends and service loops)*', '',
           '| Cable | Ø (mm) | Routed length (mm) |', '| --- | --- | --- |']
    for h in r['harness']:
        out.append('| %s | %.1f | %d |' % (h['cable'], h['d_mm'], h['length_mm']))
    return '\n'.join(out)


def _intended(pair):
    """Contacts that are meant to be there: cable ends in their plugs or on their board
    terminals, splices of one cable at a joint axis, and the brain module vs the turret
    example (alternative fits of the same top mount)."""
    a, b = pair.split('/')
    fam = lambda n: re.sub(r' (?:[A-D]|plate \d)( \(mirror side\))?$', '', n)
    if a.startswith('Harness') and b.startswith('Harness') and fam(a) == fam(b):
        return True
    names = (a, b)
    if any(n.startswith('Plug') for n in names) and any(n.startswith('Harness') for n in names):
        return True
    brain = lambda n: n.startswith('Brain') or n in ('RaspberryPi5', 'OAK-D-Lite', 'LD19_LiDAR', 'PololuD36V50F5') or         n.startswith(('Harness Brain', 'Harness OAK', 'Harness LD19'))
    if any(brain(n) for n in names) and any(n in ('QR_AdapterPlate_Example', 'STS3032') for n in names):
        return True
    boards = ('BusHub', 'CoxaBusHub', 'ServoDriver', 'PololuD36', 'HC-SR04', 'XT60', 'FuseHolder', 'BalanceSocket', 'PololuACS',
              'PololuBig', 'Feather', 'BNO085', 'VL53L1X', 'ELRS', 'RaspberryPi5', 'LD19', 'OAK-D-Lite', 'FootSwitch')
    return any(n.startswith('Harness') for n in names) and any(x in pair for x in boards)


def gen_interference():
    r = _read_json(CAD, 'interference_report.json')
    if not r:
        return '*(interference_report.json missing)*'
    out = ['*(generated from cad/interference_report.json: intended contacts filtered out, i.e. cable ends in their plugs or on their board terminals, and splices of one cable)*', '',
           '| Legs moved | Pose (Fusion: c coxa, f femur +down, t tibia +fold) | Result |', '| --- | --- | --- |']
    clean = total = 0
    for mode, poses in r.items():
        for pose, hits in poses.items():
            total += 1
            bad = {k: v for k, v in hits.items() if v > 0.5 and not _intended(k)}
            if bad:
                out.append('| %s | %s | %s |' % (mode, pose, ', '.join('%s %.0f mm³' % (k, v) for k, v in bad.items())))
            else:
                clean += 1
    out.insert(2, '**%d of %d poses clean.** Poses that are not clean are listed below; the femur 10° down + tibia 45° fold corner is outside the coupled limit.' % (clean, total))
    out.insert(3, '')
    return '\n'.join(out)


GENERATORS = {'loads': gen_loads, 'mass': gen_mass, 'fillets': gen_fillets, 'harness': gen_harness, 'interference': gen_interference}


# ---------------------------------------------------------------------------
# tiny Markdown -> HTML
# ---------------------------------------------------------------------------
def _img(src, alt, base):
    path = os.path.normpath(os.path.join(base, src))
    if not os.path.exists(_L(path)):
        return '<p class="missing">[missing image: %s]</p>' % html.escape(src)
    data = base64.b64encode(open(_L(path), 'rb').read()).decode()
    mime = 'image/png' if path.lower().endswith('.png') else 'image/jpeg'
    return '<figure><img alt="%s" src="data:%s;base64,%s"><figcaption>%s</figcaption></figure>' % (
        html.escape(alt), mime, data, html.escape(alt))


def _inline(t):
    t = html.escape(t, quote=False)
    t = re.sub(r'`([^`]+)`', r'<code>\1</code>', t)
    t = re.sub(r'\*\*([^*]+)\*\*', r'<strong>\1</strong>', t)
    t = re.sub(r'(?<![*\w])\*([^*]+)\*(?!\*)', r'<em>\1</em>', t)
    t = re.sub(r'\[([^\]]+)\]\(([^)]+)\)', r'<a href="\2">\1</a>', t)
    return t


def md_to_html(md, base):
    lines = md.split('\n')
    out, i = [], 0
    while i < len(lines):
        ln = lines[i]
        if ln.startswith('```'):
            j = i + 1
            buf = []
            while j < len(lines) and not lines[j].startswith('```'):
                buf.append(lines[j])
                j += 1
            out.append('<pre><code>%s</code></pre>' % html.escape('\n'.join(buf)))
            i = j + 1
            continue
        m = re.match(r'^(#{1,4})\s+(.*)$', ln)
        if m:
            lvl = len(m.group(1))
            out.append('<h%d>%s</h%d>' % (lvl, _inline(m.group(2)), lvl))
            i += 1
            continue
        m = re.match(r'^!\[([^\]]*)\]\(([^)]+)\)\s*$', ln)
        if m:
            out.append(_img(m.group(2), m.group(1), base))
            i += 1
            continue
        if ln.startswith('|'):
            rows = []
            while i < len(lines) and lines[i].startswith('|'):
                rows.append([c.strip() for c in lines[i].strip().strip('|').split('|')])
                i += 1
            h = '<table><thead><tr>%s</tr></thead><tbody>' % ''.join('<th>%s</th>' % _inline(c) for c in rows[0])
            for r in rows[2:]:
                h += '<tr>%s</tr>' % ''.join('<td>%s</td>' % _inline(c) for c in r)
            out.append(h + '</tbody></table>')
            continue
        if re.match(r'^\s*(- |\d+\. )', ln):
            ordered = bool(re.match(r'^\s*\d+\. ', ln))
            items = []
            while i < len(lines) and re.match(r'^\s*(- |\d+\. )', lines[i]):
                items.append(re.sub(r'^\s*(- |\d+\. )', '', lines[i]))
                i += 1
                while i < len(lines) and lines[i].startswith('   ') and not re.match(r'^\s*(- |\d+\. )', lines[i]):
                    items[-1] += ' ' + lines[i].strip()
                    i += 1
            tag = 'ol' if ordered else 'ul'
            out.append('<%s>%s</%s>' % (tag, ''.join('<li>%s</li>' % _inline(x) for x in items), tag))
            continue
        if not ln.strip():
            i += 1
            continue
        buf = [ln]
        i += 1
        while i < len(lines) and lines[i].strip() and not re.match(r'^(#|\||!\[|```|\s*- |\s*\d+\. )', lines[i]):
            buf.append(lines[i])
            i += 1
        out.append('<p>%s</p>' % _inline(' '.join(buf)))
    return '\n'.join(out)


CSS = """
body{font-family:system-ui,-apple-system,Segoe UI,sans-serif;max-width:980px;margin:2rem auto;padding:0 1.2rem;line-height:1.5;color:#1d1f23}
h1{border-bottom:2px solid #c44f2a;padding-bottom:.3rem;margin-top:3rem}
section.entry{page-break-before:always}
table{border-collapse:collapse;margin:1rem 0;font-size:.9rem;width:100%}
th,td{border:1px solid #ccc;padding:.3rem .5rem;text-align:left;vertical-align:top}
th{background:#f3f0ec}
code{background:#f1f1f1;padding:0 .2rem;border-radius:3px}
pre{background:#f6f6f6;padding:.8rem;overflow:auto}
figure{margin:1rem 0}figure img{max-width:100%;border:1px solid #ddd}
figcaption{font-size:.85rem;color:#666}
.toc a{text-decoration:none}.missing{color:#a00}
@media print{body{max-width:none}}
"""


def main(out_path=None):
    ent_dir = os.path.join(HERE, 'entries')
    files = sorted(f for f in os.listdir(_L(ent_dir)) if f.endswith('.md'))
    sections, toc = [], []
    for k, f in enumerate(files):
        md = open(_L(os.path.join(ent_dir, f)), encoding='utf-8').read()
        for key, fn in GENERATORS.items():
            if '{{%s}}' % key in md:
                md = md.replace('{{%s}}' % key, fn())
        title = re.search(r'^#\s+(.*)$', md, re.M)
        anchor = 'e%02d' % k
        toc.append('<li><a href="#%s">%s</a></li>' % (anchor, _inline(title.group(1) if title else f)))
        sections.append('<section class="entry" id="%s">%s</section>' % (anchor, md_to_html(md, ent_dir)))
    imgdir = os.path.join(DOCS, 'img')
    gallery = ['# Current design (renders from Fusion)', '']
    for f in sorted(os.listdir(_L(imgdir))):
        if f.endswith('.png'):
            gallery.append('![%s](%s)' % (f[:-4].replace('_', ' '), os.path.join('..', '..', 'img', f)))
    toc.append('<li><a href="#gallery">Current design</a></li>')
    sections.append('<section class="entry" id="gallery">%s</section>' % md_to_html('\n'.join(gallery), ent_dir))
    stamp = datetime.datetime.now().strftime('%Y-%m-%d %H:%M')
    doc = ('<!doctype html><html><head><meta charset="utf-8"><title>Hexapod Mk1: Engineering Design Notebook</title>'
           '<style>%s</style></head><body><p><em>Compiled %s from docs/notebook/entries.</em></p>'
           '<nav class="toc"><h2>Contents</h2><ol>%s</ol></nav>%s</body></html>') % (CSS, stamp, ''.join(toc), ''.join(sections))
    out_path = out_path or os.path.join(HERE, 'Hexapod_Mk1_Design_Notebook.html')
    open(_L(out_path), 'w', encoding='utf-8').write(doc)
    print('wrote', out_path, '(%d entries, %.1f MB)' % (len(files), len(doc) / 1e6))


if __name__ == '__main__':
    main(*sys.argv[1:2])
