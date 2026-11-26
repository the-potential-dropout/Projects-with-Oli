"""Hexapod Mini joint-load check (static, the robot's own weight).

Worst case: tripod stance (3 feet down), each foot anywhere in its stance window
(horizontal coxa-axis -> foot distance r), hip plane at the walking height.  Unlike
the Mk1 check, which shares the weight equally (F = m g / 3), the foot loads here are
solved from the tripod geometry: the lone middle foot carries ~44 % of the weight.

  femur torque = F_foot x (r - coxa)                 horizontal femur axis -> foot
  tibia torque = F_foot x |r - knee_x|               horizontal tibia axis -> foot
  coxa  torque = mu x F_foot x r                     sideways push, mu = 0.3

Servo: MG90S at 5.0 V (Mini560 bucks).  Its datasheet gives one torque figure,
1.8 kg.cm at 4.8 V and 2.2 kg.cm at 6.0 V, i.e. the stall torque; interpolated to
5.0 V that is 1.87 kg.cm.  "Rated" below means that datasheet figure, as there is
no separate continuous rating.  Requirement: rated / required >= 2.

    python load_calc_mini.py [mass_kg]       (default: cad/build_report.json total)
"""
import json
import math
import os
import sys

G = 9.81
KGCM = 0.0980665
HERE = os.path.dirname(os.path.abspath(__file__))

GEO = dict(coxa=0.030, femur=0.044, tibia=0.060, height=0.045,
           mounts=[(0.048, 0.072, 40), (0.060, 0.0, 0), (0.048, -0.072, -40),
                   (-0.048, 0.072, 140), (-0.060, 0.0, 180), (-0.048, -0.072, -140)],
           reach=0.058,
           window_middle=(0.052, 0.061), window_corner=(0.052, 0.070))
MG90S = dict(name='MG90S @ 5.0 V', rated=1.8 + (2.2 - 1.8) * (5.0 - 4.8) / (6.0 - 4.8))
MU = 0.3


def foot(i, r):
    x, y, d = GEO['mounts'][i]
    a = math.radians(d)
    return (x + r * math.cos(a), y + r * math.sin(a))


def tripod_shares(legs, r):
    """Vertical foot loads (fractions of the weight) for 3 feet, CoM over the origin."""
    p = [foot(i, r) for i in legs]
    # sum F = 1, sum F x = 0, sum F y = 0  (3x3 linear system)
    A = [[1, 1, 1], [p[0][0], p[1][0], p[2][0]], [p[0][1], p[1][1], p[2][1]]]
    b = [1, 0, 0]
    det = lambda M: (M[0][0] * (M[1][1] * M[2][2] - M[1][2] * M[2][1]) - M[0][1] * (M[1][0] * M[2][2] - M[1][2] * M[2][0])
                     + M[0][2] * (M[1][0] * M[2][1] - M[1][1] * M[2][0]))
    d0 = det(A)
    out = []
    for k in range(3):
        M = [row[:] for row in A]
        for j in range(3):
            M[j][k] = b[j]
        out.append(det(M) / d0)
    return dict(zip(legs, out))


def knee_x(r, h):
    c, f, t = GEO['coxa'], GEO['femur'], GEO['tibia']
    d = r - c
    L = math.hypot(d, h)
    a = math.atan2(-h, d) + math.acos((f * f + L * L - t * t) / (2 * f * L))
    return c + f * math.cos(a)


def joint_torques(F, window, h):
    r_lo, r_hi = window
    fem = F * (r_hi - GEO['coxa'])
    tib, r = 0.0, r_lo
    while r <= r_hi + 1e-9:
        tib = max(tib, F * abs(r - knee_x(r, h)))
        r += 0.0005
    cox = MU * F * r_hi
    return {'coxa': cox, 'femur': fem, 'tibia': tib}


def row(F, window, h, label):
    t = joint_torques(F, window, h)
    out = {'case': label, 'F_foot_N': round(F, 2), 'window_mm': [round(w * 1000) for w in window]}
    for j in ('coxa', 'femur', 'tibia'):
        req = t[j] / KGCM
        out[j] = {'required_kgcm': round(req, 2), 'rated_kgcm': round(MG90S['rated'], 2), 'margin': round(MG90S['rated'] / req, 2) if req > 0 else None}
    return out


def main(mass=None):
    if mass is None:
        rep = os.path.join(HERE, 'build_report.json')
        mass = json.load(open(rep))['mass_g']['total'] / 1000.0 if os.path.exists(rep) else 0.65
    W = mass * G
    h = GEO['height']
    s = tripod_shares((0, 2, 4), GEO['reach'])
    lone, corner = s[4], max(s[0], s[2])
    rows = [
        row(W * lone, GEO['window_middle'], h, 'tripod, lone middle foot (%.0f %% of weight)' % (100 * lone)),
        row(W * corner, GEO['window_corner'], h, 'tripod, corner foot (%.0f %% of weight)' % (100 * corner)),
        row(W / 3, GEO['window_corner'], h, 'Mk1 convention: m g / 3, feet out to 70 mm'),
        row(W * lone, (0.052, 0.070), h, 'for reference: lone middle foot out to 70 mm (no clamp)'),
    ]
    res = {'mass_kg': round(mass, 3), 'servo': MG90S['name'], 'rated_kgcm': round(MG90S['rated'], 2),
           'stand_height_mm': h * 1000, 'shares': {str(k): round(v, 3) for k, v in s.items()}, 'rows': rows}
    print(json.dumps(res, indent=1))
    return res


if __name__ == '__main__':
    main(*(float(a) for a in sys.argv[1:2]))
