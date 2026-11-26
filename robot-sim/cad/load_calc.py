"""Hexapod Mk1 joint-load check.

Worst case asked for in the design brief: tripod stance (3 feet down) carrying
the whole robot plus the heaviest top attachment, at the widest stance the gait
ever commands.  The gait geometry is copied from core/src/gait.rs: each foot
sits at `footReach` from its coxa axis, slides +-(v + w x r) * stance_time / 2,
and the coxa joint is `standHeight` above the floor.

Torques (static, quasi-steady):
  femur = F_foot * horizontal distance femur axis -> foot
  tibia = F_foot * horizontal distance tibia axis -> foot
  coxa  = mu * F_foot * horizontal distance coxa axis -> foot   (sideways push, mu = 0.3)
with F_foot = m g / 3 (dynamics are covered by the 2x margin).

    python load_calc.py            prints the table used in docs/DESIGN.md
"""
import json
import math
import sys

G = 9.81


def leg_ik(r, y, coxa, femur, tibia):
    """r = horizontal coxa-axis -> foot distance, y = foot height (negative).
    Returns (femur_angle, knee_x, femur_axis_x) in the leg plane, or None."""
    d = r - coxa
    L = math.hypot(d, y)
    if L > femur + tibia or L < abs(femur - tibia):
        return None
    a = math.atan2(y, d)
    b = math.acos((femur ** 2 + L ** 2 - tibia ** 2) / (2 * femur * L))
    f = a + b                       # knee up
    kx = coxa + femur * math.cos(f)
    return f, kx


def worst_stance(cfg, speeds, hz_list, omega):
    mounts = cfg['mounts']
    worst = 0.0
    for (mx, mz), splay in zip(mounts, cfg['legSplay']):
        yaw = (math.pi - math.radians(splay)) if mx < 0 else math.radians(splay)
        nx, nz = mx + cfg['footReach'] * math.cos(yaw), mz + cfg['footReach'] * math.sin(yaw)
        for hz in hz_list:
            st = 0.5 / hz
            for k in range(36):
                ang = 2 * math.pi * k / 36
                for v in speeds:
                    for w in (-omega, 0.0, omega):
                        dvx = v * math.cos(ang) + w * nz
                        dvz = v * math.sin(ang) - w * nx
                        for s in (-1.0, 1.0):
                            fx, fz = nx + dvx * st * 0.5 * s, nz + dvz * st * 0.5 * s
                            worst = max(worst, math.hypot(fx - mx, fz - mz))
    return worst


def check(cfg, mass_kg, attach_kg, servos, r_min=0.08, r_max=0.115, mu=0.3):
    """Worst static joint torques with the feet anywhere from r_min to r_max
    (horizontal coxa-axis -> foot distance) at stand height, tripod stance."""
    F = (mass_kg + attach_kg) * G / 3.0
    h = -cfg['standHeight']
    t_fem = F * (r_max - cfg['coxa'])
    t_tib, r = 0.0, r_min
    while r <= r_max + 1e-9:
        sol = leg_ik(r, h, cfg['coxa'], cfg['femur'], cfg['tibia'])
        if sol:
            t_tib = max(t_tib, F * abs(r - sol[1]))
        r += 0.0005
    t_cox = mu * F * r_max
    out = {'F_foot_N': round(F, 2), 'stance_mm': [r_min * 1000, r_max * 1000]}
    for joint, t in (('coxa', t_cox), ('femur', t_fem), ('tibia', t_tib)):
        s = servos[joint]
        out[joint] = {'servo': s['name'], 'required_kgcm': round(t / 0.0980665, 2), 'required_Nm': round(t, 3),
                      'rated_kgcm': s['rated'], 'stall_kgcm': s['stall'],
                      'margin_rated': round(s['rated'] * 0.0980665 / t, 2),
                      'margin_stall': round(s['stall'] * 0.0980665 / t, 2)}
    return out


ST3215 = {'name': 'Feetech STS3215 / Waveshare ST3215 (12 V)', 'rated': 10.0, 'stall': 30.0}
STS3250 = {'name': 'Feetech STS3250 (12 V)', 'rated': 16.0, 'stall': 50.0}

BASE = dict(coxa=0.045, femur=0.075, tibia=0.120, standHeight=0.09, footReach=0.10,
            mounts=[(-0.06, 0.11), (-0.08, 0.0), (-0.06, -0.11), (0.06, 0.11), (0.08, 0.0), (0.06, -0.11)],
            legSplay=[35, 0, -35, 35, 0, -35], speed_walk=0.26, speed_sprint=0.48, hz_walk=1.7, hz_sprint=2.6, turn=1.6)


def main(mass=2.96, attach=0.4):
    chosen = {'coxa': ST3215, 'femur': STS3250, 'tibia': ST3215}
    budget = {'coxa': ST3215, 'femur': ST3215, 'tibia': ST3215}
    rows = {
        'chosen: STS3250 femurs, feet 80-115 mm': check(BASE, mass, attach, chosen),
        'budget: all ST3215, feet 80-115 mm': check(BASE, mass - 6 * 0.0055, attach, budget),
        'for reference: old stance, feet out to 130 mm, all ST3215': check(BASE, mass, attach, budget, r_max=0.13 + 0.046),
        'uncapped gait reach (sprint + turn), all ST3215': check(BASE, mass, attach, budget, r_max=worst_stance(BASE, [BASE['speed_walk'], BASE['speed_sprint']], [BASE['hz_walk'], BASE['hz_sprint']], BASE['turn'])),
    }
    print(json.dumps(rows, indent=1))
    return rows


if __name__ == '__main__':
    main(*(float(a) for a in sys.argv[1:3]))
