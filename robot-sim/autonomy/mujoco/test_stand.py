"""Smoke test: the model loads, stands on the ground on its own servos, and the
sensors report sensible values.  python test_stand.py   (exit code 0 = pass)"""
import math
import sys

import mujoco
import numpy as np

from model import JOINTS, LEGS, load, stand_pose


def main():
    m = load('scene_flat.xml')
    d = mujoco.MjData(m)
    qpos, ctrl = stand_pose(m)
    d.qpos[:] = qpos
    d.ctrl[:] = ctrl
    mujoco.mj_forward(m, d)
    total = sum(m.body_mass)
    for _ in range(int(3.0 / m.opt.timestep)):
        mujoco.mj_step(m, d)
    torso = m.body('torso').id
    z = d.xpos[torso][2]
    up = d.xmat[torso].reshape(3, 3)[:, 2]
    tilt = math.degrees(math.acos(max(-1.0, min(1.0, up[2]))))
    touch = [d.sensor(f'{l}_contact').data[0] for l in LEGS]
    err = max(abs(d.qpos[7 + i] - ctrl[i]) for i in range(18))
    torques = [abs(d.actuator_force[i]) for i in range(18)]
    print(f'mass {total:.3f} kg | body height {z * 1000:.1f} mm | tilt {tilt:.2f} deg | worst joint error {math.degrees(err):.2f} deg')
    print('foot forces (N):', ' '.join(f'{t:.1f}' for t in touch), f'| sum {sum(touch):.1f} vs weight {total * 9.81:.1f}')
    print('servo torques (N m): coxa %.2f femur %.2f tibia %.2f (max)' % (max(torques[0::3]), max(torques[1::3]), max(torques[2::3])))
    ok = 0.075 < z < 0.10 and tilt < 3.0 and all(t > 1.0 for t in touch) and abs(sum(touch) - total * 9.81) < 0.15 * total * 9.81
    # walk with the firmware's tripod gait (gait_ref.py) for 4 s at 0.15 m/s
    from gait_ref import ClassicalGait
    g = ClassicalGait()
    x0 = d.qpos[0]
    for k in range(int(4.0 / m.opt.timestep)):
        if k % 10 == 0:
            d.ctrl[:] = g.step(10 * m.opt.timestep, vz=0.15)
        mujoco.mj_step(m, d)
    up = d.xmat[torso].reshape(3, 3)[2, 2]
    walked = d.qpos[0] - x0
    print(f'walk: {walked:.3f} m forward in 4 s (command 0.15 m/s), height {d.qpos[2] * 1000:.1f} mm, upright {up:.3f}')
    ok = ok and 0.4 < walked < 0.7 and up > 0.98
    print('PASS' if ok else 'FAIL')
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
