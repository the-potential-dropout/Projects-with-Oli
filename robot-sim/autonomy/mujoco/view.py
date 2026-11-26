"""Open Hexapod Mk1 in the MuJoCo viewer.

    python view.py                 # flat floor, robot holding its stand pose
    python view.py --terrain       # random rough heightfield (up to 6 cm)
    python view.py --walk          # run the classical gait (same maths as the firmware)

Press Tab / the left panel to toggle group 3 (collision shapes) and the sensor sites.
"""
import argparse
import time

import mujoco
import mujoco.viewer

from gait_ref import ClassicalGait
from model import load, rough_terrain, stand_pose


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--terrain', action='store_true')
    ap.add_argument('--walk', action='store_true')
    a = ap.parse_args()
    m = load('scene_terrain.xml' if a.terrain else 'scene_flat.xml')
    if a.terrain:
        rough_terrain(m)
    d = mujoco.MjData(m)
    qpos, ctrl = stand_pose(m)
    d.qpos[:] = qpos
    d.qpos[2] += 0.06 if a.terrain else 0.0
    d.ctrl[:] = ctrl
    gait = ClassicalGait() if a.walk else None
    with mujoco.viewer.launch_passive(m, d) as v:
        t0 = time.time()
        while v.is_running():
            if gait is not None:
                d.ctrl[:] = gait.step(m.opt.timestep, vz=0.15, yaw=0.0)
            mujoco.mj_step(m, d)
            v.sync()
            dt = d.time - (time.time() - t0)
            if dt > 0:
                time.sleep(dt)


if __name__ == '__main__':
    main()
