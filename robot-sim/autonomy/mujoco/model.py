"""Load the Hexapod Mk1 MuJoCo scenes (works from any path length: files are read
into MuJoCo's in-memory asset map, so Windows' 260-character limit doesn't bite)."""
import math
import os

import mujoco
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
B = chr(92)
LEGS = ['fr', 'mr', 'rr', 'fl', 'ml', 'rl']
JOINTS = [f'{l}_{j}' for l in LEGS for j in ('coxa', 'femur', 'tibia')]


def _L(p):
    p = os.path.abspath(p)
    return (B + B + '?' + B + p) if os.name == 'nt' and not p.startswith(B + B) else p


def load(scene='scene_flat.xml'):
    assets = {}
    for name in ('hexapod_mk1.xml', 'scene_flat.xml', 'scene_terrain.xml'):
        assets[name] = open(_L(os.path.join(HERE, name)), 'rb').read()
    mdir = os.path.join(HERE, 'meshes')
    if os.path.isdir(_L(mdir)):
        for f in os.listdir(_L(mdir)):
            assets['meshes/' + f] = open(_L(os.path.join(mdir, f)), 'rb').read()
    return mujoco.MjModel.from_xml_string(assets[scene].decode(), assets)


def stand_pose(m):
    k = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_KEY, 'stand')
    return m.key_qpos[k].copy(), m.key_ctrl[k].copy()


def rough_terrain(m, height=0.06, smooth=6, seed=0):
    """Fill the scene_terrain heightfield with random bumps (0..1 scaled by the hfield size)."""
    rng = np.random.default_rng(seed)
    hid = 0
    nr, nc = m.hfield_nrow[hid], m.hfield_ncol[hid]
    z = rng.random((nr, nc))
    k = np.ones(smooth) / smooth
    for _ in range(2):
        z = np.apply_along_axis(lambda r: np.convolve(r, k, mode='same'), 0, z)
        z = np.apply_along_axis(lambda r: np.convolve(r, k, mode='same'), 1, z)
    z = (z - z.min()) / max(z.max() - z.min(), 1e-6)
    c = nr // 2
    z[c - 15:c + 15, c - 15:c + 15] = z[c - 15:c + 15, c - 15:c + 15].min()   # flat start pad
    m.hfield_data[:] = z.ravel()
    return z
