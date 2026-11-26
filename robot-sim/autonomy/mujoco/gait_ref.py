"""Python port of the onboard tripod gait (firmware/hexapod_controller/kinematics.cpp,
same as robot-sim/core/src/gait.rs) used as the reference motion in MuJoCo and as the
base the RL policy adds residuals to.  Sim frame inside: +X left, +Y up, +Z forward."""
import math

import numpy as np

COXA, FEMUR, TIBIA = 0.045, 0.075, 0.120
STAND, REACH, RMIN, RMAX = 0.090, 0.100, 0.080, 0.115
MOUNTS = [(-0.06, 0.11), (-0.08, 0.0), (-0.06, -0.11), (0.06, 0.11), (0.08, 0.0), (0.06, -0.11)]
SPLAY = [35, 0, -35, 35, 0, -35]
TIBIA_MIN = math.radians(-135)
K_MIN = math.sqrt(FEMUR ** 2 + TIBIA ** 2 - 2 * FEMUR * TIBIA * math.cos(math.pi + TIBIA_MIN)) + 2e-4


def solve_leg(x, y, z):
    coxa = math.atan2(z, x)
    horiz = math.hypot(x, z) - COXA
    d = math.hypot(horiz, y)
    d = min(max(d, max(K_MIN, abs(FEMUR - TIBIA)) + 1e-4), FEMUR + TIBIA - 1e-4)
    femur = math.atan2(y, horiz) + math.acos(max(-1, min(1, (FEMUR ** 2 + d * d - TIBIA ** 2) / (2 * FEMUR * d))))
    tibia = math.acos(max(-1, min(1, (FEMUR ** 2 + TIBIA ** 2 - d * d) / (2 * FEMUR * TIBIA)))) - math.pi
    return coxa, femur, tibia


class ClassicalGait:
    def __init__(self, step_hz=1.7, step_height=0.035, height=STAND):
        self.yaw = [(math.pi - math.radians(s)) if x < 0 else math.radians(s) for (x, _), s in zip(MOUNTS, SPLAY)]
        self.neutral = [(x + REACH * math.cos(a), z + REACH * math.sin(a)) for (x, z), a in zip(MOUNTS, self.yaw)]
        self.phase, self.act, self.vel = 0.0, 0.0, np.zeros(3)
        self.step_hz, self.step_height, self.height = step_hz, step_height, height

    def step(self, dt, vx=0.0, vz=0.0, yaw=0.0):
        k = 1 - math.exp(-dt * 8)
        self.vel += (np.array([vx, vz, yaw]) - self.vel) * k
        moving = abs(self.vel[0]) + abs(self.vel[1]) + abs(self.vel[2]) * 0.15 > 0.004
        self.act += ((1.0 if moving else 0.0) - self.act) * (1 - math.exp(-dt * 5))
        self.phase = (self.phase + dt * self.step_hz) % 1.0 if self.act > 0.02 else self.phase
        st = 0.5 / self.step_hz
        out = []
        for i in range(6):
            nx, nz = self.neutral[i]
            hx = (self.vel[0] + self.vel[2] * nz) * st * 0.5
            hz = (self.vel[1] - self.vel[2] * nx) * st * 0.5
            local = self.phase if i % 2 == 0 else (self.phase + 0.5) % 1.0
            if local < 0.5:
                u = local / 0.5
                s, lift = (u * u * (3 - 2 * u)) * 2 - 1, math.sin(math.pi * u)
            else:
                s, lift = 1 - 2 * (local - 0.5) / 0.5, 0.0
            fx, fz = nx + hx * s, nz + hz * s
            fy = -self.height + lift * self.step_height * min(self.act, 1.0)
            mx, mz = MOUNTS[i]
            c, sn = math.cos(self.yaw[i]), math.sin(self.yaw[i])
            lx, lz = (fx - mx) * c + (fz - mz) * sn, -(fx - mx) * sn + (fz - mz) * c
            horiz = math.hypot(lx, lz) - COXA
            if horiz < K_MIN:
                fy = min(fy, -math.sqrt(K_MIN ** 2 - horiz ** 2))
            out += solve_leg(lx, fy, lz)
        return np.array(out)
