"""Gymnasium environment for Hexapod Mk1 locomotion on MuJoCo (sim2real training).

Design (docs/AUTONOMY.md section 6):
- Action = residual joint offsets (rad, scaled to +-0.25) added to the classical gait
  (gait_ref.py = the firmware's gait).  On the robot the same residuals go to the
  ESP32 as an "R," line, so the ESP32 keeps IK limits, balance fallback and safety.
- Observation (all available on the real robot): gravity in the IMU frame, gyro,
  joint positions and velocities (bus-servo readback), previous action, command
  (vx, vz, yaw), gait phase (sin, cos) and the six foot switches.
- Domain randomisation every reset: payload 0-0.4 kg, body mass +-12 %, CoM +-15 mm,
  friction 0.5-1.2, servo kp +-25 %, action latency 0-30 ms, backlash +-0.6 deg,
  sensor noise, heightfield terrain (scene_terrain).

STATUS: working skeleton - it runs, steps and resets (python hexapod_env.py), the
reward is a sensible starting point, not a tuned result.
"""
import math

import gymnasium as gym
import mujoco
import numpy as np

from gait_ref import ClassicalGait
from model import LEGS, load, rough_terrain, stand_pose


class HexapodEnv(gym.Env):
    metadata = {'render_modes': []}

    def __init__(self, terrain=True, control_hz=50, seed=None):
        self.m = load('scene_terrain.xml' if terrain else 'scene_flat.xml')
        self.terrain = terrain
        self.d = mujoco.MjData(self.m)
        self.frame_skip = int(round(1.0 / control_hz / self.m.opt.timestep))
        self.qpos0, self.ctrl0 = stand_pose(self.m)
        self.rng = np.random.default_rng(seed)
        self.nominal = {k: getattr(self.m, k).copy() for k in ('body_mass', 'geom_friction', 'actuator_gainprm', 'actuator_biasprm', 'body_ipos')}
        self.torso = self.m.body('torso').id
        self.payload = self.m.body('payload').id
        n_obs = 3 + 3 + 18 + 18 + 18 + 3 + 2 + 6
        self.observation_space = gym.spaces.Box(-np.inf, np.inf, (n_obs,), np.float32)
        self.action_space = gym.spaces.Box(-1.0, 1.0, (18,), np.float32)
        self.action_scale = 0.25
        self.sid = {n: self.m.sensor(n).adr[0] for n in ('imu_quat', 'imu_gyro')}
        self.touch = [self.m.sensor(f'{l}_contact').adr[0] for l in LEGS]

    # ---------------------------------------------------------------- randomisation
    def _randomise(self):
        m, r, n = self.m, self.rng, self.nominal
        m.body_mass[:] = n['body_mass'] * r.uniform(0.88, 1.12, size=n['body_mass'].shape)
        m.body_mass[self.payload] = r.uniform(0.0, 0.4)
        m.body_ipos[self.torso] = n['body_ipos'][self.torso] + r.uniform(-0.015, 0.015, 3) * [1, 1, 0.3]
        m.geom_friction[:, 0] = n['geom_friction'][:, 0] * r.uniform(0.55, 1.35)
        kp = r.uniform(0.75, 1.25, size=m.nu)
        m.actuator_gainprm[:, 0] = n['actuator_gainprm'][:, 0] * kp
        m.actuator_biasprm[:, 1] = n['actuator_biasprm'][:, 1] * kp
        self.latency = int(r.integers(0, 3))          # control periods of delay (0-30 ms at 50 Hz -> 0-1.5 periods)
        self.backlash = math.radians(r.uniform(0.0, 0.6))
        if self.terrain:
            rough_terrain(m, seed=int(r.integers(1 << 30)))
        self.cmd = np.array([r.uniform(-0.1, 0.1), r.uniform(-0.05, 0.25), r.uniform(-0.8, 0.8)])

    # ---------------------------------------------------------------- gym API
    def reset(self, *, seed=None, options=None):
        if seed is not None:
            self.rng = np.random.default_rng(seed)
        self._randomise()
        mujoco.mj_resetData(self.m, self.d)
        self.d.qpos[:] = self.qpos0
        self.d.qpos[2] += 0.07 if self.terrain else 0.0
        self.d.ctrl[:] = self.ctrl0
        self.gait = ClassicalGait()
        self.prev = np.zeros(18)
        self.queue = [np.zeros(18)] * (self.latency + 1)
        self.t = 0
        mujoco.mj_forward(self.m, self.d)
        return self._obs(), {}

    def _obs(self):
        d = self.d
        q = d.sensordata[self.sid['imu_quat']:self.sid['imu_quat'] + 4]
        R = np.zeros(9)
        mujoco.mju_quat2Mat(R, q)
        grav = R.reshape(3, 3).T @ np.array([0, 0, -1.0])
        gyro = d.sensordata[self.sid['imu_gyro']:self.sid['imu_gyro'] + 3]
        jp = d.qpos[7:25] + self.rng.normal(0, math.radians(0.3), 18)
        jv = d.qvel[6:24] + self.rng.normal(0, 0.05, 18)
        feet = np.array([d.sensordata[a] > 1.0 for a in self.touch], float)
        ph = 2 * math.pi * self.gait.phase
        return np.concatenate([grav, gyro, jp, jv, self.prev, self.cmd, [math.sin(ph), math.cos(ph)], feet]).astype(np.float32)

    def step(self, action):
        a = np.clip(action, -1, 1) * self.action_scale
        self.queue.append(a)
        a_applied = self.queue.pop(0)                      # latency
        base = self.gait.step(1.0 / 50, vx=self.cmd[0], vz=self.cmd[1], yaw=self.cmd[2])
        target = base + a_applied
        # backlash: the joint only follows once the target moves past the dead band
        self.d.ctrl[:] = np.clip(target + self.rng.uniform(-self.backlash, self.backlash, 18),
                                 self.m.actuator_ctrlrange[:, 0], self.m.actuator_ctrlrange[:, 1])
        for _ in range(self.frame_skip):
            mujoco.mj_step(self.m, self.d)
        self.t += 1
        d = self.d
        # body velocity in the body frame (MuJoCo x fwd, y left) vs command (sim: vx left, vz fwd)
        R = d.xmat[self.torso].reshape(3, 3)
        v = R.T @ d.qvel[0:3]
        w = d.qvel[5]
        up = R[2, 2]
        track = math.exp(-((v[0] - self.cmd[1]) ** 2 + (v[1] - self.cmd[0]) ** 2) / 0.01) + 0.5 * math.exp(-(w - self.cmd[2]) ** 2 / 0.1)
        energy = float(np.sum(np.abs(d.actuator_force * d.qvel[6:24])))
        smooth = float(np.sum((a - self.prev) ** 2))
        height = d.xpos[self.torso][2]
        reward = 1.5 * track + 0.5 * up - 0.002 * energy - 0.5 * smooth - 0.2 * float(np.sum(a ** 2))
        self.prev = a
        fell = up < math.cos(math.radians(40)) or height < 0.035
        truncated = self.t >= 1000
        return self._obs(), float(reward - (5.0 if fell else 0.0)), bool(fell), bool(truncated), {}


if __name__ == '__main__':
    env = HexapodEnv(terrain=True, seed=1)
    obs, _ = env.reset()
    total = 0.0
    for _ in range(200):
        obs, r, term, trunc, _ = env.step(np.zeros(18))
        total += r
        if term or trunc:
            break
    print('obs', obs.shape, 'return over 200 steps with the plain gait:', round(total, 2), 'fell' if term else 'ok')
