"""Train a residual locomotion policy for Hexapod Mk1 with PPO (Stable-Baselines3).

    pip install mujoco gymnasium stable-baselines3 torch
    python train_ppo.py --steps 20_000_000 --envs 8

Runs on a normal Windows / Linux PC on the CPU (8 parallel MuJoCo envs about
3-5k steps/s on a 8-core desktop: ~20 M steps overnight).  For GPU-scale training port
hexapod_env.py to MuJoCo Playground (MJX/JAX, Linux + CUDA) - see README.md.

STATUS: skeleton.  Hyper-parameters are standard PPO locomotion defaults; expect to
tune the reward weights in hexapod_env.py.
"""
import argparse

from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import CheckpointCallback
from stable_baselines3.common.vec_env import SubprocVecEnv, VecNormalize

from hexapod_env import HexapodEnv

CONFIG = dict(
    policy='MlpPolicy',
    n_steps=1024, batch_size=8192, n_epochs=5, learning_rate=3e-4, gamma=0.99, gae_lambda=0.95,
    clip_range=0.2, ent_coef=0.0, vf_coef=0.5, max_grad_norm=1.0,
    policy_kwargs=dict(net_arch=dict(pi=[256, 128, 64], vf=[256, 128, 64])),
)


def make(i, terrain):
    return lambda: HexapodEnv(terrain=terrain, seed=1000 + i)


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('--steps', type=int, default=20_000_000)
    ap.add_argument('--envs', type=int, default=8)
    ap.add_argument('--flat', action='store_true', help='start on flat ground (curriculum stage 1)')
    ap.add_argument('--out', default='runs/hexapod_ppo')
    a = ap.parse_args()
    env = VecNormalize(SubprocVecEnv([make(i, not a.flat) for i in range(a.envs)]), norm_obs=True, norm_reward=True, clip_obs=10.0)
    model = PPO(env=env, verbose=1, tensorboard_log=a.out, **CONFIG)
    model.learn(total_timesteps=a.steps, callback=CheckpointCallback(500_000, a.out, name_prefix='ppo'))
    model.save(a.out + '/final')
    env.save(a.out + '/vecnormalize.pkl')
