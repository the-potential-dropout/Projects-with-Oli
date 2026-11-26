"""Export a trained policy to ONNX for the brain module (Raspberry Pi 5, onnxruntime).

    python export_onnx.py runs/hexapod_ppo/final.zip runs/hexapod_ppo/vecnormalize.pkl hexapod_policy.onnx

The observation normalisation is folded into the graph, so the Pi feeds raw
observations (same order as HexapodEnv._obs) and gets 18 residuals in [-1, 1];
multiply by 0.25 rad and send them to the ESP32 as  R,<18 values in degrees>  at 50 Hz.
"""
import pickle
import sys

import numpy as np
import torch
from stable_baselines3 import PPO


class Policy(torch.nn.Module):
    def __init__(self, sb3, mean, var, clip=10.0):
        super().__init__()
        self.net = sb3.policy
        self.mean = torch.tensor(mean, dtype=torch.float32)
        self.std = torch.tensor(np.sqrt(var + 1e-8), dtype=torch.float32)
        self.clip = clip

    def forward(self, obs):
        x = torch.clamp((obs - self.mean) / self.std, -self.clip, self.clip)
        feat = self.net.extract_features(x)
        lat = self.net.mlp_extractor.forward_actor(feat)
        return torch.clamp(self.net.action_net(lat), -1.0, 1.0)


if __name__ == '__main__':
    model_zip, norm_pkl, out = sys.argv[1:4]
    sb3 = PPO.load(model_zip, device='cpu')
    vn = pickle.load(open(norm_pkl, 'rb'))
    pol = Policy(sb3, vn.obs_rms.mean, vn.obs_rms.var).eval()
    dummy = torch.zeros(1, sb3.observation_space.shape[0])
    torch.onnx.export(pol, dummy, out, input_names=['obs'], output_names=['residual'], opset_version=17,
                      dynamic_axes={'obs': {0: 'batch'}, 'residual': {0: 'batch'}})
    print('wrote', out)
