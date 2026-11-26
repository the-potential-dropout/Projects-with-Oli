# Hexapod Mk1 in MuJoCo

```
pip install mujoco numpy trimesh gymnasium         # model, viewer, env
pip install stable-baselines3 torch onnx           # training and export
```

## Files

| File | What it does | Status |
| --- | --- | --- |
| `make_mjcf.py` | Generates `hexapod_mk1.xml`, the scenes, `meshes/*.stl` and `../urdf/hexapod_mk1.urdf` from the CAD report and meshes | Working |
| `hexapod_mk1.xml` | The robot (see below) | Generated |
| `scene_flat.xml` / `scene_terrain.xml` | Floor / 6 × 6 m heightfield (bumps up to 6 cm, filled at run time) | Working |
| `model.py` | Loader (works from long Windows paths), stand keyframe, random terrain | Working |
| `view.py` | Viewer: `python view.py [--terrain] [--walk]` | Working |
| `test_stand.py` | Smoke test: stands, sensors, walks 4 s with the firmware gait | **Passes** (89 mm stand height, tilt 0.02°, 0.58 m in 4 s at 0.15 m/s) |
| `gait_ref.py` | The firmware's tripod gait in Python (the base the policy corrects) | Working |
| `hexapod_env.py` | Gymnasium env: residual actions, 71 observations, domain randomisation, terrain | Working skeleton (reward not tuned) |
| `train_ppo.py` | Stable-Baselines3 PPO, 8 CPU envs, checkpoints | Skeleton |
| `export_onnx.py` | Policy + observation normalisation → `hexapod_policy.onnx` | Skeleton |

## The model

**Frames.**
- World z up, x forward, y left.
- **Joint signs equal the simulator's and the firmware's:** coxa + = foot forward, femur + = up, tibia + = unfold, −90° = tibia vertical. So `qpos[7:25]` and `ctrl` are the sim's joint angles in radians, leg order FR, MR, RR, FL, ML, RL.

**Mass.**
- Total 2.96 kg plus a 0.30 kg payload (the brain module) on the top mount.
- Per-link masses come from the Fusion autonomy build (`cad/build_report_autonomy.json`: STS3250 femurs + brain).
- The battery's mass sits at its real position, 43 mm behind the centre.
- Inertias come from the collision primitives.

**Joints.**
- Limits: coxa ±45°, femur −10…70°, tibia −135…30°.
- Damping 0.05, frictionloss 0.03, armature 0.008 (STS3215) / 0.010 (STS3250).

**Actuators.**
- `position` actuators: STS3215 kp 11 N·m/rad, kv 0.25; STS3250 kp 18, kv 0.35.
- Force limit = stall torque (2.94 / 4.9 N·m).
- Fit kp, kv and the latency to a measured step response on the real robot (AUTONOMY.md §6).

**Contacts and sensors.**
- Feet: spheres r = 11 mm, μ = 0.9.
- `imu_quat` / `imu_gyro` / `imu_acc` at the BNO085 position.
- `<leg>_contact` touch sensors at each foot; the firmware's foot switch closes at about 1.5 N.
- Joint position and velocity for all 18 joints.

**Linkages.** None. The Mk1 leg is a serial 3-DOF chain, so there are no equality constraints or tendons.

## Training (why Gymnasium + SB3, and the upgrade path)

**On a normal PC.** `python train_ppo.py --flat --steps 3000000`, then `python train_ppo.py` on terrain. 8 envs run at about 3–5k steps/s on an 8-core CPU, about 20 M steps overnight. It is plain Windows-native Python with no GPU needed. TensorBoard logs go to `runs/` (ignored by git).

**Upgrade path: MuJoCo Playground (MJX/JAX).** It is 100× faster on an NVIDIA GPU under Linux or WSL2; the JAX GPU path isn't available on native Windows. Port `HexapodEnv` to a Playground `MjxEnv` (same observation, action and reward), load `hexapod_mk1.xml`, and use `brax` PPO.

## Deploy

```
python export_onnx.py runs/hexapod_ppo/final.zip runs/hexapod_ppo/vecnormalize.pkl hexapod_policy.onnx
```

1. Copy `hexapod_policy.onnx` to the brain's Pi and run it with onnxruntime at 50 Hz.
2. Build the 71-value observation from the `T,` telemetry, the joint readback and the commanded velocity, in the same order as `HexapodEnv._obs()`.
3. Send `R,` + 18 residuals × 0.25 rad, converted to degrees, to the ESP32.

**The ESP32 stays in charge.** It clamps residuals to ±15° and the joint limits. It drops them after 100 ms, or the moment the radio drives, and its balance and terrain layers keep running underneath.
