# 17 · MuJoCo model and sim2real
*2026-11-25*

Oli picked **MuJoCo**.

**The model.** `autonomy/mujoco/make_mjcf.py` generates the MJCF (and a URDF) from the CAD build report and meshes:
- masses per link;
- the battery at its real position;
- a payload body at the top mount;
- STS servo position actuators with stall-torque limits;
- the IMU and foot-touch sensors.

Its joint angles are the simulator's.

**Checks** (`test_stand.py`, passes):
- The model **stands at 89 mm**, tilt 0.02°, all six feet loaded, femurs 0.22 N·m.
- The **firmware's own tripod gait walks it 0.58 m in 4 s** at a 0.15 m/s command.

**Training setup.**
- Gymnasium env: the policy learns **residuals** on top of the classical gait, so a zero output is the firmware gait.
- Domain randomisation: payload, mass, CoM, friction, servo kp, latency, backlash, terrain.
- Stable-Baselines3 PPO on the CPU (Windows-friendly); MuJoCo Playground on a Linux GPU is the upgrade path.
- ONNX export to the Pi.

The ESP32 clamps and expires residuals, so the classical controller stays the safety net.

**Status.** The model and env work. Training hasn't been run yet.
