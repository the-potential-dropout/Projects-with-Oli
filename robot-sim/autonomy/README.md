# Hexapod Mk1: autonomy and sim2real

Design and decisions are in [docs/AUTONOMY.md](../docs/AUTONOMY.md); the control interface is in [docs/CONTROL.md](../docs/CONTROL.md).

| Folder | What | Status |
| --- | --- | --- |
| [mujoco/](mujoco/) | MuJoCo model (MJCF, primary), scenes, viewer, stand test, Gymnasium env, PPO training and ONNX export | Model and test **working**. Env **working skeleton**. Training and export are **skeletons** (not yet trained) |
| [urdf/](urdf/) | `hexapod_mk1.urdf`, generated with the MJCF, for ROS 2 (robot_state_publisher, RViz, Nav2 footprint) | Generated; masses and limits match the MJCF |

Regenerate both after any CAD change:

```
python mujoco/make_mjcf.py
python mujoco/test_stand.py
```

`make_mjcf.py` reads `cad/build_report.json` and `cad/mesh/*.glb`, so re-run the Fusion export first.
