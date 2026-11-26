# 16 · Remote control and autonomy
*2026-11-25*

**Radio.** An EdgeTX radio (RadioMaster TX16S MK3) with ExpressLRS. Both are open source and the radio is Lua-scriptable; it beat a Bluetooth gamepad (no telemetry, short range) and a home-made ESP-NOW radio.

**Robot side.**
- An RP1 V2 receiver in the plastic pod, its antenna vertical on a lid post.
- CRSF goes to the Feather.
- Telemetry goes back to the radio: battery, attitude, mode, faults.
- The Lua widget and the black-and-white script are in `radio/`.

**Command priority.** Safety > radio > brain (radio in auto, sticks centred) > USB host. Without a radio, the brain can't drive at all.

**Brain module.**
- **Compute.** A Pi 5 instead of a Jetson Orin Nano: the OAK-D Lite runs the neural networks on its own VPU, and the Pi uses about 9 W against 15–25 W and costs about a third.
- **Packaging.** A swappable top-deck module: Pi 5, OAK-D Lite, LD19 and a 5.5 A buck. It is 300 g, inside the 0.4 kg attachment allowance, and runs from J1. The Pi talks to the ESP32 over J1's two AUX lines; the Feather tells a brain from a weapon by AUX2's idle level.

**ROS 2 Jazzy design.**
- slam_toolbox;
- Nav2 (MPPI, omnidirectional);
- elevation map;
- OAK-D detections;
- a battle behaviour tree;
- an ONNX policy that sends `R,` residuals.

Details are in [AUTONOMY.md](../../AUTONOMY.md).
