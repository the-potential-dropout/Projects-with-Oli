# 18 · Open questions and next steps
*2026-11-25*

- **Femur servos.** Is the STS3250 (about $99 each, 2.0× margin) worth it, or is the all-STS3215 budget build (1.3× rated, 3.9× stall) good enough while Oli learns?
- **Simulator** (the sim session):
  - mirror the onboard controller (`control` in `hexapod.config.js`): stance-window rule, lift cap, ripple and wave gaits, balance PI-D, touchdown search, stability rule;
  - uneven terrain, a simulated IMU and foot contacts;
  - a radio-style control layout and an "autonomous" bot that uses the battle behaviour tree;
  - read the `B,` and `T,` telemetry lines.
- **Verify on real parts.**
  - STS3250 housing and rated torque on the shipped datasheet; socket positions on a real ST3215.
  - STS P/D register addresses read back before writing.
  - Battery fit; the LD19's real size (listings disagree); the D2F switch travel in the printed boot (coupon first).
- **Firmware.** Compile against the real libraries and run on the stand: the controller maths is host-tested, the hardware layers aren't. Then tune as in CONTROL.md §12.
- **Autonomy.**
  - Write the ROS 2 `hexapod_bridge` node.
  - Train the residual policy.
  - Measure a servo step response to fit the MuJoCo actuator model.
- **Not yet done.** Nothing has been printed or built. Loads, currents and runtimes are calculations, to be checked with the ACS724 log on the first walk.
