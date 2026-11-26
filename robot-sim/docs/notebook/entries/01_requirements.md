# 01 · Requirements and constraints
*2026-11-25*

**Goal.** A real six-legged robot that Oli can print, build and drive from the Robot Sim. The robot and the sim share `hexapod.config.js`, so the CAD must match it, or the config must change with it.

**Fixed inputs from the simulator.**
- Leg 45 / 75 / 120 mm (coxa / femur / tibia).
- Six coxa mounts at front ±60, +110; middle ±80, 0; rear ±60, −110 mm.
- Splay 35°.
- Body about 260 × 140 mm.

**Requirements, in the user's priority order.**
1. **Printability (DFM).**
   - Every part fits a 220 × 220 × 250 mm bed (note anything that needs 180 mm).
   - Flat print faces, 45° overhang rule, teardrop holes.
   - PLA/PETG walls, 0.2–0.3 mm fits, M3 heat-set inserts.
2. **Assembly (DFA).**
   - Few unique parts: identical legs, mirrored only if unavoidable.
   - One fastener type.
   - Servos drop in from one direction.
   - Captive inserts, support on both sides of every joint, self-locating features.
3. **Fillets and chamfers** on stress corners, and an elephant-foot chamfer on first-layer edges.
4. **Room for the electronics**, including a battery you can swap without taking the robot apart.
5. **A modular top deck**: grid, quick-release, connector, example adapter, front sensor bracket.

**Added later (rev B, entry 05).**
- A rechargeable battery that powers everything.
- Actuators that are easy to buy and strong enough (at least 2× their *rated* torque).
- The manufacturers' real CAD for every standard part.
- Fillets and chamfers wherever stress needs them, with no interference.
- Every wire, cable and connection modelled.
- This notebook.

**Safety and tone.** Weapon attachments stay toys: an IR/LED "laser" and a foam-ball launcher. LiPo handling follows hobby safety practice for a student.
