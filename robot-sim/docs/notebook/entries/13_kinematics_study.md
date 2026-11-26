# 13 · Leg architecture study (rev B2)
*2026-11-25*

**Question** (Oli and the parent brief). Could smarter kinematics (linkages, coupled legs, fewer actuators) do the job with less? Oli then added a hard rule: **the body must still tilt**, and ideally change height.

**Options studied** (full table and sources in [KINEMATICS.md](../../KINEMATICS.md)):
- 3-DOF serial, 18 servos;
- 3-DOF with the knee servo moved to the hip through a four-bar;
- 2-DOF pantograph, 12 servos;
- hybrid, 16 servos;
- coupled tripods, 2–4 motors;
- Jansen / Klann, 2 motors;
- RHex, 6 motors.

**Decision: keep the 3-DOF serial leg.**
1. **Excluded by the tilt rule.** Coupled, linkage-walker and RHex designs can't control each foot's height, so they can't tilt the body.
2. **The four-bar knee is too small a gain.** It saves only about 6 % femur torque, because rev B already points the tibia servo's case back towards the femur axis. And four-bar knees reach under about 90°, where Mk1 needs 165°.
3. **The 2-DOF pantograph gives up too much.** It saves 6 servos (≈ $130) but costs:
   - about 2 mm of foot slip per step;
   - no strafing;
   - no stance-width control (our load cap);
   - about 72 pivots.

**For the simulator.** Nothing changes: 3 DOF, the same lengths and limits, no passive joints.
