# 03 · Rev A verification: interference and joint ranges
*2026-11-25*

**Interference check.** Fusion's interference analysis on the whole assembly found and fixed:

- the adapter rail's root fillet rubbing the receiver slot;
- a femur fillet under the tibia servo tab;
- a bracket fillet dipping into the plate edge;
- the servo horn overlapping its spline;
- a battery tray fillet touching the battery.

**Joint sweep.** Each joint was driven to its extremes. Rev A was limited by:

- femur down ≤ 35°: the femur cradle hits the coxa cradle;
- tibia fold-in ≤ 45°.

The simulation session then measured the gait's real commanded ranges. The sim asked for a −148° tibia fold, so the sim capped its gait at −135°.

**Lesson carried into rev B.** Joint limits and gait ranges must be checked together. A fold limit becomes a minimum reach in the inverse kinematics (IK), the maths that turns a foot position into joint angles.
