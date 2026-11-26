# 15 · Wiring every part
*2026-11-25*

**Brief.** Consider every wire from the battery to each load (including every motor), and put the wires in the assembly.

**What changed from rev B.**
- **Fusing.** One 3 A PTC per leg branch on the side hubs, 5 A on J1, 1.1 A on the bridge, 1000 µF on each hub, and a star ground at the switch output.
- **Wire sizes.** 14 AWG main path (0.26 V drop at a 25 A peak), 18 AWG hub trunks, 22 AWG servo leads (worst servo 3.5 % total drop), 28 AWG twisted foot leads.
- **New leads, all modelled:**
  - six foot leads, each crossing the coxa, femur and knee axes on the axis;
  - the IMU lead from under the belly;
  - the bridge UART;
  - the Feather 5 V;
  - J1 AUX;
  - sonar and ToF;
  - the receiver and its antenna;
  - the five brain-module cables.
- **New routing features:**
  - plate tie slots;
  - femur fins;
  - tibia grooves, with a cross slot and an internal channel;
  - pod entries, with spares.

**Iterations driven by the collision sweep.**
1. The ToF wedge first went inside the bottom plate. It moved to the flange top.
2. A lid boss sat in a foot lead's path. The bosses moved.
3. Pod leads crossed. Each lead got its own hole.
4. The IMU board hit an insert column. The board moved.
5. The foot lead ran over a horn-screw head. The groove moved to x = 4.5 mm.
6. The **femur fins hit the folding tibia** at full fold. They moved next to the femur axis.

**Final sweep.** 46 of 49 poses are clean. That is the same as rev B: the only hits are at the coupled-limit corner.

The current cable table is in entry 11 (generated). The cut list, pin maps and assembly order are in [WIRING.md](../../WIRING.md).
