# 07 · Power system
*2026-11-25*

**Voltage.** The STS servos run at 12 V nominal, so the pack is 3S (12.6 V full, 9.9 V cut-off).

**Chemistry and pack.**
- **3S1P 21700 Li-ion packs** (Molicel P45B, 4.5 Ah, 45 A): the commercial packs found are triangular bundles about 50 mm tall, which don't fit between plates set 30.4 mm apart by the servo case. Building a pack means spot-welding bare cells, which isn't suitable for a student project. **Rejected.**
- **Gens Ace 3S 4000 mAh 50–60C hard-case LiPo:** 138.5 × 43 × 24.5 mm, about 305 g, XT60, sold everywhere. **Chosen.** It slides in from the rear between the rear coxa servos.

**Protection.** Battery-side protection is a balance charger (which does CC/CV and per-cell balancing), plus:

- a 20 A ATC fuse right after the XT60;
- a Pololu MOSFET switch with reverse-polarity protection;
- an ACS724 ±30 A current sensor and a voltage divider read by the ESP32;
- a firmware cut-off at 3.4 V/cell (3 s debounce).

A separate BMS was considered. Cheap 3S boards limit current and add a failure point, and the balance charger plus fuse plus cut-off covers each risk a BMS would.

**Charge port.** The rear panel carries the pack's XT60 charge leads and a JST-XH 4 balance extension. Charge with the robot off, attended, on a non-flammable surface; a bag charge out of the robot is always the option.

**Current budget.**

| State | Current |
| --- | --- |
| Logic | about 0.2 A |
| Standing | about 2 A |
| Walking | 4–6 A |
| Peak | about 25 A transient |
| Theoretical stall | 50 A |

**Runtime.** 3.2 Ah usable gives **32–48 min walking**.

{{mass}}
