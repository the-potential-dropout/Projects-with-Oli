# Design versions

Every version of the robots is kept: in git (a tag for each), in the repo as files, and in Fusion's cloud. Nothing has been deleted; older versions are replaced only as the *default* to build.

## Hexapod Mk1

| Version | What it is | Status | Git tag | Files in the repo | Fusion cloud |
| --- | --- | --- | --- | --- | --- |
| Rev A | MG996R PWM servos, 2S pack, two PCA9685 boards | Superseded | `mk1-revA` | At the tag; described in notebook entries 01–04 | "Hexapod Mk1" version 1 |
| Rev B | Feetech bus servos, 3S LiPo, vendor CAD, harness | Superseded by B2 | `mk1-revB` | At the tag; notebook entries 05–12 | "Hexapod Mk1" version 2 |
| **Rev B2 RC** | **Default build.** All 18 leg servos STS3215, Feather ESP32-S3 onboard control, IMU, foot switches, ELRS radio. No brain module; turret (or another attachment ≤ 0.2 kg) on the top mount. About US$1,100 | **Current: build this first** | `mk1-revB2-rc` | `cad/Hexapod_Mk1.f3d`, `cad/build_report.json`, `docs/img/*.png` | **"Hexapod Mk1 RC"** |
| Rev B2 autonomy | Upgrade of the RC build: six STS3250 femurs + the brain module (Pi 5, OAK-D Lite, LD19), MuJoCo / RL policy. About US$1,920 | Kept for the upgrade | `mk1-revB2-autonomy` | `cad/Hexapod_Mk1_autonomy.f3d`, `cad/build_report_autonomy.json`, `docs/img/autonomy/*.png`, `autonomy/` | "Hexapod Mk1" version 3 |

**One generator, two builds.** `cad/hexapod_mk1_fusion.py` builds either one: set `BUILD = 'rc'` or `'autonomy'` at the top, or call `build(out_dir, use_vendor, variant='rc' | 'autonomy')`. The two builds differ only in:

- the femur servo, STS3215 or STS3250 (same housing, so the same printed parts);
- the brain module shown on the top mount, or the turret example in its place.

The printed parts, STLs, wiring and firmware are identical.

**Where each document draws its numbers from.**

| Topic | RC build | Autonomy build |
| --- | --- | --- |
| Load check | DESIGN.md §3, "RC build" table (femur 1.39× rated, attachments ≤ 0.2 kg) | DESIGN.md §3, main table (femur 2.04×, attachments ≤ 0.4 kg) |
| Parts and cost | ELECTRONICS.md, rows without *autonomy upgrade*; §8 left column | All rows; §8 right column |
| Simulator (`src/robots/hexapod.config.js`) | `build: 'rc'`, `massKg: 2.93` | `brainModule` adds 0.30 kg |
| MuJoCo (`autonomy/mujoco`) | not used | Generated from `build_report_autonomy.json` |

**Upgrading an RC robot to autonomy.**
1. Swap the six femur servos for STS3250s, keeping the IDs 2, 5, 8, 11, 14, 17.
2. Print and fit the brain module (STLs `BrainBase`, `BrainLid`, `BrainCamMount`), and plug it into J1.
3. Re-run the femur trim (BUILD.md).

About US$810.

## Hexapod Mini

| Version | What it is | Git tag | Files | Fusion cloud |
| --- | --- | --- | --- | --- |
| Mini v1 | Separate small remote-control hexapod: 18 × MG90S, ESP32 + Bluetooth gamepad, 2 × 18650. About US$196 | `mini-v1` | `mini/` (notebook entry 19) | Not saved yet (local `.f3d` in `mini/cad`) |

## Getting an old version back

```
git checkout mk1-revB -- robot-sim/cad      # just the CAD folder as it was at rev B
```

In Fusion, open the document's **Versions** list in the Data Panel and choose **Promote** on the version you want.
