# Radio: RadioMaster TX16S MK3 (EdgeTX + ExpressLRS) → Hexapod Mk1

**Radio.** The robot is driven by any **EdgeTX** radio with an **ExpressLRS** link. Both are fully open source, and the radio runs Lua scripts.
- **Recommended:** the RadioMaster TX16S MK3. It has a big colour touch screen, Hall gimbals, ELRS built in, and costs about $230 for the standard model or $270 for the MAX.
- **Cheaper options:** the RadioMaster Pocket or Boxer, using the black-and-white script.

**Receiver.** A **RadioMaster RP1 V2** (2.4 GHz ELRS, about $19) sits in the robot's control pod and speaks CRSF to the Feather ESP32-S3 at 420000 baud. The reasons for this choice are in [docs/AUTONOMY.md](../docs/AUTONOMY.md) §2.

## Files

| File | Where it goes on the radio's SD card | What it does |
| --- | --- | --- |
| `WIDGETS/HexaMk1/main.lua` | `/WIDGETS/HexaMk1/main.lua` | Colour widget (TX16S): battery V, A and %, flight mode and faults, link quality, artificial horizon, low-battery beep |
| `SCRIPTS/TELEMETRY/hexa.lua` | `/SCRIPTS/TELEMETRY/hexa.lua` | The same on black-and-white radios (Pocket, Boxer) |

## Model setup

**Link and telemetry**
1. Create a new model.
2. Set Internal RF to CRSF / ExpressLRS. **Packet rate 150 Hz** (plenty for walking; longer range), telemetry ratio 1:8.
3. Bind the RP1: power the robot within 30 s of power-up three times, or use a binding phrase. Update the RP1 to the same ELRS major version as the radio first.
4. On the Telemetry page, run **Discover new sensors**. You should get RxBt, Curr, Capa, Bat%, Ptch, Roll, Yaw, FM, RQly, RSNR.
5. Set a **telemetry-lost alarm** and RxBt alarms: warn at 10.5 V, critical at 10.2 V.

**Mixer (AETR order)**

| Ch | Source | Robot meaning (firmware `readRc()`) |
| --- | --- | --- |
| 1 | Ail (right stick ←→) | strafe left / right (`vx`), or **roll** in pose mode |
| 2 | Ele (right stick ↑↓) | walk forward / back (`vz`), or **pitch** in pose mode |
| 3 | Thr (left stick ↑↓, no centre spring) | **body height** 60–110 mm |
| 4 | Rud (left stick ←→) | turn (`yaw rate`), or **body yaw** in pose mode |
| 5 | SA (2-pos) | **ARM**. The robot only moves when armed. |
| 6 | SB (3-pos) | gait: tripod / ripple / wave |
| 7 | SC (3-pos) | mode: **manual** walk / **pose** (sticks tilt the body) / **auto** (brain or PC drives; any stick input takes over) |
| 8 | SD (2-pos) | terrain mode (touchdown search, soft servos, higher steps) |
| 9 | SH (momentary) | fire the attachment (toy IR "laser" / foam launcher) |
| 10 | SF (2-pos) | **KILL**: servos limp instantly, overrides everything |

Set the **failsafe** to "No pulses" on the radio's ELRS module. When the link drops, the robot stops sending its gait. After 250 ms without RC frames the firmware falls back to the next source, or disarms and goes limp in a crouch when there is no other source.

## Control rules (on the robot)

1. KILL and the low-battery cut-out always win.
2. If the radio link is up, the radio arms and disarms. In **auto**, commands come from the brain module (or the PC). Moving any stick returns control to the radio at once.
3. With no radio, only a USB-connected PC (the simulator) can drive, after it sends `A,1`. The brain can't drive without a radio, because the radio is the e-stop.

## Coding your own screens

EdgeTX Lua reference: https://luadoc.edgetx.org. A good first project is to add a "Gait" line that reads the `FM` string and highlights the active gait. The robot sends `MAN`, `AUTO`, `SAFE` or `LOWBAT`, with a `!` added on any fault.
