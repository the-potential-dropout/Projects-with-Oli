# Hexapod Mini firmware

`mini_hexapod/` is the whole robot program for the ESP32-DevKitC:

- leg inverse kinematics;
- a tripod gait with the stance-window rule from the load check;
- Bluetooth gamepad control (Bluepad32);
- two PCA9685 servo boards;
- battery monitoring and failsafes;
- the Robot Sim's USB `S,` protocol.

| File | What it is |
| --- | --- |
| `mini_hexapod.ino` | Main loop: gamepad, modes (limp, standing up, walk, sitting down, USB drive), failsafes, servo output, telemetry |
| `kinematics.h/.cpp` | IK and tripod gait in plain C++, the same maths and conventions as `robot-sim/core/src/ik.rs` and `gait.rs` |
| `config.h` | Geometry (matches the CAD), stance window, joint limits, channel map, servo directions and trims, pins, battery thresholds |
| `../test/test_mini.cpp` | Host tests, run on a PC: IK round trip, tibia fold limit, standing pose, gait (tripods never both up, stance window, coxa range, comes to rest), servo range, channel map |

## Build and flash

1. **Arduino IDE 2.x.** In *File > Preferences > Additional boards manager URLs*, add:
   `https://raw.githubusercontent.com/ricardoquesada/esp32-arduino-lib-builder/master/bluepad32_files/package_esp32_bluepad32_index.json`
2. In *Boards Manager*, install **esp32_bluepad32**. Select *Tools > Board > esp32_bluepad32 > ESP32 Dev Module*.
3. In *Library Manager*, install **Adafruit PWM Servo Driver Library**. Bluepad32 comes with the board package.
4. Open `mini_hexapod/mini_hexapod.ino` and upload. Flash the board **before** it goes on the robot. On some DevKitC boards you have to hold **BOOT** while the upload starts, and once the board is mounted pins-up the button faces down. To re-flash later, cut the zip tie and lift the board, or just try it in place: the DevKitC's auto-reset usually works.

## Host tests

```
cd robot-sim/mini/firmware/test
g++ -std=c++17 -O2 -I../mini_hexapod test_mini.cpp ../mini_hexapod/kinematics.cpp -o test_mini
./test_mini
```

On Windows, the repo path can be longer than g++ accepts. If the compiler says "No such file", copy `mini_hexapod/` and `test/` to a short folder first.

## Controls

| Input | Action |
| --- | --- |
| A (Cross) | Arm: the servos switch on and the robot stands up (1.5 s) |
| B (Circle) | Finish the step, sit down, servos off |
| Left stick | Walk forward/back and strafe |
| Right stick X | Turn |
| D-pad up/down | Body height 35–55 mm |
| R1 (held) | Fast steps (2.8 Hz instead of 2.0 Hz) |

**Failsafes.**

| Condition | What the robot does |
| --- | --- |
| Pad silent for 0.3 s | Stops walking |
| Pad silent for 3 s | Sits down and goes limp |
| Battery below 6.6 V for 3 s | Sits down and goes limp. It won't re-arm until the battery reads above 7.0 V |
| Battery below 6.9 V | Short rumble every 5 s |

**Limp.** "Limp" means the PCA9685 **OE** pin is held high, so there are no servo pulses at all. The robot also starts limp at power-up.

## Driving it from the Robot Sim (USB)

The firmware speaks the same lines as `robot-sim/firmware/hexapod_bridge`:

| Line | Direction | Meaning |
| --- | --- | --- |
| `S,<a0>,…,<a17>` | sim → robot | Servo degrees 0–180, in the order leg 0..5 × (coxa, femur, tibia). It drives the servos directly. The firmware adds only its trims. |
| `A,1` / `A,0` | sim → robot | Arm / disarm USB mode. Ignored while a gamepad has the robot armed, and while the battery is low. The servos go limp 0.5 s after the last `S` line. |
| `B,<mV>,0,0,<OK\|LOW\|NOLINK>` | robot → sim | Battery telemetry, twice a second |

The simulator's `hexapod.config.js` describes Mk1, so the sim needs this robot's numbers to drive it:

- **Geometry:** `leg: {coxa: 0.030, femur: 0.044, tibia: 0.060}`.
- **Mounts:** `mounts` = the `MOUNT_X`/`MOUNT_Z` pairs in `config.h`; `legSplay` = 40, 0, −40, 40, 0, −40.
- **Direction:** `servos.direction` = −1 for every joint of every leg. The parts are rotated copies, not mirror images.
- **Mapping:** the same as Mk1, `90 + dir × angle`, with the tibia offset by 90°.

## What has been checked, and what hasn't

- **Checked:**
  - The kinematics and gait pass the host tests.
  - The sketch passes a syntax and type check against stub headers that copy the Bluepad32, Adafruit and ESP32 calls it uses.
- **Not checked:**
  - The sketch has not been compiled against the real ESP32/Bluepad32 core.
  - It has not been run on hardware.
  - The servo directions (`DIR`) are an assumption about the MG90S. BUILD.md step 9 checks them on the stand.
