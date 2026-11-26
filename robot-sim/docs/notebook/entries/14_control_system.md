# 14 · Control system on the robot
*2026-11-25*

**Brief.** Balance, walking, tilting, climbing up and down uneven ground, with every sensor and controller on the robot itself.

**Controller.** The rev B Waveshare board has too few free pins (one), so the main controller became an **Adafruit Feather ESP32-S3**. The Waveshare board stays as a servo-bus bridge that also reads the battery.

**Sensors.**
- **BNO085 IMU** under the belly, on grommets.
- **Omron D2F micro-switch** in every tibia tip, sealed by a sliding TPU boot with a plunger nub.
- **VL53L1X** looking 30° down from the front bracket.
- Bus-servo readback of position, temperature and load.

**Control layers** (all equations in [CONTROL.md](../../CONTROL.md)):
- the servos' internal PID, softened for terrain;
- IK identical to the simulator, plus a lift cap near the body;
- a tripod / ripple / wave gait with the stance-window rule (feet 80–115 mm);
- body pose;
- IMU balance (PI on attitude, D on the gyro);
- a stability margin that switches tripod → ripple below 25 mm;
- touchdown search and terrain following;
- safety: kill, low battery, tip, temperature, link loss.

**Verification** (host tests on the PC, `firmware/test`):

| Check | Result |
| --- | --- |
| IK ↔ FK | Round trip matches |
| Standing | All feet on the ground |
| Walking | The two tripods are never up together |
| Sprint + turn | Feet stay 86–115 mm out |
| Ripple / wave | At least 4 / 5 feet down |
| Tripod margin | 83 mm |
| 12° slope | Levelled to 0.05° in 4 s |
| CRSF | Frames round-trip; a bad CRC is rejected |

**What the tests changed.**
1. They found that a sprinting swing foot would need the tibia folded past its limit, which led to the lift cap.
2. The first balance gains took 3 s to level; Ki went from 0.6 to 2.0.

**Terrain limits from the geometry.**
- Step up **40 mm** (terrain mode, 105 mm body height).
- Step down 131.6 mm − body height (about 27 mm at the 105 mm terrain height). The sim found the femur's −10° limit sets this.
- Full levelling about 10° (balance output clamped there).

**Fixed after the sim review.** Touchdown heights are now taken in the IMU's level frame, so the gait can't teach itself the body's tilt.
- Slope 20° rated (static tipping at 29° in a tripod).
