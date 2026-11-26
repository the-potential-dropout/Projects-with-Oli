# Hexapod Mini: build guide

This guide is written for Oli, with an adult nearby for the soldering and the first battery connection. The robot is a toy: there are no weapons and nothing that launches.

**Time:**

- about 14 h of printing;
- an afternoon of soldering;
- an evening of assembly and calibration.

**Parts:** see [BOM.md](BOM.md).
**Wiring:** see [WIRING.md](WIRING.md).
**Why it is built this way:** see [DESIGN.md](DESIGN.md).

## Safety first

- **Lithium cells.**
  - Charge them only in the XTAR charger, out of the robot, on a hard non-flammable surface, and not overnight.
  - Throw away (recycle) a cell that is dented, hot or has a torn wrapper.
  - Never short a cell. Keep loose cells in a plastic case, not a pocket.
- **Soldering iron:** 350 °C. Use a stand, work in a ventilated space and wear eye protection when clipping leads.
- **Pinch points.** A servo is stronger than it looks. Keep fingers out of the joints while the robot is armed. Disarm (B on the gamepad) or switch off before touching the legs.
- **Kill.** Flick the rocker switch off. Pressing B sits the robot down and turns the servos off.

## 1. Print

**Settings:**

- PLA, 0.2 mm layers, 3 walls, 25 % gyroid infill;
- no supports;
- brim only if your bed needs it.

Every STL in [`../cad/stl/`](../cad/stl/) is already in print orientation, flat face down.

| Part | Qty | Notes |
| --- | --- | --- |
| Tray | 1 | the largest part, about 146 × 170 mm: fits a 180 mm bed |
| Deck | 1 | |
| BatteryHatch | 1 | |
| Coxa | 6 | |
| Femur | 6 | |
| Tibia | 6 | prints on its side; the foot is the round end |

**Before printing the batch, test the fit:**

1. Print **one coxa link**.
2. Check that a servo drops into its pocket with a light push. It should not be loose, and it should not need force.
3. Check that the servo's tabs sit flat on the seat.
4. If the fit is wrong, change `fit_clearance` (Fusion: Modify > Change Parameters) and re-run the generator. MG90S cases differ by ±0.2 mm between makers.

## 2. Prepare the servos (before assembly)

1. Plug each servo into the PCA9685 R, channel 0, with the board powered from a Mini560 (or a 5 V bench supply).
2. Flash the firmware (step 8). Then, with no gamepad connected, send over USB:
   `A,1` then `S,90,90,90,90,90,90,90,90,90,90,90,90,90,90,90,90,90,90`
3. Every servo goes to **90°** (centre).
4. Fit the horn **pointing along the link** at centre:
   - coxa horn pointing straight out from the body;
   - femur horn along the femur (horizontal);
   - tibia horn along the tibia (straight down).
5. Put a dot of paint on each servo so you know it has been centred.

## 3. Solder the power harness

Follow the wire list in [WIRING.md](WIRING.md) §3. Cut each wire about 15 % longer than the CAD length.

1. Holder leads → BMS B+ and B−. Solder a short wire from the holder's middle contact to BMS BM (the mid-tap).
2. BMS B+ (P+) → fuse holder → switch → both Mini560 IN+. BMS P− → both Mini560 IN−.
3. Mini560 OUT+ / OUT− → the PCA9685 V+ / GND screw terminals: right buck to the right board, left buck to the left board.
4. Fit the Schottky diode (band towards the ESP32) in a wire from the right board's V+ terminal to the ESP32 **5V** pin.
5. Make the battery-sense divider:
   - 100 kΩ from BMS P+ to the GPIO34 lead;
   - 47 kΩ from the GPIO34 lead to GND.
   Heat-shrink it as one small block.
6. **Test before any servo is connected.**
   1. Fit the fuse and both cells.
   2. Switch on.
   3. Check that each Mini560 output reads 5.0 ± 0.2 V.
   4. Check that the ESP32 5V pin reads about 4.7 V (the diode drops about 0.3 V).
   5. Switch off.

## 4. Body

1. Bridge the **A0** pads on the left PCA9685 (0x41).
2. Screw both boards onto the deck standoffs, terminal blocks facing each other in the middle (M2.5 × 6).
3. Stick both Mini560s under the deck, at the rear (foam tape), and the BMS under the deck at the front.
4. Lay the fuse holder on the tray floor at the rear left. Clip the switch into its deck cut-out.
5. Put the ESP32 on its cradle at the front of the deck, **pins up**, USB to the front. Hold it with a zip tie through the guides.
6. Screw the battery holder to the hatch (2 × M3 countersunk). Screw the hatch on (2 × M3 × 6).
7. Drop the six coxa servos into their rings in the tray, shaft up, leads towards the middle.
8. Lay the deck over the servo tabs. Screw the tabs through the deck with M2 × 8 and nuts.
9. Screw the deck to the tray (4 × M3 × 10).

## 5. Legs

Do one leg at a time. All six legs are the same parts.

1. **Femur servo into the coxa link.**
   1. Push the servo into the pocket, lead first, so the lead comes out of the channel at the bottom.
   2. Screw the tabs with 2 self-tapping screws.
2. **Tibia servo into the femur.**
   1. Push it in through the tab plate.
   2. Screw the tabs.
   3. Thread its lead out through the slot in the femur's cradle.
3. **Femur onto the femur servo.**
   1. Place the horn arm on the femur servo's horn (servo centred, femur horizontal).
   2. Screw the horn with 2 × M2 × 5 and fit the horn centre screw through the access hole.
   3. Fit the **M3 × 6 idler pin** through the femur's idler arm into the coxa link. It self-taps into the femur and spins in the coxa link. **Snug, not tight:** the joint must swing freely by hand.
4. **Tibia onto the tibia servo.** Same as the femur, tibia pointing straight down.
5. **Coxa link onto the coxa servo.**
   1. Horn arm on the coxa horn, leg pointing straight out.
   2. Fit the M3 × 6 pivot pin up through the lower arm into the tray floor.
6. **Leads.** Route them as in [WIRING.md](WIRING.md) §2: over the top of the coxa link and down its outer side. Leave a 30 mm loop at each joint. Plug each lead into its channel from the table (brown to GND).

## 6. Gamepad

Pairing an **8BitDo Lite 2**:

1. Set its mode switch to **S** (Switch mode).
2. Power the robot.
3. Hold the pad's **pair** button for 3 s until its LEDs run.
4. The robot's ESP32 LED blinks while disarmed.

Other pads: put a DualShock 4 in pairing mode with Share + PS; a Switch Pro pad with the sync button. The robot takes the first pad that connects.

## 7. Battery voltage check

1. Switch on and open the Arduino Serial Monitor at 115200.
2. The robot prints `B,<mV>,0,0,<state>` twice a second.
3. Compare the number with a multimeter on the battery.
4. If it is more than 0.1 V out, set `VBAT_CAL` in `config.h` (measured ÷ printed) and re-flash.

## 8. Firmware

See [../firmware/README.md](../firmware/README.md):

1. Install the Bluepad32 board package and the Adafruit PWM Servo Driver library.
2. Select **ESP32 Dev Module** (esp32_bluepad32).
3. Upload `mini_hexapod.ino`.

## 9. Calibration on the stand

1. Stand the robot on a box (a mug works), so the feet hang free.
2. Arm with **A**. The robot "stands up" in the air.
3. **Direction check.** Push the left stick forward slightly. Watch one leg:

   | Joint | Expected movement |
   | --- | --- |
   | Coxa | swings the foot towards the front while that leg is on the ground (stance), backwards in the air |
   | Femur | lifts the leg when it swings |
   | Tibia | folds the foot in when the leg lifts |

   If a joint moves the wrong way on **every** leg, flip its sign in `DIR` in `config.h`.
4. **Trim.**
   1. Disarm.
   2. Send `A,1` and the all-90 `S` line from step 2 over USB.
   3. Look at each joint. Write the error into `TRIM[leg][joint]` (degrees) and re-flash: coxa should point straight out, femur level, tibia vertical.
5. **Servo period.** If any servo buzzes at rest, measure the PWM period. It should be 20 ms. If it isn't, adjust `setOscillatorFrequency` in `setup()` (the PCA9685 clock varies ±5 %).

## 10. First walk

1. Put the robot on a carpet or a rubber mat (PLA feet slip on glass and tiles).
2. Arm with **A**.
3. Walk slowly with the left stick. Turn with the right stick.
4. Hold R1 for faster steps. D-pad up/down changes the body height.
5. Watch for:
   - a servo getting **hot**, more than you can hold a finger on;
   - a leg **buzzing** while standing (trim it);
   - the body **sagging** (battery low, or a femur servo slipping on its horn spline: tighten the horn centre screw).
6. Press **B** to sit, then switch off.
