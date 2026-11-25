# Building the real hexapod

The simulator's numbers come from [src/robots/hexapod.config.js](../src/robots/hexapod.config.js). If you change a part, update that file so the digital twin keeps matching the robot. The Fusion model (**Hexapod Mk1**) is generated from the same numbers. See [DESIGN.md](DESIGN.md) for the printed parts, assembly order and attachment interface, and [ELECTRONICS.md](ELECTRONICS.md) for the shopping list.

## Key dimensions

| Part | Value |
| --- | --- |
| Body plates (top + bottom) | 260 × 140 mm core hexagon + 21 mm lobes round each coxa (262 × 202 mm overall), 5 mm thick, 45 mm apart, each split in two for printing |
| Coxa (hip) | 45 mm joint-to-joint |
| Femur (thigh) | 75 mm |
| Tibia (shin) | 120 mm |
| Standing body height | 90 mm |
| Front/rear leg splay | 35° |
| Leg mounts (x, z) | front ±60, +110 · middle ±80, 0 · rear ±60, −110 mm |

## Parts list (starting point)

| Qty | Part | Notes |
| --- | --- | --- |
| 18 | MG996R metal-gear servos (DS3218MG recommended for the 6 femurs) | Legs. The robot comes out at about 2.7 kg, which puts MG996R femurs at about 70 % of stall: see DESIGN.md §8 |
| 5 | MG90S micro servos | Robotic arm (base, shoulder, elbow, wrist, gripper) |
| 1 | MG90S | Turret pitch for the laser / potato cannon |
| 2 | PCA9685 16-channel PWM boards | Addresses 0x40 and 0x41 (solder A0 on the second one) |
| 1 | ESP32 dev board (or Arduino Mega) | Talks to the PCA9685s over I²C, to the simulator over USB |
| 1 | 2S LiPo 7.4 V, 2200 mAh + 5–6 V / 20 A UBEC | Servos draw a lot of current, so don't power them from USB |
| 1 | HC-SR04 ultrasonic sensor | "Sonar" front attachment |
| 1 | LD06 / RPLIDAR C1 | "LiDAR" front attachment (optional, pricier) |
| — | 3D-printed plates and leg parts (STLs in `cad/stl/`), M3 heat-set inserts and screws, 18 × 623ZZ bearings, 25T aluminium horns | Full list in ELECTRONICS.md |

The "toy laser" is a laser-tag style IR LED + red LED, never a real laser pointed at people. The "potato cannon" in real life is a spring or flywheel foam-ball launcher, not a combustion cannon.

## Servo channels

| Leg | Coxa | Femur | Tibia |
| --- | --- | --- | --- |
| 0 Front right | 0 | 1 | 2 |
| 1 Middle right | 4 | 5 | 6 |
| 2 Rear right | 8 | 9 | 10 |
| 3 Front left | 16 | 17 | 18 |
| 4 Middle left | 20 | 21 | 22 |
| 5 Rear left | 24 | 25 | 26 |

Channels 0–15 are on board 0x40 and 16–31 on board 0x41. Attachment servos use channels 12–15 and 28–31.

## Hooking the simulator to the robot

1. Flash [firmware/hexapod_bridge/hexapod_bridge.ino](../firmware/hexapod_bridge/hexapod_bridge.ino) to the ESP32. It needs the Adafruit PWM Servo Driver library.
2. Power the servos from the UBEC, **not** the USB cable. Connect the grounds together.
3. Open the simulator in Chrome or Edge on a computer, start Free drive, press `T`, then **Connect robot (USB)** and pick the ESP32's port.
4. Walk the robot in the simulator. The real one copies it.

The link sends one text line per frame, 25 times a second:

```
S,<servo 0>,<servo 1>,...,<servo N>\n
```

Each value is a whole number of degrees, 0–180. The first 18 values are the legs (leg 0 coxa, femur, tibia, then leg 1, and so on). Then come the attachment servos: turret pitch, or arm base, shoulder, elbow, wrist and gripper.

## Calibration ("trim")

With the robot on a stand (feet off the ground):

1. Connect and stand still in the simulator.
2. Each leg should look like the sim: coxa straight out, femur level, tibia pointing straight down.
3. Where a joint is off, put the correction in degrees into `servos.trim` in `hexapod.config.js`. If a joint moves the wrong way, flip its sign in `servos.direction`.

## Safety

- Hold the robot or put it on a stand the first time you connect.
- Keep fingers out of the legs while it's powered. MG996Rs are strong.
- Unplug the battery when you're done. LiPos need a proper charger and a fire-safe bag.
