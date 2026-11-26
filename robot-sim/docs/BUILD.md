# Building the real hexapod (Mk1 rev B2)

The simulator's numbers come from [src/robots/hexapod.config.js](../src/robots/hexapod.config.js), and the Fusion model (**Hexapod Mk1**) is generated from the same numbers. If you change a part, update the config so the digital twin keeps matching the robot.

| Document | What's in it |
| --- | --- |
| [DESIGN.md](DESIGN.md) | Parts, load check, joint limits, attachment interface |
| [KINEMATICS.md](KINEMATICS.md) | Why the legs are 3-DOF |
| [CONTROL.md](CONTROL.md) | Gait, balance, terrain, safety, tuning |
| [WIRING.md](WIRING.md) | Every wire: power tree, sizes, connectors, pin maps, **cut list**, when each cable goes in |
| [ELECTRONICS.md](ELECTRONICS.md) | Shopping list |
| [AUTONOMY.md](AUTONOMY.md) | Radio, brain module, ROS 2, MuJoCo sim2real |
| [notebook/](notebook/) | How the design got here |

## Key dimensions

| Part | Value |
| --- | --- |
| Leg (joint to joint) | coxa 45, femur 75, tibia 120 mm (tibia to the floor, including the 3 mm boot) |
| Leg mounts (x, z) | front ±60, +110 · middle ±80, 0 · rear ±60, −110 mm; splay 35° front/rear |
| Body plates | 137 mm wide, 5 mm thick, gap 30.4 mm (= the servo case), each split in two |
| Coxa-joint height when walking | 90 mm (`standHeight`), 105 mm in terrain mode. Lowest part 38.5 mm below it (coxa bracket); the IMU pod is 12 mm under the belly |
| Stance (load limit) | every stance foot 80–115 mm from its coxa axis (`footReachMin/Max`); the firmware enforces it |
| Mass | RC build about 2.93 kg without an attachment (attachments ≤ 0.2 kg). Autonomy build 2.96 kg; the brain module adds 0.30 kg |

## Servo IDs

Bus servos are addressed by ID, not by channel. **Set each servo's ID before assembly** with Waveshare's or Feetech's tool: the bridge board's stock firmware web page, or FD.exe from Feetech. Do one servo on the bus at a time, and write the ID on the case.

| Leg | Coxa | Femur (STS3215 RC / STS3250 autonomy) | Tibia |
| --- | --- | --- | --- |
| 0 Front right | 1 | 2 | 3 |
| 1 Middle right | 4 | 5 | 6 |
| 2 Rear right | 7 | 8 | 9 |
| 3 Front left | 10 | 11 | 12 |
| 4 Middle left | 13 | 14 | 15 |
| 5 Rear left | 16 | 17 | 18 |
| Attachment servos | 21–25 | | |

While each servo is on the bus, also centre it (position 2048) before fitting its horn.

## Firmware

| Board | Sketch | Arduino board setting |
| --- | --- | --- |
| Waveshare Servo Driver with ESP32 | [firmware/bus_bridge](../firmware/bus_bridge) | ESP32 Dev Module. **Set the servo IDs first** with the stock firmware, then flash this |
| Adafruit Feather ESP32-S3 | [firmware/hexapod_controller](../firmware/hexapod_controller) | Adafruit Feather ESP32-S3 (No PSRAM / 2 MB PSRAM), USB CDC on boot enabled. Libraries: SCServo (Waveshare `ST_Servo.zip`), Adafruit BNO08x, VL53L1X (Pololu) |

**Host tests** of the control maths need no hardware: `g++ -std=c++17 -I firmware/hexapod_controller firmware/test/test_core.cpp firmware/hexapod_controller/kinematics.cpp` and run the result. On Windows, build from a short path.

[firmware/hexapod_bridge](../firmware/hexapod_bridge) is the rev B single-board sketch. It still works if you only want the simulator to drive the servos directly, with no balance, radio or sensors.

## Assembly order

1. **Coupons first.**
   1. Print one servo pocket (both clamp faces), the dovetail, and one tibia tip with its boot.
   2. Check the fits. The boot must slide 1 mm and click the switch.
   3. Adjust `fit_clearance` and re-run the generator if needed.
2. **Inserts.** Set every heat-set insert in every printed part.
3. **Servos.** Set the IDs and centre every servo (above).
4. **Bottom plate.**
   1. Join the two halves at the jigsaw keys.
   2. Fit the battery tray (4 × M3).
   3. Fit the bridge board, the ACS724 and the two side hubs. **Solder the hubs' 18 AWG trunks, 1000 µF capacitors and 3 A PTCs first.**
   4. Fit the XT60 pair and the fuse holder.
   5. Fit the IMU pod underneath, with the BNO085 on TPU grommets. Thread its lead up through the plate.
5. **Coxa servos.** Drop the six coxa servos onto the bottom plate: label boss up, pad in its pocket. Screw them in from below (2 × M2 × 8 each). Plug in each `Bus coxaN` lead and tie it at the plate slots.
6. **Top plate.**
   1. Join the halves.
   2. Hang the electronics carrier underneath, with the MOSFET switch and 5 V buck on it.
   3. Run the bridge-UART, Feather-5 V and IMU leads up through the plate holes at (±20, 38) and (8, 34), **before** lowering the plate.
   4. Lower the top plate onto the pillars and coxa servos, and fit 6 × M3 into the pillars and 2 × M2 into each servo.
   5. Fit the rear panel, the front bracket (with the HC-SR04 and the VL53L1X on its wedge), and the QR receiver (with J1).
7. **Control pod.**
   1. Screw the pod to the plate's grid inserts.
   2. Fit the Feather (USB-C forward) and the receiver (foam tape).
   3. Solder every lead to the Feather pads; WIRING.md has the map.
   4. Fit the antenna in the lid post and close the lid.
8. **Body wiring.** Do the remaining main-path wiring as WIRING.md §8. **Check every rail with a bench supply (12 V, 3 A limit) before the battery.**
9. **Legs**, each identical:
   1. Fit the horns at centre.
   2. Fit the foot switch in the tibia tip (2 × M2 × 10 + nuts), solder its lead, and push the boot on until it clicks into its grooves.
   3. Bolt CoxaMain onto the coxa servo horns: top arm 4 × M3 × 8, bottom arm 4 × M3 × 8.
   4. Fit the femur servo into CoxaMain, then fit CoxaCap (3 × M3 × 10, 4 × M2 × 8).
   5. Fit the femur onto the femur servo's horns, then the tibia servo into the femur (4 × M2 × 8).
   6. Fit the tibia onto the tibia servo's horns.
   7. Fit the coxa hub on the coxa's top arm and run its three bus leads.
   8. Lay the foot lead in the tibia groove, snap it into the femur fin, cross the coxa axis above the hub, then lay it over the top plate. Tie it in the slots and pass it into the pod's side wall.
10. **Battery and strap.** Fit the battery and strap, put the robot on a stand, and calibrate (below).
11. **Brain module** (optional).
    1. Fit the Pi 5 and its cooler, the buck, and the OAK-D Lite on its mount. Fit the camera screws before the mount.
    2. Fit the LD19 on the lid.
    3. Slide the module onto the dovetail, plug it into J1, and fit the lock screw through the lid hole.

## Charging (inside the robot or out)

The rear panel brings the pack's main leads (XT60) and balance lead (JST-XH 4) out, so you can charge without opening the robot. You can also slide the pack out of the rear for a bag charge or a swap.

1. **Switch the robot OFF** (rocker). Put it on a non-flammable surface (tile, concrete, a metal tray), away from anything that burns, and **stay with it**.
2. Plug the charger's XT60 into the panel XT60, and its balance lead into the panel balance socket.
3. Set the charger to **LiPo, 3S, BALANCE charge, 2.0 A** (0.5C; never above 4 A).
4. Stop if the pack gets hot, smells or puffs. Unplug it and move it outside.
5. Unplug the balance lead last. For storage longer than a week, use the charger's STORAGE mode (3.8 V/cell).

Never charge a damaged or puffy pack. Never leave a charging LiPo unattended. **Charging in a LiPo bag outside the robot is the safest option**; use it whenever an adult isn't in the room.

## Radio

Set up the RadioMaster as in [radio/README.md](../radio/README.md): CRSF/ELRS model, 150 Hz, the channel map, and the HexaMk1 widget.

| Switch | Job |
| --- | --- |
| **SA** | Arms |
| **SF** | Kills (always works) |
| **SC** | Manual / pose / auto |
| **SB** | Gait |
| **SD** | Terrain mode |

## Hooking the simulator to the robot

1. Flash both boards (above).
2. Put the robot on a stand, feet off the ground. Switch on.
3. Open the simulator in Chrome or Edge, start Free drive, press `T`, then **Connect robot (USB)** and pick the **Feather's** port.
4. **Without a radio**, the robot only obeys the PC once it has received `A,1` (arm). **With a radio**, put it in **auto**.
5. Walk the robot in the simulator. The real one copies it.

**USB protocol.** Details are in CONTROL.md §6.

| Line | Direction | Rate | Status |
| --- | --- | --- | --- |
| `S,<deg 0>,…` | sim → robot | 25 lines/s | Unchanged. Joint bridge: the sim drives every servo |
| `C,vx,vz,yaw,roll,pitch,height,gait,flags` | sim → robot | ≥ 5 lines/s | **New** command mode: the robot does its own gait, balance and terrain |
| `R,<18 residuals>` | sim → robot | | **New**: learned-policy corrections |
| `A,0/1` | sim → robot | | **New**: disarm / arm (only without a radio) |
| `B,<mV>,<mA>,<°C>,<OK\|LOW\|NOLINK>` | robot → sim | 2 Hz | Unchanged |
| `T,roll,pitch,yaw,feet,margin_mm,mode,source,lq` | robot → sim | 10 Hz | **New**: attitude, foot switches, stability margin, who is driving |

Safety behaviour is in CONTROL.md §9:
- If the link stops for 0.5 s, the joint bridge relaxes and command mode stops.
- If the pack stays below 10.2 V for 3 s, the servos relax until it reads above 11.0 V.

## Calibration ("trim") and tuning

On a stand:
1. Connect and stand still in the simulator (joint bridge).
2. Each leg should match the sim: coxa straight out, femur level, tibia pointing straight down.
3. If a joint moves the wrong way, flip its sign in `servos.direction` in `hexapod.config.js` **and** in `DIR` in `firmware/hexapod_controller/config.h`.
4. Put any remaining offset into `trim` (both files).
5. Then follow CONTROL.md §12: IMU signs → feet → stand → balance gains → gait → terrain.

## Safety

- Hold the robot or use a stand the first time you connect. Bus servos are strong: 30 kg·cm (STS3215) or 50 kg·cm (STS3250) will pinch fingers.
- The radio's **kill switch (SF)** cuts the servos instantly. Test it every session.
- Switch off before working on the robot. Unplug the battery when you're done.
- Only use the 20 A fuse. Never bridge it or the PTCs.
- The "toy laser" is a laser-tag style IR LED plus a red LED, never a real laser. The "potato cannon" is a spring or flywheel **foam-ball** launcher, not a combustion cannon.
