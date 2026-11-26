# Building the real hexapod (Mk1 rev B)

The simulator's numbers come from [src/robots/hexapod.config.js](../src/robots/hexapod.config.js), and the Fusion model (**Hexapod Mk1**) is generated from the same numbers. If you change a part, update the config so the digital twin keeps matching the robot.

- [DESIGN.md](DESIGN.md): parts, load check, joint limits, attachment interface.
- [ELECTRONICS.md](ELECTRONICS.md): shopping list.
- [notebook/](notebook/): how the design got here.

## Key dimensions

| Part | Value |
| --- | --- |
| Leg (joint to joint) | coxa 45, femur 75, tibia 120 mm (tibia to the floor, including the 3 mm boot) |
| Leg mounts (x, z) | front ±60, +110 · middle ±80, 0 · rear ±60, −110 mm; splay 35° front/rear |
| Body plates | 137 mm wide, 5 mm thick, gap 30.4 mm (= the servo case), each split in two |
| Coxa-joint height when walking | 90 mm (`standHeight`); lowest part 38.5 mm below that |
| Stance (load limit) | every stance foot 80–115 mm from its coxa axis (`footReachMin/Max`) |
| Mass | about 2.86 kg without an attachment |

## Servo IDs

Bus servos are addressed by ID, not by channel. **Set each servo's ID before assembly** with Waveshare's or Feetech's tool (the driver board's web page, or FD.exe from Feetech), one servo on the bus at a time, and write the ID on the case.

| Leg | Coxa | Femur (STS3250) | Tibia |
| --- | --- | --- | --- |
| 0 Front right | 1 | 2 | 3 |
| 1 Middle right | 4 | 5 | 6 |
| 2 Rear right | 7 | 8 | 9 |
| 3 Front left | 10 | 11 | 12 |
| 4 Middle left | 13 | 14 | 15 |
| 5 Rear left | 16 | 17 | 18 |
| Attachment servos | 21–25 | | |

While you have each servo on the bus, also centre it (position 2048) before fitting its horn.

## Assembly order

1. **Coupons first.** Print one servo pocket (both clamp faces) and the dovetail. Check the fit, then adjust `fit_clearance` and re-run the generator if needed.
2. **Inserts.** Set every heat-set insert in every printed part.
3. **Servos.** Set the IDs and centre every servo (above).
4. **Bottom plate.**
   1. Join the two halves at the jigsaw keys.
   2. Fit the battery tray (4 × M3).
   3. Fit the driver board, the ACS724 and the two side hubs (M2.5 self-tapping).
   4. Fit the XT60 pair and the fuse holder.
5. **Coxa servos.** Drop the six STS3215s onto the bottom plate: label boss up, pad in its pocket. Screw them in from below (2 × M2 × 8 each).
6. **Top plate.**
   1. Join the halves.
   2. Hang the electronics carrier underneath, with the MOSFET switch and 5 V buck on it.
   3. Lower the top plate onto the pillars and coxa servos, and fit 6 × M3 into the pillars and 2 × M2 into each servo.
   4. Fit the rear panel, the front bracket and the QR receiver.
7. **Body wiring.** Wire the body harness (diagram below) and route every lead as in the model. Check every rail with a bench supply before connecting the battery.
8. **Legs**, each identical:
   1. Fit the horns at centre.
   2. Bolt CoxaMain onto the coxa servo horns: top arm 4 × M3 × 8, bottom arm 4 × M3 × 8.
   3. Fit the femur servo into CoxaMain, then fit CoxaCap (3 × M3 × 10, 4 × M2 × 8).
   4. Fit the femur onto the femur servo's horns, then the tibia servo into the femur (4 × M2 × 8).
   5. Fit the tibia onto the tibia servo's horns, then the boot.
   6. Fit the coxa hub on the coxa's top arm and run its three leads.
9. **Battery and strap.** Fit the battery and strap, put the robot on a stand, and do the calibration below.

## Wiring and power

```
                 REAR PANEL (top plate)                                       FRONT BAY
   ┌──────────────────────────────────┐
   │ JST-XH 4 bal.   XT60 (charge)   rocker │
   └────┬─────────────────┬─────────────┬───┘
        │ balance ext.    │ 2 x 14 AWG  │ control (thin)
        │ (over battery)  │             └──────────────────────────────┐
  ┌─────┴──────────┐      │                                             │
  │ 3S LiPo 4000   │═XT60═╪═XT60══ 20 A ATC fuse ── ACS724 ±30 A ── Pololu MOSFET switch HP
  │ (slides out    │      └──────────────(battery node)    │ out→GPIO35 (10k/20k)    │ switched VBAT
  │  the rear)     │                                        │                         ├── Servo Driver w/ ESP32 (DC jack) ── USB-C ── PC / sim
  └────────────────┘                                        │                         │       │ bus DATA (GPIO18/19)
                                                            │                         ├── 5 V buck ── HC-SR04, ACS724, LiDAR
                                                            │                         ├── VBAT sense 100k/22k → GPIO34
                                                            │                         └── bus trunks (VBAT + GND + DATA) ─┬─ right hub ─ coxa servos 0,1,2 ; J1 (5 A polyfuse)
                                                                                                                       └─ left hub  ─ coxa servos 3,4,5
   each leg:  coxa servo ── coxa hub (on the coxa bracket) ──┬── femur servo
                                                             └── tibia servo
```

- **One ground.** Everything shares the battery negative. The bus leads carry GND + VBAT + DATA (Feetech 3-pin).
- The driver board's own servo connector carries data only to the trunks. **Do not** take the servos' power through the driver board's DC jack; the hubs feed VBAT straight from the switch output.
- Use 14 AWG for every battery-current wire. Each bus trunk feeds 9 servos, so make the trunks from 18–20 AWG leads (not the thin stock leads).
- **Routing.** Follow the Fusion model (`Harness …` components). Where a lead crosses a joint, clip it on both sides of the axis, leave a 30 mm loop at the axis, and zip-tie it at the bottom-plate slots beside each coxa servo.

## Charging (inside the robot or out)

The rear panel brings the pack's main leads (XT60) and balance lead (JST-XH 4) out, so you can charge without opening the robot. You can also slide the pack out of the rear for a bag charge or a swap.

1. **Switch the robot OFF** (rocker), put it on a non-flammable surface (tile, concrete, a metal tray), away from anything that burns, and **stay with it**.
2. Plug the charger's XT60 into the panel XT60, and its balance lead into the panel balance socket.
3. Set the charger to **LiPo, 3S, BALANCE charge, 2.0 A** (0.5C; never above 4 A).
4. Stop if the pack gets hot, smells, or puffs. Unplug and move it outside.
5. Unplug the balance lead last. For storage longer than a week, use the charger's STORAGE mode (3.8 V/cell).

Never charge a damaged or puffy pack. Never leave a charging LiPo unattended. **Charging in a LiPo bag outside the robot is the safest option**; it's the one to use whenever an adult isn't in the room.

## Hooking the simulator to the robot

1. Install the Arduino ESP32 core and Waveshare's **SCServo** library (from the ST3215 wiki, `ST_Servo.zip`). Flash [firmware/hexapod_bridge/hexapod_bridge.ino](../firmware/hexapod_bridge/hexapod_bridge.ino) to the Servo Driver with ESP32 (board: "ESP32 Dev Module").
2. Put the robot on a stand, feet off the ground. Switch on.
3. Open the simulator in Chrome or Edge, start Free drive, press `T`, then **Connect robot (USB)** and pick the driver's port.
4. Walk the robot in the simulator. The real one copies it.

**USB protocol (unchanged from rev A).**

- Simulator → robot, 25 lines/s: `S,<deg 0>,<deg 1>,…\n`. Whole degrees 0–180, 90 = centre. The first 18 values are the legs (leg 0 coxa, femur, tibia, then leg 1, …), followed by up to 5 attachment servos.
- Robot → simulator, 2 lines/s (new): `B,<battery mV>,<current mA>,<hottest servo °C>,<OK|LOW|NOLINK>\n`.
- If the link stops for 0.5 s, all servos relax.
- If the pack stays below 10.2 V for 3 s, the servos relax and motion commands are ignored until it reads above 11.0 V again.

## Calibration ("trim")

With the robot on a stand:

1. Connect and stand still in the simulator.
2. Each leg should match the sim: coxa straight out, femur level, tibia pointing straight down.
3. If a joint moves the wrong way, flip its sign in `servos.direction` in `hexapod.config.js`. The bus servos' direction depends on which way each one is mounted, so expect to flip some.
4. Put any remaining offset in degrees into `servos.trim`.

## Safety

- Hold the robot or use a stand the first time you connect. STS3250s are strong: 50 kg·cm will pinch fingers.
- Switch off before working on the robot. Unplug the battery when you're done.
- Only use the 20 A fuse. Never bridge it.
- The "toy laser" is a laser-tag style IR LED plus a red LED, never a real laser. The "potato cannon" is a spring or flywheel **foam-ball** launcher, not a combustion cannon.
