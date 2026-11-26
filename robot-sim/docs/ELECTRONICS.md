# Hexapod Mk1 (rev B2): electronics and bought parts

Everything that isn't printed is a standard catalogue part that ships by normal post.

**Prices.** US$, checked on 2026-11-25 where a link is given. Prices marked **approx.** are typical street prices I couldn't confirm on a product page. Prices change; treat this as a budget, not a quote.

**Related documents.**
- Design and load check: [DESIGN.md](DESIGN.md).
- Leg architecture study: [KINEMATICS.md](KINEMATICS.md).
- Control system: [CONTROL.md](CONTROL.md).
- Every wire: [WIRING.md](WIRING.md).
- Autonomy and radio: [AUTONOMY.md](AUTONOMY.md).
- Assembly and charging: [BUILD.md](BUILD.md).

## 1. Servos

| Qty | Part | Key spec | ≈ Price | Where |
| --- | --- | --- | --- | --- |
| 12 (+1 spare) | **Feetech STS3215 / Waveshare ST3215, 12 V version** (C018/C047) | 30 kg·cm stall, 10 kg·cm rated, 2.7 A stall at 12 V, 45.22 × 24.72 × 35 mm, about 69 g, 12-bit magnetic encoder, TTL bus, metal horns included | $20–22 | [Waveshare ST3215 ($21.99)](https://www.waveshare.com/product/robotics/motors-servos/servos/st3215-servo.htm), [DFRobot SER0070 ($19.90)](https://www.dfrobot.com/product-2962.html), [RobotShop](https://robotshop.com/products/feetech-12v-30kgcm-magnetic-encoding-servo-sts3215), Amazon / AliExpress "STS3215 12V". **Buy the 12 V / 30 kg version, not the 7.4 V C001** |
| 6 | **Feetech STS3250**, 12 V (femurs) | 50 kg·cm stall, 16 kg·cm rated (listing figure), same housing, steel gears, about 74.5 g | $97–99 | [Openelab STS3250-C001](https://openelab.io/products/feetech-sts3250-c001-ttl-servo), [AIFitLab](https://aifitlab.com/collections/servo-motor/products/feetech-sts3250-servo-motor) |
| | *Budget option* | 6 more STS3215 instead of STS3250 (femur margin 1.3× rated, DESIGN.md §3) | | |
| 2 / 4 | Feetech STS3032 micro bus servo (attachments only) | 6 V, 4.5 kg·cm stall, 1.5 kg·cm rated, 23.2 × 12.1 × 28.5 mm, 20 g | approx. $12 | [RobotShop](https://www.robotshop.com/products/feetech-6v-45-kg-serial-bus-servo) |
| 3 | STS3215 (arm base, shoulder, elbow) | as above | $22 | |

## 2. Controller, sensors and radio (rev B2)

| Qty | Part | Job | ≈ Price | Where |
| --- | --- | --- | --- | --- |
| 1 | **Adafruit Feather ESP32-S3** (#5323, 8 MB, or #5477 with 2 MB PSRAM) | Main controller: gait, balance, terrain, safety (CONTROL.md) | $17.50 | [Adafruit](https://www.adafruit.com/product/5323), DigiKey 5477 |
| 1 | **Waveshare Servo Driver with ESP32** (kept from rev B) | Servo-bus bridge + battery ADC (`firmware/bus_bridge`) | $15–16 | [Waveshare](https://www.waveshare.com/servo-driver-with-esp32.htm) |
| 1 | **Adafruit BNO085** 9-DOF IMU, STEMMA QT (#4754) | Body attitude: game rotation vector + gyro | approx. $25 | [Adafruit 4754](https://www.adafruit.com/product/4754), [DigiKey](https://www.digikey.com/en/products/detail/adafruit-industries-llc/4754/13426653) |
| 1 | **Pololu VL53L1X** ToF carrier (#3415) | Ground look-ahead, 30° down | approx. $13 | [Pololu 3415](https://www.pololu.com/product/3415) |
| 6 (+2) | **Omron D2F-01** pin-plunger micro-switch | Foot contact, one per tibia tip | approx. $2 each | DigiKey / Mouser / [RS](https://uk.rs-online.com/web/p/micro-switches/6160136) |
| 1 | **RadioMaster RP1 V2** ExpressLRS 2.4 GHz receiver | Radio link, CRSF to the Feather | $18.99 | [RadioMaster](https://radiomasterrc.com/collections/elrs-receivers/products/rp1-expresslrs-2-4ghz-nano-receiver) |
| 1 | **RadioMaster TX16S MK3** (ELRS), EdgeTX | The radio (Lua screens in `radio/`) | $229.99 | [RadioMaster](https://radiomasterrc.com/collections/radio-controller/products/tx16s-mk3-radio-controller), [GetFPV](https://www.getfpv.com/radios/radio-controllers/radiomaster-tx16s-mk3-radio-controller-elrs-dual-band.html) |
| | *Budget radio* | RadioMaster Pocket or Boxer (ELRS), same code | approx. $70–140 | RadioMaster |
| 2 + 6 | Hubs: perfboard (18 × 18 and 14 × 14 mm) + Molex 5264 3-pin right-angle headers | Side hubs (power distribution) and coxa hubs | approx. $10 for the lot | Amazon "5264 3 pin connector kit" |
| 30 | Bus servo leads, 3-pin 5264, 150 / 200 / 300 mm, **22 AWG** if you make your own | | approx. $10 | Waveshare / Amazon |
| 1 | HC-SR04 ultrasonic, 5 V (SparkFun SEN-15569) | Obstacles (front bracket) | $5.25 | [SparkFun](https://www.sparkfun.com/products/15569) |

## 3. Power (rechargeable)

```
3S LiPo ─XT60══XT60─┬─ 20 A ATC ─ ACS724 ─ Pololu MOSFET switch ═╤═ STAR POINT
 4000 mAh           │                         ▲ rocker            ├─ side hub R (1000 µF, 3 × 3 A PTC) ─ legs 0-2 ; J1 (5 A PTC)
                    └─ rear-panel XT60 (charge)                   ├─ side hub L (1000 µF, 3 × 3 A PTC) ─ legs 3-5
   balance lead ─ rear-panel JST-XH 4                             ├─ bus bridge (1.1 A PTC)
                                                                   └─ D36V28F5 5 V ─ Feather, receiver, sonar, ACS724, J1 5 V
   J1 ─ brain module: 470 µF ─ D36V50F5 5 V 5.5 A ─ Pi 5, OAK-D Lite, LD19
```

| Qty | Part | Key spec | ≈ Price | Where |
| --- | --- | --- | --- | --- |
| 1 (2 to swap) | **Gens Ace 3S 11.1 V 4000 mAh 50–60C LiPo, XT60** | 138.5 × 43 × 24.5 mm, about 305 g. **Check that your pack fits the 45 × 140 × 26 mm bay** | $44–54 | [Horizon Hobby (60C G-Tech)](https://www.horizonhobby.com/product/11.1v-4000mah-3s-60c-g-tech-lipo-battery-xt60/GEA403S60X6GT.html), [Amain (50C)](https://www.amainhobbies.com/gens-ace-3s-soft-50c-lipo-battery-pack-w-xt60-connector-11.1v-4000mah-ga-b1120/p795335) |
| 1 | LiPo **balance charger** (2–4S) + LiPo bag | Charge at 2 A (0.5C) | approx. $45–70 | Amazon / RC shops |
| 1 | **Pololu Big MOSFET Slide Switch, HP** (#2815) | About 16 A continuous, reverse-polarity protected | $6.75 | [Pololu #2815](https://www.pololu.com/product/2815) |
| 1 | **Pololu ACS724LLCTR-30AB** (#4047) | ±30 A, 66 mV/A | approx. $11 | [Pololu #4047](https://www.pololu.com/product/4047) |
| 1 | **Pololu D36V28F5** 5 V 3.2 A buck | Logic 5 V | $21.95 | [Pololu #3782](https://www.pololu.com/product/3782) |
| 1 (brain) | **Pololu D36V50F5** 5 V 5.5 A buck | Pi 5 + camera + LiDAR | approx. $25 | [Pololu #4091](https://www.pololu.com/product/4091) |
| 1 per arm/turret | Pololu D36V28F6 6 V buck (on the attachment adapter) | For STS3032 micro servos | approx. $22 | [Pololu #3783](https://www.pololu.com/product/3783) |
| 1 | Inline ATC fuse holder (14 AWG) + 20 A fuse | Main fuse | approx. $6 | Amazon |
| 6 + 1 + 1 | **Littelfuse RXEF300** (3 A hold), **RXEF500** (5 A), **RXEF110** (1.1 A) PTC fuses | Per-leg, J1, bridge | approx. $0.40 each | DigiKey / Mouser |
| 2 + 1 + 1 | Low-ESR electrolytic 1000 µF 25 V, 470 µF 25 V, 100 µF 10 V | Hub bulk, brain buck input, 5 V rail | approx. $3 | DigiKey (Panasonic FR) |
| 1 | SS14 Schottky diode | Feather 5 V feed (no back-feed from USB) | $0.20 | |
| 3 pairs | XT60 (genuine Amass): battery pair, panel-mount female for charging | 30 A | approx. $8 | Amazon / AliExpress (Amass) |
| 1 | XT60 panel-mount holder | | approx. $3 | Amazon |
| 1 | JST-XH 4-pin balance extension, 200 mm | | approx. $3 | Amazon |
| 1 | KCD1 mini rocker | Control signal only | approx. $0.30 | Amazon (10-pack) |
| | Resistors 100 k, 22 k, 10 k, 20 k (1 %), heat-shrink | VBAT and current-sense dividers at the bridge, sonar ECHO divider | | Resistor kit |

**Current budget** (12 V nominal).

| State | Without brain | With brain |
| --- | --- | --- |
| Logic only | 0.25 A | 1.5 A |
| Standing | about 2 A | 3.3 A |
| Walking | 4–6 A | 5.3–7.5 A |
| Peaks | about 25 A | about 27 A |
| Stall (fuse) | 50 A | 50 A |

**Runtime** (3S 4000 mAh, 80 % = 3.2 Ah used):
- **32–48 min** walking without the brain;
- **26–36 min** with it;
- more than 2 h standing still or idle.

## 4. Brain module (autonomy, swappable, AUTONOMY.md)

| Qty | Part | Job | ≈ Price | Where |
| --- | --- | --- | --- | --- |
| 1 | **Raspberry Pi 5, 8 GB** + official Active Cooler | ROS 2 Jazzy, SLAM, Nav2, behaviour tree, ONNX policy | $80 + $5 | [raspberrypi.com](https://www.raspberrypi.com/products/raspberry-pi-5/) |
| 1 | microSD 64 GB A2 (or an NVMe HAT later) | OS | approx. $10 | |
| 1 | **Luxonis OAK-D Lite** | Stereo depth + 4-TOPS on-camera NN, 61 g, 4.5 W max | approx. $149 | [Luxonis](https://docs.luxonis.com/hardware/products/OAK-D%20Lite) |
| 1 | **LDROBOT LD19** (D300 kit) 2-D LiDAR | 12 m, 360°, UART 230400, 47 g, 0.9 W | approx. $70–99 | [Waveshare D300 / LD19](https://www.waveshare.com/dtof-lidar-ld19.htm) |
| 1 | Right-angle USB-A → USB-C cable, 150 mm | Pi → OAK | approx. $8 | |
| — | Toy IR "laser-tag" emitter (940 nm LED + red LED + MOSFET) on the camera mount | Battle behaviour; **not a laser** | < $5 | |

## 5. Sensors and attachments

| Qty | Part | Key spec | ≈ Price | Where |
| --- | --- | --- | --- | --- |
| 1 (optional) | RPLIDAR C1 360° LiDAR (instead of the LD19, as a top attachment) | 5 V, UART | $69–80 | [DFRobot](https://www.dfrobot.com/product-2803.html) |
| — | Toy "laser" turret: 940 nm IR LED + red LED + MOSFET, 2 × STS3032 pan/tilt | IR and visible LEDs only, **not a laser** | < $30 | |
| — | Foam-ball flywheel launcher: 2 × 130 motors + MOSFET driver, STS3032 feeder | Soft foam balls only | approx. $15 | |

## 6. Harness (WIRING.md §7 has the cut list)

| Buy | Item | ≈ Price |
| --- | --- | --- |
| 1 m each | 14 AWG silicone red/black (main path, charge leads) | $6 |
| 1 m each | 18 AWG silicone red/black (side-hub trunks) | $4 |
| 1 m each | 20 AWG silicone red/black (J1, brain) | $4 |
| 6 m | 22 AWG 3-core servo lead (or 3 colours × 6 m) for custom bus leads | $10 |
| 5 m | 28 AWG twisted-pair silicone (foot leads, AUX) | $6 |
| 2 m | 26 AWG (signals, 5 V logic) | $3 |
| 2 | STEMMA QT / Qwiic cables, 200 mm and 100 mm | $2 |
| 1 kit | JST-XH and JST-SH crimp kits, Molex 5264 crimps + crimper | $25 |
| 1 | Heat-shrink assortment, 2.5 mm cable ties (100), braided sleeve 4 mm (1 m) | $10 |

## 7. Mechanical hardware

| Buy | Part | Used for | ≈ Price | Where |
| --- | --- | --- | --- | --- |
| 200 | M3 × 5 × 4 brass heat-set inserts | Plates, pillars, deck, CoxaMain, pods, brain | approx. $10 | Amazon / [Prusa (CNC Kitchen)](https://www.prusa3d.com/product/heat-set-inserts-m3-standard-100-pcs/) |
| 200 | M3 × 8 button head (ISO 7380) | Horn to bracket (144), deck, tray, carrier, pods | approx. $8 | Amazon assortment, [McMaster 94500A222](https://www.mcmaster.com/94500a222) |
| 20 | M3 × 10 button head | CoxaCap (3 per leg), panel | in assortment | |
| 80 | M2 × 8 self-tapping (PT) screws | Servo cases (72), VL53L1X | approx. $6 | Amazon |
| 12 + 6 nuts | M2 × 10 + M2 nuts | Foot switches (2 per tibia) | approx. $3 | |
| 2 | M2 × 12 | Camera to the brain mount | in assortment | |
| 30 | M2.5 × 6 self-tapping | Feather, BNO085, Pi 5, buck boards, hubs | approx. $5 | Amazon |
| 4 | M2.5 TPU / rubber grommets (or 3 mm TPU washers, printed) | IMU vibration isolation | $2 | |
| 50 | 2.5 mm cable ties | Strain relief at the plate slots and joint splices | approx. $3 | Amazon |
| 1 | 20 mm hook-and-loop strap | Battery | approx. $3 | Amazon |
| 2.2 kg | PETG (legs, plates, brackets, pods, brain) + 100 g TPU 95A (boots) | | approx. $50 | Any |

## 8. Rough total

| Group | Chosen (STS3250 femurs) | Budget (all STS3215) |
| --- | --- | --- |
| Leg servos | 13 × 22 + 6 × 99 ≈ $880 | 19 × 22 ≈ $420 |
| Controller, bridge, sensors, receiver | ≈ $115 | ≈ $115 |
| Power (switch, sensor, bucks, fuses, PTCs, caps, XT60s) | ≈ $70 | ≈ $70 |
| Battery + charger + bag | ≈ $110 | ≈ $110 |
| Harness (wire, crimps, sleeves) | ≈ $70 | ≈ $70 |
| Hardware + filament | ≈ $95 | ≈ $95 |
| **Robot total** | **≈ $1,340** | **≈ $880** |
| Radio (TX16S MK3) | + $230 | + $70 (Pocket) |
| Brain module (Pi 5, cooler, SD, OAK-D Lite, LD19, buck, cables) | + ≈ $350 | + ≈ $350 |
