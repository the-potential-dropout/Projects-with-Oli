# Hexapod Mk1 (rev B): electronics and bought parts

Everything that isn't printed is a standard catalogue part that ships by normal post.

**Prices** are US$ and were checked on 2026-11-25 where a link is given. Prices marked **approx.** are typical street prices I couldn't confirm on a product page. Prices change; treat this as a budget, not a quote. The design and load check are in [DESIGN.md](DESIGN.md), and the wiring and charging procedure in [BUILD.md](BUILD.md).

## 1. Servos

| Qty | Part | Key spec | ≈ Price | Where |
| --- | --- | --- | --- | --- |
| 12 (+1 spare) | **Feetech STS3215 / Waveshare ST3215, 12 V version** (C018/C047) | 30 kg·cm stall, 10 kg·cm rated, 2.7 A stall at 12 V, 45.22 × 24.72 × 35 mm, about 69 g, 12-bit magnetic encoder, TTL bus, metal horns included | $20–22 | [Waveshare ST3215 ($21.99)](https://www.waveshare.com/product/robotics/motors-servos/servos/st3215-servo.htm), [DFRobot SER0070 ($19.90)](https://www.dfrobot.com/product-2962.html), [RobotShop](https://robotshop.com/products/feetech-12v-30kgcm-magnetic-encoding-servo-sts3215), Amazon / AliExpress "STS3215 12V". **Buy the 12 V / 30 kg version, not the 7.4 V C001** |
| 6 | **Feetech STS3250**, 12 V (femurs) | 50 kg·cm stall, 16 kg·cm rated (listing figure), same housing, steel gears, about 74.5 g | $97–99 | [Openelab STS3250-C001](https://openelab.io/products/feetech-sts3250-c001-ttl-servo), [AIFitLab](https://aifitlab.com/collections/servo-motor/products/feetech-sts3250-servo-motor) |
| | *Budget option* | 6 more STS3215 instead of STS3250 (femur margin 1.3× rated, see DESIGN.md §3) | | |
| 2 / 4 | Feetech STS3032 micro bus servo (attachments only) | 6 V, 4.5 kg·cm stall, 1.5 kg·cm rated, 23.2 × 12.1 × 28.5 mm, 20 g | approx. $12 | [RobotShop](https://www.robotshop.com/products/feetech-6v-45-kg-serial-bus-servo) |
| 3 | STS3215 (arm base, shoulder, elbow) | As above | $22 | |

## 2. Control

| Qty | Part | Key spec | ≈ Price | Where |
| --- | --- | --- | --- | --- |
| 1 | **Waveshare Servo Driver with ESP32** | ESP32-WROOM, USB-C, bus-servo port (GPIO18/19), 6–12 V DC jack, OLED, 65 × 30 mm, holes 58 × 23 | $15–16 | [Waveshare](https://www.waveshare.com/servo-driver-with-esp32.htm), [The Pi Hut](https://thepihut.com/products/esp32-servo-driver-with-wifi-and-bluetooth), [RobotShop](https://www.robotshop.com/products/esp32-servo-driver-expansion-board-built-in-wifi-bluetooth) |
| 2 + 6 | Bus hubs: perfboard (18 × 18 and 14 × 14 mm) + Molex 5264 3-pin right-angle headers (6 per side hub, 3 per coxa hub) | Hand-built: all pins 1, all pins 2, all pins 3 joined (GND, V, DATA) | approx. $10 for the lot | Amazon "5264 3 pin connector kit" |
| 30 | Bus servo leads, 3-pin 5264, 150 / 200 / 300 mm | Spares; each servo ships with one lead | approx. $10 | Waveshare / Amazon |
| 1 | JST-XH 2.54 kit + crimper | J1 attachment port (6-pin) and balance socket (4-pin) | approx. $15 | Amazon |

## 3. Power (rechargeable)

```
3S LiPo ─XT60══XT60─┬─ 20 A ATC fuse ─ ACS724 (±30 A) ─ Pololu MOSFET switch ─┬─ driver board DC jack (2 A)
 4000 mAh           │                                        ▲ rocker (rear)   ├─ 5 V buck (D36V28F5): sonar, ACS724, sensors
                    │                                                          ├─ side hub R ─ coxa servos 0-2 (+ J1, 5 A polyfuse)
                    └─ charge leads ─ rear-panel XT60                          └─ side hub L ─ coxa servos 3-5
   balance lead ─ extension ─ rear-panel JST-XH 4                VBAT sense: 100k/22k → GPIO34   ACS724 out: 10k/20k → GPIO35
```

| Qty | Part | Key spec | ≈ Price | Where |
| --- | --- | --- | --- | --- |
| 1 (2 to swap) | **Gens Ace 3S 11.1 V 4000 mAh 50–60C LiPo, XT60** | 138.5 × 43 × 24.5 mm, about 305 g, hard or soft case. **Check that your pack fits the 45 × 140 × 26 mm bay** | $44–54 | [Horizon Hobby (60C G-Tech)](https://www.horizonhobby.com/product/11.1v-4000mah-3s-60c-g-tech-lipo-battery-xt60/GEA403S60X6GT.html), [Amain (50C)](https://www.amainhobbies.com/gens-ace-3s-soft-50c-lipo-battery-pack-w-xt60-connector-11.1v-4000mah-ga-b1120/p795335) |
| 1 | LiPo **balance charger** (2–4S, CC/CV, e.g. ISDT 608AC, ToolkitRC M6AC, SkyRC) + LiPo safe bag | Charge at 2 A (0.5C) | approx. $45–70 | Amazon / RC shops |
| 1 | **Pololu Big MOSFET Slide Switch with Reverse Voltage Protection, HP** (#2815) | About 16 A continuous, 4.5–40 V, reverse-polarity protection. The rear rocker drives its control input | $6.75 | [Pololu #2815](https://www.pololu.com/product/2815) |
| 1 | **Pololu ACS724LLCTR-30AB current sensor** (#4047) | ±30 A, 66 mV/A, 5 V supply. Battery gauge and current log | approx. $11 | [Pololu #4047](https://www.pololu.com/product/4047) |
| 1 | **Pololu D36V28F5** 5 V 3.2 A buck | 5 V for the sonar, current sensor, LiDAR | $21.95 | [Pololu #3782](https://www.pololu.com/product/3782) |
| 1 per arm/turret | Pololu D36V28F6 6 V buck (on the attachment adapter) | For STS3032 micro servos | approx. $22 | [Pololu #3783](https://www.pololu.com/product/3783) |
| 1 | Inline ATC fuse holder, 14 AWG + 20 A fuse | Right after the battery XT60 | approx. $6 | Amazon |
| 1 | 5 A PTC polyfuse (radial) | In the J1 VBAT line | approx. $1 | DigiKey / Mouser |
| 3 pairs | XT60 (genuine Amass): battery pair, panel-mount female for charging | | approx. $8 | Amazon / AliExpress (Amass) |
| 1 | XT60 panel-mount holder (or print the panel cut-out) | | approx. $3 | Amazon |
| 1 | JST-XH 4-pin balance extension, 200 mm | To the rear-panel balance socket | approx. $3 | Amazon |
| 1 | KCD1 mini rocker | Control signal only | approx. $0.30 | Amazon (10-pack) |
| 1 m each | 14 AWG silicone wire (red/black), 22 AWG, 26 AWG | Power, feeds, signal | approx. $12 | Amazon |
| | Resistors 100 k, 22 k, 10 k, 20 k (1 %), heat-shrink | VBAT and current-sense dividers, soldered in line at the driver | | Resistor kit |

## 4. Sensors and attachments

| Qty | Part | Key spec | ≈ Price | Where |
| --- | --- | --- | --- | --- |
| 1 | HC-SR04 ultrasonic, 5 V (SparkFun SEN-15569) | Powered from the 5 V buck. Divide ECHO 5 V → 3.3 V (10k/20k) | $5.25 | [SparkFun](https://www.sparkfun.com/products/15569) |
| 1 (optional) | RPLIDAR C1 360° LiDAR | 5 V, UART | $69–80 | [DFRobot](https://www.dfrobot.com/product-2803.html) |
| — | Toy "laser" turret: 940 nm IR LED + red LED + MOSFET, 2 × STS3032 pan/tilt | IR and visible LEDs only, **not a laser** | < $30 | |
| — | Foam-ball flywheel launcher: 2 × 130 motors + MOSFET driver, STS3032 feeder | Soft foam balls only | approx. $15 | |

## 5. Mechanical hardware

| Buy | Part | Used for | ≈ Price | Where |
| --- | --- | --- | --- | --- |
| 200 | M3 × 5 × 4 brass heat-set inserts | Plates, pillars, deck, CoxaMain, adapters | approx. $10 | Amazon / [Prusa (CNC Kitchen)](https://www.prusa3d.com/product/heat-set-inserts-m3-standard-100-pcs/) |
| 200 | M3 × 8 button head (ISO 7380) | Horn to bracket (4 per horn face, 72 faces → 144), deck, tray, carrier | approx. $8 | Amazon assortment, [McMaster 94500A222](https://www.mcmaster.com/94500a222) |
| 20 | M3 × 10 button head | CoxaCap (3 per leg), panel | in assortment | |
| 80 | M2 × 8 self-tapping (PT) screws | Servo cases (4 per servo × 18 = 72) | approx. $6 | Amazon |
| 30 | M2.5 × 6 self-tapping | Driver board, ACS724, hubs, buck, switch | approx. $5 | Amazon |
| 50 | 2.5 mm cable ties | Strain relief at the bottom-plate slots and joint splice points | approx. $3 | Amazon |
| 1 | 20 mm hook-and-loop strap | Battery | approx. $3 | Amazon |
| 2 kg | PETG (legs, plates, brackets) + 100 g TPU 95A (boots) | | approx. $45 | Any |

The servo horns are included with the STS servos.

## 6. Rough total

| Group | Chosen (STS3250 femurs) | Budget (all STS3215) |
| --- | --- | --- |
| Leg servos | 13 × 22 + 6 × 99 ≈ $880 | 19 × 22 ≈ $420 |
| Driver, hubs, leads, connectors | ≈ $50 | ≈ $50 |
| Power (switch, sensor, buck, fuse, XT60s, wire) | ≈ $70 | ≈ $70 |
| Battery + charger + bag | ≈ $110 | ≈ $110 |
| Hardware + filament | ≈ $80 | ≈ $80 |
| **Total** | **≈ $1,190** | **≈ $730** |
