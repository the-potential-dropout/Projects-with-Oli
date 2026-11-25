# Hexapod Mk1: electronics and bought parts

Everything that isn't 3D-printed is a **standard catalogue part**: hobby servos, common breakout boards, ISO/DIN M3 hardware, 623ZZ bearings, XT60 / JST-XH / ATC-fuse parts. All of it ships by normal post. Where it's cheaper, buy a multi-pack from Amazon or AliExpress. The "brand / official" column gives a named product page (Adafruit, Pololu, SparkFun, Espressif via DigiKey, McMaster-Carr) to use if you want a known-good part or a datasheet.

**Prices** are US$ and were checked in November 2026 where a link is given. Anything marked **approx.** is a typical street price that I couldn't confirm on a product page: check before ordering. The design is in [DESIGN.md](DESIGN.md).

## 1. Power

```
2S LiPo ──XT60── 20 A ATC fuse ── Pololu Big MOSFET switch HP ──┬── UBEC A (6.0 V) ── PCA9685 #1 V+  (legs 0-2, aux 12-15)
 (7.4 V)                          (rocker on rear panel)        ├── UBEC B (6.0 V) ── PCA9685 #2 V+  (legs 3-5, aux 28-31)
                                                                │                 └─ 5 A fuse ── J1 V_SERVO (attachments)
                                                                ├── 5 V buck (D36V28F5) ── ESP32 5V, HC-SR04/LiDAR, J1 +5V
                                                                └── 100k/33k divider ── ESP32 GPIO34 (battery voltage)
ESP32 3V3 ── PCA9685 VCC (logic) and I²C pull-ups;  SDA = GPIO21, SCL = GPIO22 (firmware Wire.begin() defaults)
All grounds common (star point on the perfboard).
```

| Qty | Part | Key spec | ≈ Price | Brand / official | Cheap source |
| --- | --- | --- | --- | --- | --- |
| 1 (2 recommended) | 2S 7.4 V 2200 mAh LiPo, hard case, XT60 | ≥ 30C. Must fit the 37 × 27 × 112 mm tray: check the pack size | $28 | [Gens Ace 2S 60C hardcase (Horizon Hobby)](https://www.horizonhobby.com/product/7.4v-2200mah-2s-60c-hardcase-g-tech-lipo-battery-xt60/GEA222S60X76GT.html) | Amazon / any RC shop |
| 1 | LiPo balance charger (2S–4S, e.g. ISDT, ToolkitRC, SkyRC) | Balance charging | $30–60 approx. | | Amazon |
| 1 | LiPo safe bag | | $10 approx. | | Amazon |
| 2 | UBEC, set to 6.0 V | Hobbywing UBEC 10A V2 (30603003): 10 A continuous, 2–6S in, 5/6/7.4/8.4 V selectable | $24–35 each | [Hobbywing UBEC 10A](https://www.hobbywing.com/en/products/ubec-10a-2-6s152.html) | [Active Powersports listing](https://www.activepowersports.com/hobbywing-ubec-10a-v2-car-2-6s-30603003), Amazon |
| 1 | 5 V buck for logic | Pololu D36V28F5: 5 V, 3.2 A, up to 50 V in | $21.95 | [Pololu #3782](https://www.pololu.com/product/3782) | (MP1584 modules on Amazon work, but are less robust) |
| 1 | Main power switch | Pololu Big MOSFET Slide Switch HP: about 16 A, 4.5–40 V, reverse-voltage protection. The panel rocker drives its control input | $6.75 | [Pololu #2815](https://www.pololu.com/product/2815) | |
| 1 | Mini rocker switch (KCD1, 19 × 13 mm cut-out) | Only switches the MOSFET switch's control line | $0.30 approx. (10-pack) | | Amazon "KCD1 rocker switch" |
| 1 | Inline ATC/ATO blade fuse holder, 14 AWG + 20 A and 5 A blade fuses | Main fuse 20 A, attachment rail 5 A | $8 approx. | | Amazon |
| 2 pairs | XT60 male/female (genuine Amass) | Battery connector | $5 approx. | | Amazon / AliExpress (Amass) |
| 1 m each | Silicone wire 14 AWG (red/black), 22 AWG for logic | | $10 approx. | | Amazon |
| 2 | 1000–2200 µF 10 V low-ESR electrolytic | Servo rail bulk capacitor, one per PCA9685 | $1 approx. | | Amazon / DigiKey |
| 2 | 100 kΩ and 33 kΩ resistors | Battery voltage divider | — | | Resistor kit |

## 2. Control

| Qty | Part | Key spec | ≈ Price | Brand / official | Cheap source |
| --- | --- | --- | --- | --- | --- |
| 1 | ESP32-DevKitC-32E | ESP32-WROOM-32E, USB-serial, 28 mm wide, 25.4 mm header spacing | $8–10 | [Espressif ESP32-DEVKITC-32E (DigiKey)](https://www.digikey.com/en/products/detail/espressif-systems/ESP32-DEVKITC-32E/12091810) | Amazon "ESP32 DevKitC" (clones are fine) |
| 2 | PCA9685 16-channel PWM driver | I²C, 0x40 and 0x41 (solder A0 on the second board), servo V+ terminal, 62.5 × 25.4 mm | $14.95 each | [Adafruit #815](https://www.adafruit.com/product/815) | Amazon PCA9685 2-pack, about $10 approx. Check the hole spacing matches the bosses |
| 1 | 50 × 70 mm FR4 perfboard, 3 mm corner holes | Controller carrier board | $8 approx. (multi-size pack) | | Amazon |
| 2 | 1 × 19 female headers (2.54 mm) | ESP32 sockets | in a header kit | | Amazon |
| 1 kit | JST-XH 2.54 mm connector kit (2–8 pin) + crimper | J1 (8-pin) and J2 (6-pin) attachment port, servo-lead splices | $15 approx. | | Amazon |
| 6 | 30 cm servo extension leads | Only where stock leads are short (front/rear tibia) | $8 approx. (10-pack) | | Amazon |

## 3. Servos

| Qty | Part | Key spec | ≈ Price | Notes |
| --- | --- | --- | --- | --- |
| 12 (or 18) | MG996R metal-gear servo | 40.7 × 19.7 × 42.9 mm, 55 g, 9.4 kg·cm at 4.8 V and 11 kg·cm at 6 V (stall), 2.5 A stall at 6 V, 25T spline ([TowerPro datasheet copy](https://people.ece.cornell.edu/land/courses/ece4760/FinalProjects/f2017/kmv48_rp625/kmv48_rp625/kmv48_rp625/MG996RTowerPro.pdf)) | $4–6 each approx. in 10–20 packs | Coxa and tibia joints. Buy 2 spares |
| 6 | DS3218MG 20 kg servo, **180° version** | 40 × 20 × 40.5 mm, about 60–68 g, 25T spline | $15–19 each approx. ([RobotShop Yahboom 20 kg](https://ca.robotshop.com/products/yahboom-yahboom-20kg-digital-servo-motor-w-full-metal-gear-waterproof-180deg-control-angle-high-torque-rc-model)) | **Recommended for the femur joints** (torque check, DESIGN.md §8). Check the tab height against the MG996R pocket before printing 6 coxas |
| 6 | MG90S micro servo | 22.8 × 12.2 × 28.5 mm, 13.4 g, 1.8–2.2 kg·cm | $2–3 each approx. (packs) | 5 for the arm, 1 for the turret pitch. Buy from a reputable seller: MG90S clones vary |
| 20 | 25T aluminium round servo horn, about 25 mm, tapped M3 | Check the bolt circle and set `horn_pcd` | $1 each approx. (10-packs) | Amazon "25T aluminum servo horn" |

## 4. Sensors and attachment electronics

| Qty | Part | Key spec | ≈ Price | Brand / official |
| --- | --- | --- | --- | --- |
| 1 | HC-SR04 ultrasonic, **3.3 V version** | 45 × 20 mm board, transducers 26 mm apart. The 3.3 V version talks straight to the ESP32 | $6.95 | [SparkFun SEN-24049 (3.3 V)](https://www.sparkfun.com/ultrasonic-distance-sensor-3-3v-hc-sr04.html). The 5 V [SEN-15569](https://www.sparkfun.com/products/15569) ($5.25) needs a divider on ECHO |
| 1 (optional) | RPLIDAR C1 360° DTOF LiDAR | 12 m range, UART, 5 V | $69–80 | [DFRobot](https://www.dfrobot.com/product-2803.html), [Waveshare](https://www.waveshare.com/rplidar-c1.htm) |
| — | Toy "laser" turret: 940 nm IR LED (laser-tag style) + red LED + 2N7000 MOSFET | **Not a laser module.** IR LED + visible LED only | < $5 approx. | Amazon / DigiKey |
| — | Foam-ball launcher: 2 × 130-size DC motors + flywheels, IRLZ44N MOSFET + flyback diode, MG90S feeder | Soft foam darts/balls only | $10 approx. | Amazon |

## 5. Mechanical hardware

Standard metric parts. Quantities are from DESIGN.md §4 (the "buy" column includes spares).

| Buy | Part | Standard | ≈ Price | McMaster-Carr | Cheap source |
| --- | --- | --- | --- | --- | --- |
| 200 | M3 brass heat-set insert, M3 × 5 OD × 4 L | Voron-style "M3 × 5 × 4" | $10 approx. | Tapered brass M3 insert [94180A331](https://www.mcmaster.com/94180A331/) (3.8 mm long, larger tapered hole: change `insert_hole`) | Amazon / AliExpress "M3x5x4 heat set insert", [Prusa / CNC Kitchen M3 inserts](https://www.prusa3d.com/product/heat-set-inserts-m3-standard-100-pcs/) |
| 200 | M3 × 8 button head hex screw | ISO 7380, A2 stainless or black 10.9 | $8 approx. | [94500A222](https://www.mcmaster.com/94500a222) (316 SS, M3 × 8) | Amazon M3 button-head assortment |
| 25 | M3 × 12 button head | ISO 7380 | in assortment | [ISO 7380 button head range](https://www.mcmaster.com/products/screws/specifications-met~iso-7380-1/) | Amazon assortment |
| 10 | M3 × 10 button head | ISO 7380 | in assortment | same range | Amazon assortment |
| 50 | M3 washer, 7 mm OD | DIN 125-A / ISO 7089 | $3 approx. | McMaster M3 18-8 washers | Amazon assortment |
| 20 | 623ZZ ball bearing 3 × 10 × 4 mm | 623-2Z, shielded | $8 approx. (20-pack) | [623-2Z bearings](https://www.mcmaster.com/products/bearings/bearing-trade-number~623-2z): check the 4 mm width column | Amazon "623ZZ" (uxcell etc.) |
| 10 | M3 × 45 mm hex standoff, female-female, aluminium | 5.5 mm across flats | $8 approx. | McMaster aluminium hex standoffs | Amazon |
| 5 | M3 × 10 knurled thumb screw | Quick-release lock | $5 approx. | | Amazon |
| 10 + 10 | M2.5 × 6 and M2 × 6 self-tapping (PT) screws | PCA9685 and HC-SR04 | $5 approx. | | Amazon |
| 2 | 20 mm hook-and-loop strap, 200 mm | Battery strap | $5 approx. | | Amazon |
| 100 | Cable ties 2.5 mm | Strain relief | $3 approx. | | Amazon |
| 2 kg | Filament: 1 kg PLA + 1 kg PETG, plus 100 g TPU 95A | | $40–55 approx. | | Any |

## 6. Tools

Soldering iron with a heat-set insert tip (M3), crimper for JST-XH and Dupont, 2 mm and 2.5 mm hex keys, calipers (essential for checking servo and bearing sizes), multimeter, LiPo charger.

## 7. Rough total

| Group | ≈ US$ |
| --- | --- |
| Servos (12 MG996R + 6 DS3218 + 6 MG90S + horns) | 180 |
| Control (ESP32, 2 × PCA9685, perfboard, connectors) | 60 |
| Power (2 UBECs, buck, switch, fuse, XT60, wire, caps) | 115 |
| Battery + charger + bag | 70–100 |
| Hardware | 55 |
| Filament | 45 |
| **Total** | **≈ 525–555** (about 440 with all-MG996R servos and AliExpress boards) |

Prices change often. Treat this as a planning budget, not a quote.
