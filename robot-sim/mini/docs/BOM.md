# Hexapod Mini: bill of materials

**Total: about US$196, under the US$200 budget.** Every bought part is counted, including filament, the battery, the charger, the gamepad and shipping. Sales tax is not included, because it depends on the state.

How the prices were found:

- Checked on the web on **2026-11-25 and 2026-11-26** (US dollars).
- **"approx."** means there was no clear live price for that exact item, so the figure is a typical price from several listings.
- Where an item only comes in packs, the pack price is counted.
- Prices move. Check each link before ordering.

| # | Item | Qty | Where (link) | Unit | Line | Price |
| --- | --- | --- | --- | --- | --- | --- |
| 1 | **MG90S** metal-gear micro servo (incl. horns and screws) | 20 (18 + 2 spares) | [Waveshare MG90S](https://www.waveshare.com/mg90s-servo.htm), 10–49 pcs price | $2.15 | $43.00 | checked |
| 2 | **ESP32-DevKitC-32E** (classic ESP32, Bluetooth Classic + BLE) | 1 | [Gridconnect](https://gridconnect.com/products/esp32-devkitc-32e-wroom-module-development-kit) ($9.95), [Mouser](https://www.mouser.com/c/?q=ESP32-DevKitC-32E) ($10.00) | $10.00 | $10.00 | checked |
| 3 | **PCA9685** 16-channel servo board, 2-pack (generic) | 1 pack | e.g. [Walmart 2-pack](https://www.walmart.com/ip/17526473022) ($8.89, listing unclear); the genuine [Adafruit 815](https://www.adafruit.com/product/815) is $14.95 each | $12.00 | $12.00 | approx. |
| 4 | **Mini560** 5 V / 5 A buck module | 2 | e.g. [eBay](https://www.ebay.com/itm/256367033646) ($3.83), [Gogo:Tronics](https://sparks.gogo.co.nz/catalog/DC-Power-260/Step-Down-289/Switch-Mode-Buck-Converters-123/Fixed-Output-Step-Down-Converter-Out-5v-up-to-5A-Continuous-In-7-20v-1009.html) ($2.95) | $4.00 | $8.00 | approx. |
| 5 | **2S 20 A BMS**, balanced version | 1 | e.g. [eBay](https://www.ebay.de/itm/166283753293) (EUR 3.51) | $4.00 | $4.00 | approx. |
| 6 | **Molicel P28A** 18650, 2800 mAh, 35 A | 2 | [18650 Battery Store](https://www.18650batterystore.com/products/molicel-p28a) | $5.99 | $11.98 | checked |
| 7 | 2 × 18650 holder, series (7.4 V), with leads | 1 | [Vetco VET-29179](https://vetco.net/products/vet-29179_2_x_18650_battery_holder_w-_leads) | $5.49 | $5.49 | checked |
| 8 | **XTAR MC2** USB 2-bay Li-ion charger | 1 | [Battery Junction](https://batteryjunction.com/xtar-mc2) | $7.14 | $7.14 | checked |
| 9 | KCD1 mini rocker switch (3-pack) | 1 pack | e.g. [eBay 3-pack](https://www.ebay.com/str/thingsmany) ($4.99), [Walmart](https://www.walmart.com/c/kp/kcd1-switch) | $4.99 | $4.99 | approx. |
| 10 | Inline mini-blade fuse holder (18 AWG) + 10 A mini blade fuses | 1 + pack | [OptiFuse LPM-01B-18B at Jameco](https://www.jameco.com/z/LPM-01B-18B-OptiFuse-Circuit-Protection-Black-Fuse-Holder-IP65-In-Line-for-Mini-Blade-18AWG-10A-Black-Wire_2721551.html) (~$2.60); fuses from any car-parts shop | — | $6.60 | approx. |
| 11 | 1N5822 Schottky diode; 100 kΩ and 47 kΩ resistors | 1 each | any electronics shop | — | $2.00 | approx. |
| 12 | **8BitDo Lite 2** Bluetooth gamepad (two sticks) | 1 | [8BitDo store](https://shop.8bitdo.com/products/8bitdo-lite-2-bluetooth-gamepad-for-switch-android-and-raspberry-pi) (sale price) | $20.99 | $20.99 | checked |
| 13 | **PLA** 1.75 mm, 1 kg (the robot uses about 270 g) | 1 | [Elegoo US](https://us.elegoo.com/products/elegoo-pla-filament-1-75mm-3d-printer-filament-1kg-spool-2-2-lbs) | $13.99 | $13.99 | checked |
| 14 | M2 + M3 screw and nut assortment (see the screw list below) | 1 kit | any | — | $9.00 | approx. |
| 15 | 20 AWG silicone wire, red and black | 2 × 2 m | any | — | $7.00 | approx. |
| 16 | Dupont female–female jumper wires (20 cm) | 1 set | any | — | $5.00 | approx. |
| 17 | Heat-shrink tube assortment | 1 | any | — | $5.00 | approx. |
| 18 | Double-sided foam tape, zip ties | 1 each | any | — | $5.00 | approx. |
| 19 | Shipping allowance (servos, cells, small orders) | — | — | — | $15.00 | approx. |
| | **Total** | | | | **$196.18** | |

Ways to bring the total down:

- **A gamepad you already own:** −$21. DualShock 4, DualSense, Switch Pro and Xbox Wireless pads all work with Bluepad32.
- **Servos from a bulk marketplace:** about −$10, but the quality varies.
- **No spare servos:** −$4.30.

Spending the margin instead:

- **Genuine Adafruit PCA9685 boards:** +$18.
- **A cheap multimeter:** strongly recommended if the workshop doesn't have one.

**Tools assumed in the workshop and not counted:**

- FDM printer with a bed of 180 × 180 mm or larger;
- soldering iron;
- small Phillips screwdrivers;
- wire stripper;
- multimeter;
- USB cable for the ESP32.

## Screw list (from the CAD)

| Where | Screw | Qty |
| --- | --- | --- |
| Coxa, femur and tibia servo tabs into printed seats | MG90S tab screws (in the servo bag), self-tapping | 24 |
| Coxa servo tabs, through the deck | M2 × 8 + M2 nut | 12 |
| Horns to the printed arms | M2 × 5 self-tapping, from the kit (2 per horn, in the outer horn holes) | 36 |
| Horn centre screws | MG90S (in the bag) | 18 |
| Idler pins: femur, tibia and coxa | M3 × 6 pan head, self-tapping into the printed arm | 18 |
| Deck to tray | M3 × 10 | 4 |
| Battery hatch to tray | M3 × 6 | 2 |
| Battery holder to hatch | M3 × 6 countersunk + nut | 2 |
| PCA9685 boards to the deck standoffs | M2.5 × 6 (self-tapping) | 8 |

## Why these parts (the trade-off)

The budget sets the design.

**Servos.** 18 servos are needed for a hexapod that can walk, turn and strafe.

- 18 × MG90S cost about $43.
- Cheap serial-bus servos (Feetech SCS0009 class) cost $6–9 each, so $110–160, which breaks the budget on their own.
- The MG90S also has metal gears, unlike the SG90, which strips under a walking robot's shock loads.

**Radio.** A Bluetooth gamepad on the ESP32 (Bluepad32) costs $0–21.

- An ExpressLRS handset and receiver costs $70–100.
- The gamepad's range (about 10 m indoors) is plenty for a desk-top robot.
- It uses the ESP32 the robot needs anyway.
- This is why the board is the **classic ESP32** (DevKitC): the S3 can only use BLE pads.

**Battery.** Two loose **18650 Li-ion cells** in a holder instead of a LiPo pouch.

- Steel-can cells are much harder to puncture than a pouch.
- They charge **outside the robot** in a $7 two-bay USB charger that balances each cell on its own, so no balance charger or LiPo bag is needed.
- The **BMS** cuts the pack off below about 2.5–2.8 V per cell, even if the robot is left switched on.
- The **fuse** limits a short to 10 A.

**Power.** Two **Mini560 5 V / 5 A** bucks, one per side, instead of one larger module.

- They are tiny, so they fit under the deck.
- The two halves of the robot don't share a single 5 A limit.
- The servos run at 5.0 V, inside the MG90S's 4.8–6 V range.
