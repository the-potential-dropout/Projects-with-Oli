# Vendor CAD: sources and licences

The generator ([../hexapod_mk1_fusion.py](../hexapod_mk1_fusion.py)) imports the manufacturers' STEP models when they are in this folder. If a file is missing, it falls back to an envelope built from the same datasheet numbers.

Run this once to download everything listed under "Downloaded by the script":

```
python fetch_vendor_cad.py
```

**Nothing in this folder is committed except this file and the script** (see `.gitignore`).

- Most of the downloaded files carry no explicit licence that allows redistribution, so the repo links to them instead of copying them.
- The Adafruit (Adafruit_CAD_Parts) and Luxonis (depthai-hardware) models are MIT-licensed and could be committed. They are kept out too, so the rule stays simple.
- The repo's `Hexapod_Mk1.f3d`, STLs and sim meshes are exported from the envelope build (`build(use_vendor=False)`), so they contain no vendor geometry.
- The copy in the user's Autodesk cloud ("Hexapod Mk1", Default Project) is built with the real models.

All files were fetched on 2026-11-25 from the manufacturer's own download server, with no login and no executables.

## Downloaded by the script

| Part | File | Official source | Licence | Used for |
| --- | --- | --- | --- | --- |
| Feetech STS3215 / Waveshare ST3215 serial-bus servo | `ST3215.step` (Waveshare "ST3215-3D.zip") | [waveshare.com/wiki/ST3215_Servo](https://www.waveshare.com/wiki/ST3215_Servo) → `files.waveshare.com/upload/5/59/ST3215-3D.zip` | None stated (free download). Not redistributed | 12 × coxa/tibia servos. Also the 6 × STS3250 femur servos, which use the same 45.22 × 24.72 × 35 mm housing (STS3250 listings give 45.22 × 24.72 × 35). Mounting faces, horn and socket positions were measured from this model |
| ST3215 2D drawing "SCS215" | `ST3215.pdf` (from "ST3215-2D.zip") | same wiki → `files.waveshare.com/upload/0/08/ST3215-2D.zip` | None stated. Not redistributed | Cross-check of the dimensions |
| Waveshare Servo Driver with ESP32 | `Servo Driver with ESP32.step` | [waveshare.com/wiki/Servo_Driver_with_ESP32](https://www.waveshare.com/wiki/Servo_Driver_with_ESP32) → `files.waveshare.com/upload/7/73/Servo_Driver_with_ESP32_3D.zip` | None stated. Not redistributed | Controller board |
| Pololu D36V28Fx step-down regulator | `d36v28fx-step-down-voltage-regulators.step` | [pololu.com/product/3782/resources](https://www.pololu.com/product/3782/resources) → `pololu.com/file/0J1660/…` | None stated. Not redistributed | 5 V logic buck (D36V28F5) |
| Pololu Big MOSFET Slide Switch with Reverse Voltage Protection | `big-mosfet-slide-switch-with-reverse-voltage-protection.step` | [pololu.com/product/2815/resources](https://www.pololu.com/product/2815/resources) → `pololu.com/file/0J1406/…` | None stated. Not redistributed | Main power switch (HP version) |
| Pololu ACS724 current sensor carrier | `acs724-current-sensor-carrier.step` | [pololu.com/product/4047/resources](https://www.pololu.com/product/4047/resources) → `pololu.com/file/0J1985/…` | None stated. Not redistributed | Battery current sensor (ACS724LLCTR-30AB) |
| Pololu D36V50Fx step-down regulator | `d36v50fx-step-down-voltage-regulator.step` | [pololu.com/product/4091/resources](https://www.pololu.com/product/4091/resources) → `pololu.com/file/0J1733/…` | None stated. Not redistributed | Brain module 5 V / 5.5 A buck (D36V50F5) |
| Pololu VL53L1X carrier | `vl53l1x-carrier.step` | [pololu.com/product/3415/resources](https://www.pololu.com/product/3415/resources) → `pololu.com/file/0J1195/…` | None stated. Not redistributed | Time-of-flight look-ahead on the front bracket |
| Adafruit Feather ESP32-S3 (#5323) | `5323 Feather ESP32-S3.step` | [github.com/adafruit/Adafruit_CAD_Parts](https://github.com/adafruit/Adafruit_CAD_Parts) (raw GitHub) | **MIT** | Main controller in the control pod |
| Adafruit BNO085 STEMMA QT (#4754) | `4754 BNO085 STEMMA QT.step` | same repo | **MIT** | IMU in the IMU pod |
| Luxonis OAK-D Lite enclosure (DM9095) | `DM9095_enclosure.stp` | Linked from [github.com/luxonis/depthai-hardware](https://github.com/luxonis/depthai-hardware/tree/master/DM9095_OAK-D-LITE_DepthAI_USB3C) → Luxonis CDN `oak-files.fra1.cdn.digitaloceanspaces.com` | **MIT** (repo) | Brain module camera |
| Raspberry Pi 5 | `rpi5.step` (from `RP-010083-CA-1-rpi-5 3D STEP - No Graphics small file.zip`) | [pip.raspberrypi.com, Raspberry Pi 5 category](https://pip.raspberrypi.com/categories/892-raspberry-pi-5) (public, no login) | None stated. Not redistributed | Brain module computer (the Active Cooler is an envelope) |

## Envelopes: real CAD needs a manual download

These parts have official CAD only behind a login or a dynamic web viewer, or none at all. The design uses an envelope taken from the datasheet. Download them by hand if you want them in the assembly. In Fusion, Insert → Insert a manufacturer part, or download a STEP and drop it on the matching component.

| Part | Envelope used | Where to get the model |
| --- | --- | --- |
| Feetech STS3250 (femurs) | ST3215 STEP (same housing) | Feetech publishes no STEP. Check your servo against the ST3215 drawing |
| Feetech STS3032 micro bus servo (attachments) | 23.2 × 12.1 × 28.5 mm box (listing dimensions) | Feetech / dealer. No official STEP found |
| RadioMaster RP1 V2 receiver | 10 × 11 × 3.5 mm box + vertical antenna | RadioMaster publishes no STEP |
| LDROBOT LD19 LiDAR | 54 × 46.3 mm base + Ø38.6 × 26.8 mm head (Waveshare listing; Core Electronics lists 38.6 × 38.9 × 33.5) | LDROBOT / Waveshare wiki drawings. **Measure yours.** |
| Omron D2F-01 foot switch | 12.8 × 5.8 × 6.5 mm + pin plunger + PCB pins (TME / RS listings) | Omron component site (registration) |
| Raspberry Pi Active Cooler | 50 × 40 × 10 mm box on the Pi | raspberrypi.com product brief drawing |
| Gens Ace 3S 4000 mAh LiPo | 138.5 × 43 × 24.5 mm box (listing dimensions) | No official CAD. Measure your pack |
| HC-SR04 ultrasonic sensor | PCB + two Ø16 transducers | SparkFun product page drawings. Models on GrabCAD (login) |
| XT60 plugs and panel socket | Boxes from the Amass drawing | Amass (amasshobby.com) offers models after registration |
| KCD1 rocker switch, ATC fuse holder | Boxes | Datasheet drawings. Models on distributor sites (DigiKey/Mouser ↔ SnapEDA/Ultra Librarian, login) |
| Molex 5264 3-pin plugs, JST-XH | Boxes | Molex / JST sites (free login) |
| M3 / M2 screws, heat-set inserts, standoffs | Not modelled (holes and counterbores are) | McMaster-Carr product pages offer STEP downloads in the browser (e.g. M3 × 8 button head 94500A222, tapered insert 94180A331) |
| Bus hubs (perfboard + 5264 headers) | Boxes | Hand-built. No model |
