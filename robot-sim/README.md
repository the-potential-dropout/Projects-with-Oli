# Robot Sim: hexapod digital twin

A browser simulator for the hexapod robot we're building for real. Drive it like a video game, swap its attachments, and battle a bot-controlled hexapod. It runs on phones, tablets and computers in Safari, Chrome, Samsung Internet, Edge and Firefox.

## What's in it

- **Hexapod digital twin.** 6 legs and 18 servos walk with a real tripod gait, using the same dimensions as the physical robot ([src/robots/hexapod.config.js](src/robots/hexapod.config.js)).
- **Swappable attachments**
  - Top mount: Toy Laser Cannon, Potato Cannon, or Robotic Arm (4-DOF + gripper)
  - Front bracket: Sonar Sensor (HC-SR04) or Spinning LiDAR
- **Battle vs bot.** The bot drives with exactly the same controls you do. Three difficulty levels; it flanks around cover, leads its potato shots and keeps its distance.
- **Free drive.** Practice targets to knock over.
- **Controls** for keyboard + mouse, gamepads (Xbox / PlayStation / Switch Pro) and touch joysticks.
- **Digital twin panel** (⚙ button or `T`). Shows the live servo angles. In Chrome or Edge on a computer it can also stream them over USB to the real robot (see [docs/BUILD.md](docs/BUILD.md)).
- **Room to grow.** A Rover and a Big Robotic Arm are already listed in the garage. Add a robot in [src/robots/index.js](src/robots/index.js).

## How it's built

| Part | Tech | Where |
| --- | --- | --- |
| Fast simulation core: leg inverse kinematics, tripod gait, projectile physics, ray casts, sensor scans, collisions | **Rust** compiled to **WebAssembly** | [core/](core/) |
| 3D rendering, robots, attachments, UI | JavaScript + [three.js](https://threejs.org) | [src/](src/) |
| Dev server and bundler | [Vite](https://vite.dev) | |

The compiled WebAssembly is checked in (`src/core-pkg/`), so you only need Rust if you change the Rust code.

## Run it

```bash
npm install
npm run dev
```

Open the address it prints. To try it on a phone or tablet on the same Wi-Fi, use the `Network:` address.

## Change the Rust core

One-time setup:

```bash
rustup target add wasm32-unknown-unknown
cargo install wasm-bindgen-cli --version 0.2.130
```

Then after editing anything in `core/`:

```bash
npm run test:core
npm run build:core
```

## Controls

| | Keyboard + mouse | Gamepad | Touch |
| --- | --- | --- | --- |
| Walk / strafe | `W` `S` / `A` `D` | Left stick | Left stick |
| Turn | `Q` `E` or `←` `→`, or mouse | Right stick ↔ | Right stick ↔ |
| Aim up / down | `↑` `↓` or `R` `F`, or mouse | Right stick ↕ | Right stick ↕ |
| Fire | `Space` / left click | RT or A | FIRE |
| Alt (gripper) | `G` / right click | LT or RB | ALT |
| Sprint | `Shift` | L3 or LB | Push left stick to the edge |
| Camera / twin panel / pause | `C` / `T` / `Esc` | Start = pause | Buttons, top right |

## Folder map

```
core/                 Rust: ik.rs, gait.rs, world.rs (+ unit tests)
src/core-pkg/         Generated WebAssembly + JS bindings (npm run build:core)
src/robots/           Robot types: hexapod.js, hexapod.config.js, index.js (registry)
src/attachments.js    Laser, potato cannon, arm, sonar, LiDAR
src/bot.js            Bot AI
src/game.js           Main loop, camera, HUD, match logic
src/input.js          Keyboard, mouse, gamepad, touch
src/twin.js           Digital twin panel + USB (Web Serial) link
firmware/             Robot firmware: hexapod_controller (Feather ESP32-S3: gait, balance, terrain,
                      radio, safety), bus_bridge (servo board), test (PC unit tests), hexapod_bridge (rev B)
radio/                EdgeTX Lua screens + channel map for the RadioMaster / ExpressLRS radio
autonomy/             MuJoCo model (sim2real), training env, URDF for ROS 2
cad/                  Fusion generator, STLs, sim meshes, vendor CAD fetcher, load / sweep / cut-list scripts
docs/BUILD.md         How to build, wire, charge and flash the real robot
docs/DESIGN.md        Mechanical design and load check     docs/KINEMATICS.md   leg architecture study
docs/CONTROL.md       Onboard control system               docs/WIRING.md       every wire, cut list
docs/ELECTRONICS.md   Shopping list                        docs/AUTONOMY.md     radio, brain module, ROS 2, MuJoCo
docs/notebook/        Engineering design notebook (python docs/notebook/build_notebook.py)
```
