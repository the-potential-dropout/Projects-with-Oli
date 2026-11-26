# Robot Sim: hexapod digital twin

A browser simulator for the hexapod robot we're building for real. Drive it like a video game, swap its attachments, and battle a bot-controlled hexapod. It runs on phones, tablets and computers in Safari, Chrome, Samsung Internet, Edge and Firefox.

## What's in it

- **Hexapod digital twin.** 6 legs and 18 servos, with the same dimensions and joint limits as the physical robot ([src/robots/hexapod.config.js](src/robots/hexapod.config.js)). The Rust core runs the same maths as the robot's own controller ([firmware/hexapod_controller](firmware/hexapod_controller), [docs/CONTROL.md](docs/CONTROL.md)):
  - **Gaits:** tripod, ripple and wave, with the load-check stance window. The robot slows down rather than overload a servo.
  - **Body pose:** roll, pitch, yaw and height.
  - **Balance:** the PI-D controller levels the body using a simulated IMU.
  - **Terrain mode:** foot switches sense the ground and the legs feel for it.
  - **Stability:** a margin check that switches to ripple when it gets low, and a tip-over fault.
  - **Battery:** a monitor for the 3S pack.
- **Rough ground.** Bumps, rocks, a stepped pad and a 12° hill. The body rests wherever its feet put it, so without terrain mode it rocks just like the real one.
- **Radio switches and an RC layout.** Mode (manual / pose / auto), gait, terrain mode and body height, as on the robot's RadioMaster. A telemetry panel shows what the radio's screen would.
- **Autonomy.** The bot runs the robot's behaviour tree from [docs/AUTONOMY.md](docs/AUTONOMY.md): find, approach, strafe-circle, aim, fire, retreat. It switches itself to terrain mode when its look-ahead sensor sees rough ground. Put your own robot in **AUTO** and it runs the same tree; touch a stick to take over.
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
| Fast simulation core: leg inverse kinematics, gaits, body pose, terrain adaptation, balance PID, ground height field and body fit, projectile physics, ray casts, sensor scans, collisions | **Rust** compiled to **WebAssembly** | [core/](core/) |
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
| Mode: manual / pose / auto (radio SC) | `M` | Y | MODE |
| Gait: tripod / ripple / wave (SB) | `1` `2` `3` | D-pad ←→ | GAIT |
| Terrain mode (SD) | `V` | Back | TERR |
| Balance on / off | `B` | R3 | |
| Body height | `[` `]` | D-pad ↕ | |

In **pose** mode the walk keys tilt and twist the body instead of walking.

With **Controls: RC radio** in the garage, gamepad and touch sticks work like the robot's radio in Mode 2:
- **Right stick:** walk and strafe.
- **Left stick:** turn (←→) and body height (↕).

## Folder map

```
core/                 Rust: ik.rs, gait.rs (gaits, pose, terrain, balance), terrain.rs, world.rs (+ unit tests)
src/core-pkg/         Generated WebAssembly + JS bindings (npm run build:core)
src/robots/           Robot types: hexapod.js, hexapod.control.js (onboard controller), hexapod.config.js, index.js
src/attachments.js    Laser, potato cannon, arm, sonar, LiDAR
src/bot.js            Behaviour tree (bot opponent and AUTO mode)
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
