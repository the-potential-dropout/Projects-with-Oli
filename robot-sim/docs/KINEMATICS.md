# Hexapod Mk1: leg architecture trade study

**Question.** Which leg kinematics give Mk1 what it needs with the fewest, lightest and cheapest actuators? Linkages, coupled legs and passive joints were all considered.

**Hard requirement (Oli, 2026-11-25).** The body must still tilt in roll and pitch, and ideally change height. So **every leg needs independent control of its foot height**. Any design that couples legs in height is out.

**Nice to have.** Strafing (crab walk).

**The rest of the brief.**
- Carry a 0.4 kg attachment (or the 0.3 kg brain module).
- Work on uneven ground.
- Stay a per-leg digital twin of the simulator.
- Be printable and buildable by a student.
- Survive toy battles.

**Answer.** Keep the 3-DOF serial leg (18 bus servos) that rev B already uses. It is the only option that keeps independent foot placement in all three directions, which tilt, terrain work and slip-free walking all need. The weight-saving "smart" moves are already built into its layout (§3). The linkage and coupled alternatives either lose body tilt or trade 6 servos for about 70 extra pivots, foot slip and no strafing (§2).

## 1. Options studied

Mass basis: 2.96 kg robot (rev B2) plus a 0.4 kg attachment. The worst case is a tripod stance with the feet 115 mm out (`cad/load_calc.py`).

| # | Architecture | Actuators | Walk / turn on the spot | Strafe | Body height & tilt | Rough terrain | Per-leg twin | Verdict |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| A | **3-DOF serial**: coxa yaw, femur pitch, tibia pitch (rev B) | **18** | ✓ / ✓ | ✓ | ✓ full (roll, pitch, yaw, height, shift) | ✓ each foot placed in 3-D | ✓ | **Chosen** |
| B | 3-DOF with the knee servo moved to the hip and driving the tibia through a four-bar or parallelogram | 18 | ✓ / ✓ | ✓ | ✓ | ✓ | ✓ (with a coupling equation) | Rejected: knee range too small for this leg (§2.1) |
| C | 2-DOF: hip yaw + lift through a pantograph (foot moves vertically) | 12 | ✓ (feet slip) / ✓ | ✗ (middle legs can't reach sideways) | ✓ height and tilt, but foot radius is fixed | ◐ height adapts, stance width can't | ◐ | Rejected: slip, no strafe, about 70 pivots (§2.2) |
| C′ | Hybrid: 3-DOF front and rear legs, 2-DOF middle legs | 16 | ✓ (middle legs slip) | ◐ | ✓ | ◐ | ◐ | Rejected: saves 2 servos and adds two leg designs |
| D | Coupled tripods: one motor per tripod on a crank or cam shaft | 2–4 | ✓ / ◐ | ✗ | ✗ legs coupled in height | ✗ | ✗ | **Excluded** (no tilt) |
| E | Jansen / Klann / Chebyshev linkage walker, one motor per side | 2 (+1 to steer) | ✓ / skid | ✗ | ✗ fixed foot path | ◐ step-over only | ✗ | **Excluded** (no tilt) |
| F | RHex-style rotating compliant legs | 6 | ✓ / ✓ | ✗ | ✗ can't hold a tilted pose | ✓✓ very good | ✗ | **Excluded** (no tilt; needs continuous-rotation motors) |

### What each one costs here

| # | Leg actuators (cost) | Peak torque need | Moving parts per leg | Print and assembly | Reliability | Fits the top deck and toy battles |
| --- | --- | --- | --- | --- | --- | --- |
| A | 12 × STS3215 + 6 × STS3250 ≈ $880 (all-STS3215 budget build ≈ $420) | femur 7.6 kg·cm at 115 mm reach | 3 revolute joints, all on servo horns | ~6 printed parts per leg, no pivots of its own | High: no linkages, servos are the bearings | ✓ body stays level under recoil (pose controller) |
| B | Same 18 | femur −6 % (0.4 kg·cm) | 3 joints + 4 link pivots | +2 links, 4 shoulder bolts or bearings per leg | Lower: pivot wear and backlash at the foot | ✓ |
| C | 12 × STS3250-class (lift sees about 2× the lever through a 2:1 pantograph) ≈ $750 | lift ≈ 9 kg·cm | 2 joints + ~6 pantograph links and 12 pivots | Many small links and 72 pivots to ream and bolt | Lower: slop accumulates at the foot | ◐ can't brace sideways against recoil |
| D / E | 2–4 gear motors ≈ $40–80 | high (one motor lifts 3 legs) | crank, cams, 8–12 links per leg | many links and bearings | Good, but no control | ✗ no tilt or aiming by posture |
| F | 6 continuous-rotation motors with encoders ≈ $250+ | high transient | 1 joint + compliant leg | simple | Very good | ✗ no tilt; legs sweep through the body envelope |

## 2. Why not the "smarter" linkage legs

### 2.1 Knee servo at the hip (four-bar or parallelogram tibia drive)

Putting the knee actuator near the hip and driving the knee through a linkage is a proven way to cut leg inertia:
- **Humanoid and exoskeleton patents** use a four-bar so the knee motor sits near the hip ([US 11,713,088](https://image-ppubs.uspto.gov/dirsearch-public/print/downloadPdf/11713088), [US 10,792,811](https://image-ppubs.uspto.gov/dirsearch-public/print/downloadPdf/10792811)).
- **Disney's hybrid leg** drives the knee through a five-bar ([paper](https://la.disneyresearch.com/wp-content/uploads/Design-and-Fabrication-of-a-Bipedal-Robot-using-Serial-Parallel-Hybrid-Leg-Mechanism-Paper.pdf)).
- **The Bruce hexapod** drives its joints through four-bar linkages ([arXiv 2011.00523](https://arxiv.org/pdf/2011.00523)).

For Mk1 the gain is small and the cost is decisive:
- **Small gain.** Rev B already points the tibia servo's case back towards the femur axis, so its centre of mass sits about 62 mm from that axis, not 75–90 mm. Moving it onto the axis saves 0.069 kg × 9.81 × 0.062 m ≈ 0.04 N·m (0.43 kg·cm), about 6 % of the femur's 7.6 kg·cm. That isn't enough to drop the STS3250s, which would need the femur below 5 kg·cm for a 2× margin.
- **Decisive cost.** Four-bar knee drives are limited to under about 90° of knee travel ([chain-driven quadruped paper, arXiv 2503.14255](https://arxiv.org/pdf/2503.14255)). Mk1's tibia needs about 165° (sim −135…+30°) to fold for carrying, stand tall and reach down steps. A chain or belt would solve that but adds tensioning, a second idler and more parts per leg.

### 2.2 2-DOF pantograph leg (12 servos)

**It does meet the hard requirement.** The lift joint sets each foot's height, so the body can tilt and change height. Two-DOF legs are a recognised "good compromise between energetic cost and mobility" ([insect-inspired robot review](https://encyclopedia.pub/entry/17490)). Recent work keeps two actuators per leg by adding a passive spring joint ([MDPI Biomimetics 11(2):132, 2026](https://www.mdpi.com/2313-7673/11/2/132)).

**What it gives up on Mk1:**
1. **Fixed foot radius.**
   - A stance foot can only move on a circle round its hip. When the body moves forward by s, the distance from a middle-leg hip to its foot must grow from r to √(r² + s²/4).
   - With r = 100 mm and a 40 mm stride, each foot slips about 2 mm per step, on every step. On carpet that is noise. On slopes and loose ground it is lost traction, and in the simulator it breaks the digital twin, since the sim doesn't model sliding feet.
2. **No strafing.** The middle legs can't move their feet towards or away from the body. The game's strafe and the bot's dodge would have to go.
3. **No stance-width control.** Widening the stance for stability, or narrowing it to cut torque (our 80–115 mm load cap), is impossible.
4. **Mechanics.**
   - A pantograph needs about 6 links and 12 pins per leg (72 per robot), each a source of slop at the foot.
   - The lift actuator works through the pantograph ratio, so it needs an STS3250-class servo anyway.
   - It saves 6 × STS3215 (≈ $130) and costs printing, reaming and bolting 72 pivots.

### 2.3 Coupled and single-motor walkers (excluded by the tilt requirement)

- **Tripod groups on a shared crank shaft, and Jansen / Klann / Chebyshev walkers.** These fix each foot's path relative to the body. With one motor per side, steering comes from changing stride length, which makes the body tilt and bob when it turns ([UT Austin leg-mechanism study](https://cloud.wikis.utexas.edu/wiki/spaces/RMD/pages/51059241)). Klann legs are poor at changing stride while keeping stride height ([same study](https://cloud.wikis.utexas.edu/wiki/x/KRoLAw); [Klann linkage](https://en.wikipedia.org/wiki/Klann_linkage)). None of them can level the body on a slope or aim a turret by tilting.
- **RHex** ([Saranli, Buehler & Koditschek, IJRR 2001](https://www.ri.cmu.edu/publications/rhex-a-simple-and-highly-mobile-hexapod-robot)). Six hip motors spin compliant legs through full turns in an open-loop tripod clock. It is excellent on rough ground and crosses obstacles well beyond its body clearance. But it can't hold a commanded roll, pitch or height, and its full-rotation legs need continuous-rotation motors.
- **Over-actuation isn't a problem for A.** Any legged robot with more than six actuators in a given stance set is redundant ([Kanner et al., IEEE Access 2017](https://www.eng.yale.edu/grablab/pubs/Kanner_ACCESS2017.pdf)). Mk1 controls position, and the stance cap and compliance settings (CONTROL.md) stop the legs fighting each other.

## 3. The chosen leg: what makes it "smart"

| Design choice | Effect |
| --- | --- |
| Femur servo housed **inside** the coxa bracket, case pointing down | Coxa-to-femur axis only 45 mm; the femur servo's 74.5 g sits on the coxa axis line, not out on the leg |
| Tibia servo case points **back** towards the femur axis | Its mass is about 62 mm from the femur axis, not 75–110 mm. Femur torque −0.2 to −0.4 kg·cm, and femur-axis inertia about 35 % lower than with the case pointing down the tibia |
| Tibia horn arms **outboard** of the femur arms | 165° of knee travel, so the leg folds flat for carrying and reaches down steps |
| STS3250 only where it's needed (femur) | The highest-torque joint gets 50 kg·cm stall; coxa and tibia use the cheaper STS3215 |
| Stance cap 80–115 mm, enforced by the gait | Keeps the femur at ≥ 2× rated torque without bigger servos |
| Serial bus servos | One 3-wire daisy chain per leg; every joint reports position, load, voltage and temperature, which the controller and the twin use (CONTROL.md) |
| Foot switch in the tibia tip, sealed under the TPU boot (rev B2) | Reliable touchdown sensing for terrain work; the lead crosses every joint on its axis (WIRING.md) |

**What it can and can't do:**
- **Can:** walk and turn at the same time, turn on the spot and strafe. Hold any body pose within the limits (roll ±15°, pitch ±15°, yaw ±20°, height 60–110 mm, shift ±25 mm). Level the body fully on slopes up to about 10° (and walk on 20° slopes leaning by the excess), step onto 40 mm obstacles and down about 27 mm ones at terrain height, about 42 mm with the body lowered (CONTROL.md §8), fold its legs, and give the sim per-leg control.
- **Can't:** run (servo speed), jump, or carry more than 0.4 kg on top within the 2× margin.

**For the simulator:** no change. It stays 3 DOF per leg, the same joint axes, link lengths (45 / 75 / 120 mm) and joint limits as `hexapod.config.js`. There are no passive joints and no coupling equations; the only coupling is the joint-limit rule `tibiaFoldWhenFemurDown`.

## 4. Power by architecture (rev B2 battery: 3S 4000 mAh, 80 % usable = 3.2 Ah)

| Architecture | Stand | Walk (flat) | Peak | Walking runtime | With the brain module (+1.3–1.6 A) |
| --- | --- | --- | --- | --- | --- |
| A 18 × STS (chosen) | ≈ 2 A | 4–6 A | ≈ 25 A | 32–48 min | 26–36 min |
| C 12 × STS3250-class | ≈ 1.5 A | 3–5 A | ≈ 18 A | 38–64 min | 30–42 min |
| D / E 2–4 gear motors | ≈ 0.2 A | 2–3 A | ≈ 10 A | 64–96 min | 45–62 min |
| F 6 × hip motors | ≈ 0.5 A | 4–8 A (running gait) | ≈ 30 A | 24–48 min | 20–35 min |

Sizing rules (same for any architecture):
- Pack ≥ (walking A + 0.3 A logic + brain) × 0.5 h ÷ 0.8.
- Fuse at about 0.8 × the pack's continuous rating, and above the peak, or the servos' start-up surge trips it.
- Main switch and wiring sized for the fuse.
- The buck sized for every 5 V load at once.

Full figures for the chosen design are in ELECTRONICS.md and WIRING.md.

## Sources

- [Insect-inspired robots review (encyclopedia.pub)](https://encyclopedia.pub/entry/17490)
- [MDPI Biomimetics 11(2):132 (2026), hexapods with underactuated legs](https://www.mdpi.com/2313-7673/11/2/132)
- [Kanner et al., IEEE Access 2017 (Yale GRAB lab)](https://www.eng.yale.edu/grablab/pubs/Kanner_ACCESS2017.pdf)
- [Saranli, Buehler & Koditschek, RHex, IJRR 20(7) 2001](https://www.ri.cmu.edu/publications/rhex-a-simple-and-highly-mobile-hexapod-robot)
- [UT Austin: analysis of walking leg mechanisms](https://cloud.wikis.utexas.edu/wiki/spaces/RMD/pages/51059241)
- [Klann linkage](https://en.wikipedia.org/wiki/Klann_linkage)
- [Bruce dynamic hexapod, arXiv 2011.00523](https://arxiv.org/pdf/2011.00523)
- [Chain-driven sandwich-legged quadruped, arXiv 2503.14255](https://arxiv.org/pdf/2503.14255)
- [Disney Research: serial-parallel hybrid leg](https://la.disneyresearch.com/wp-content/uploads/Design-and-Fabrication-of-a-Bipedal-Robot-using-Serial-Parallel-Hybrid-Leg-Mechanism-Paper.pdf)
- [US 11,713,088 (leg mechanism)](https://image-ppubs.uspto.gov/dirsearch-public/print/downloadPdf/11713088); [US 10,792,811 (multi-bar exoskeleton)](https://image-ppubs.uspto.gov/dirsearch-public/print/downloadPdf/10792811)
