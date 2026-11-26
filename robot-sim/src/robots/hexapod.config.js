// Single source of truth for the real hexapod's dimensions. The simulator,
// the Fusion model and the servo mapping all read from here, so if you change
// a part in real life, change it here too and the digital twin stays honest.
//
// Units: metres and degrees. Robot faces +Z, +X is the robot's LEFT side.

const deg = Math.PI / 180;

export const HEXAPOD = {
  name: 'Hexapod Mk1',
  revision: 'B2',
  // 'rc' = default RC-only build (all STS3215, attachments <= 0.2 kg); 'autonomy' = STS3250 femurs + brain module (docs/VERSIONS.md)
  build: 'rc', // rev B2: + Feather ESP32-S3 onboard control, IMU, foot switches, ELRS radio, brain module (docs/CONTROL.md, AUTONOMY.md)

  body: {
    length: 0.26, // front-to-back of the main plate
    width: 0.14,
    thickness: 0.005,
    // The coxa servos are clamped between the plates, so the gap is the servo's
    // case height. The plates are NOT symmetric about the coxa-joint height (the
    // body origin): top plate top face at +0.018825, bottom plate bottom face at -0.021575.
    plateGap: 0.0304,
    plateTop: 0.018825, // top face of the top plate above the coxa-joint height
    plateBottom: -0.021575, // bottom face of the bottom plate
    coxaInset: 0.0115, // plate edge sits this far inside each coxa axis (Fusion: body_coxaInset)
    massKg: 2.93, // RC build CAD estimate (cad/build_report.json; autonomy build 2.96, cad/build_report_autonomy.json): servos, 3S 4000 mAh LiPo, ~1.05 kg printed, harness, pods; brain module +0.30
  },

  // Leg segment lengths, measured joint-to-joint (coxa axis to femur axis, etc.)
  leg: {
    coxa: 0.045,
    femur: 0.075,
    tibia: 0.12,
    footReach: 0.1, // horizontal distance coxa joint -> foot when standing
    // Load check (docs/DESIGN.md, cad/load_calc.py): keep every stance foot between
    // these horizontal distances from its coxa axis for a 2x servo torque margin.
    footReachMin: 0.08,
    footReachMax: 0.115,
  },

  standHeight: 0.09, // coxa-joint height above the ground while walking (lowest part, the coxa bracket, is 0.0385 below it)

  // Joint limits from the Fusion collision sweep, in simulator degrees:
  // coxa 0 = along the splay; femur 0 = level, + = up; tibia relative to the
  // femur, 0 = straight out, -90 = hanging vertically, more negative = folded in.
  // Rev B sweep (cad/interference_report.json), parts AND harness checked:
  // tibia folds to -135 while the femur is level or raised; with the femur up to
  // 10 deg down it may only fold to -132 (tibiaFoldWhenFemurDown).
  jointLimits: { coxa: [-45, 45], femur: [-10, 70], tibia: [-135, 30] },
  tibiaFoldWhenFemurDown: -132,
  hitRadius: 0.17, // for battle collisions
  footprintRadius: 0.2, // for bumping into walls and other robots

  // Coxa joint positions on the body plate: [x, z] in metres.
  // Order: 0 front-right, 1 middle-right, 2 rear-right, 3 front-left, 4 middle-left, 5 rear-left
  mounts: [
    [-0.06, 0.11],
    [-0.08, 0.0],
    [-0.06, -0.11],
    [0.06, 0.11],
    [0.08, 0.0],
    [0.06, -0.11],
  ],
  // Direction each leg points out from the body (degrees, 0 = straight out sideways)
  legSplay: [35, 0, -35, 35, 0, -35],

  speed: { walk: 0.26, sprint: 0.48, turn: 1.6 }, // m/s and rad/s
  stepHz: { walk: 1.7, sprint: 2.6 },

  // Top mount for the swappable attachment (turret plate) and front sensor bracket
  // topMount: datum = top of the quick-release adapter plate, 13 mm above the top
  // plate = 0.031825 above the coxa-joint height. hexapod.js adds plateGap/2 +
  // thickness (0.0202) to topMount[1], hence 0.0116 here.
  // frontMount: HC-SR04 centre 4 mm above the coxa-joint height, sensor face 135 mm forward
  topMount: [0, 0.0116, -0.01],
  // autonomy build only: top-mount module that carries the autonomy computer (cad/mesh/brain_module.glb, body frame)
  brainModule: { massKg: 0.3, mesh: 'brain_module' },
  frontMount: [0, 0.004, 0.135],

  // Servo wiring for the real robot: which PWM channel drives which joint and
  // how a simulator angle maps to a servo angle (0-180). Adjust "trim" after
  // building so that 90 deg on the servo matches the sim's neutral pose.
  servos: {
    // Rev B uses Feetech serial-bus servos. busIds = the ID set in each servo
    // (firmware/hexapod_bridge): leg L joint J -> 1 + 3L + J; attachments 21-25.
    // The USB protocol is unchanged (S,<deg 0-180>,...), so `channels` is kept
    // only for the old PCA9685 wiring and is not used by the rev B firmware.
    busIds: [
      [1, 2, 3],
      [4, 5, 6],
      [7, 8, 9],
      [10, 11, 12],
      [13, 14, 15],
      [16, 17, 18],
    ],
    auxBusIds: [21, 22, 23, 24, 25],
    // per leg: [coxa, femur, tibia] channel numbers (old two-PCA9685 wiring)
    channels: [
      [0, 1, 2],
      [4, 5, 6],
      [8, 9, 10],
      [16, 17, 18],
      [20, 21, 22],
      [24, 25, 26],
    ],
    // direction multiplier: right-side servos are mirrored. Rev B: confirm on the
    // stand (BUILD.md) - the bus servos' positive direction depends on how each
    // one is mounted; flip a sign here if a joint moves the wrong way.
    direction: [
      [1, -1, -1],
      [1, -1, -1],
      [1, -1, -1],
      [1, 1, 1],
      [1, 1, 1],
      [1, 1, 1],
    ],
    trim: Array.from({ length: 6 }, () => [0, 0, 0]),
  },

  // Onboard controller (firmware/hexapod_controller, docs/CONTROL.md). The simulator can
  // mirror these so the twin balances, adapts to terrain and chooses gaits like the robot.
  control: {
    gaits: {
      // leg phase offsets in leg order (FR, MR, RR, FL, ML, RL) and swing fraction
      tripod: { offsets: [0, 0.5, 0, 0.5, 0, 0.5], swing: 0.5 },
      ripple: { offsets: [2 / 3, 1 / 3, 0, 1 / 6, 5 / 6, 0.5], swing: 1 / 3 },
      wave: { offsets: [2 / 6, 1 / 6, 0, 5 / 6, 4 / 6, 3 / 6], swing: 1 / 6 },
    },
    stepHeight: 0.025, // m, capped near the body by the tibia fold limit (lift cap)
    terrain: { bodyHeight: 0.105, stepHeight: 0.045, stepUpMax: 0.04, stepDownMax: 0.05, footDepthMax: 0.1316 /* search depth = min(stepDownMax, footDepthMax - height) */, searchSpeed: 0.15, contactN: 1.5 },
    pose: { rollMax: 15, pitchMax: 15, yawMax: 20, heightMin: 0.06, heightMax: 0.11, shiftMax: 0.025 }, // deg, m
    balance: { kp: 0.6, ki: 2.0, kd: 0.05, iMaxDeg: 10, outMaxDeg: 10, slewDegPerS: 60, dLowpassHz: 8 },
    stabilityMarginMin: 0.025, // m: below this the tripod gait switches to ripple
    tipDeg: 35,
    slopeMaxDeg: 20,
    loopHz: 100,
  },
};

/** Coxa mounts as the flat [x, z, yaw] * 6 array the Rust gait expects. */
export function gaitMounts(cfg = HEXAPOD) {
  const out = [];
  cfg.mounts.forEach(([x, z], i) => {
    const out_dir = x < 0 ? Math.PI : 0; // right legs point -X, left legs +X
    const splay = cfg.legSplay[i] * deg;
    // Right side: splay forward means rotating toward +Z from -X
    const yaw = x < 0 ? out_dir - splay : out_dir + splay;
    out.push(x, z, yaw);
  });
  return out;
}

/** Convert simulator joint angles (radians) for one leg into servo degrees. */
export function legToServo(leg, coxa, femur, tibia, cfg = HEXAPOD) {
  const dir = cfg.servos.direction[leg];
  const trim = cfg.servos.trim[leg];
  const c = 90 + dir[0] * (coxa / deg) + trim[0];
  const f = 90 + dir[1] * (femur / deg) + trim[1];
  const t = 90 + dir[2] * (tibia / deg + 90) + trim[2];
  const clamp = (v) => Math.max(0, Math.min(180, Math.round(v)));
  return [clamp(c), clamp(f), clamp(t)];
}
