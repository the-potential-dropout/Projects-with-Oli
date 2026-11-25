// Single source of truth for the real hexapod's dimensions. The simulator,
// the Fusion model and the servo mapping all read from here, so if you change
// a part in real life, change it here too and the digital twin stays honest.
//
// Units: metres and degrees. Robot faces +Z, +X is the robot's LEFT side.

const deg = Math.PI / 180;

export const HEXAPOD = {
  name: 'Hexapod Mk1',

  body: {
    length: 0.26, // front-to-back of the main plate
    width: 0.14,
    thickness: 0.005,
    plateGap: 0.045, // distance between the top and bottom plates (servos sit between)
    lobeRadius: 0.021, // plate lobe round each coxa axis so the servo tabs have material (Fusion: body_lobeRadius)
    massKg: 2.7, // CAD estimate: 18 x MG996R + ~1.1 kg printed PLA + battery + electronics (docs/DESIGN.md)
  },

  // Leg segment lengths, measured joint-to-joint (coxa axis to femur axis, etc.)
  leg: {
    coxa: 0.045,
    femur: 0.075,
    tibia: 0.12,
    footReach: 0.13, // horizontal distance coxa joint -> foot when standing
  },

  standHeight: 0.09, // belly height above the ground while walking

  // Joint limits from the Fusion collision sweep, in simulator degrees:
  // coxa 0 = along the splay; femur 0 = level, + = up; tibia relative to the
  // femur, 0 = straight out, -90 = hanging vertically, more negative = folded in.
  // Folding the tibia past -135 hits the coxa and the femur's servo cradle,
  // so the gait IK refuses to go further (core/src/ik.rs solve_leg_limited).
  jointLimits: { coxa: [-45, 45], femur: [-35, 90], tibia: [-135, 30] },
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
  // topMount: datum = top of the quick-release adapter plate, 13 mm above the top plate (docs/DESIGN.md)
  // frontMount: HC-SR04 centre sits 12 mm below the body mid-plane, sensor face 135 mm forward
  topMount: [0, 0.013, -0.01],
  frontMount: [0, -0.012, 0.135],

  // Servo wiring for the real robot: which PWM channel drives which joint and
  // how a simulator angle maps to a servo angle (0-180). Adjust "trim" after
  // building so that 90 deg on the servo matches the sim's neutral pose.
  servos: {
    // per leg: [coxa, femur, tibia] channel numbers (e.g. two PCA9685 boards: 0-15 and 16-31)
    channels: [
      [0, 1, 2],
      [4, 5, 6],
      [8, 9, 10],
      [16, 17, 18],
      [20, 21, 22],
      [24, 25, 26],
    ],
    // direction multiplier: right-side servos are mirrored
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
