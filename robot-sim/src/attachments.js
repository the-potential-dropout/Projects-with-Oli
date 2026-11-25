// Swappable attachments. Each one plugs into a slot on the robot ("top" turret
// plate or "front" sensor bracket), has a 3D model, and does something every
// frame. To add a new attachment: add an entry to ATTACHMENTS and a builder
// function below that returns { group, update(dt, cmd, game), hud?(), servos?() }.
import * as THREE from 'three';
import { solve_arm_2link } from './core.js';
import { hexapodMesh } from './robots/hexapod.meshes.js';

export const ATTACHMENTS = {
  none: { slot: 'any', name: 'Empty', desc: 'Nothing mounted.' },
  laser: { slot: 'top', name: 'Toy Laser Cannon', desc: 'Rapid-fire light beam. Hits instantly in a straight line. Real build: laser-tag IR emitter + red LED.' },
  potato: { slot: 'top', name: 'Potato Cannon', desc: 'Hold fire to charge, release to launch. Arcs with gravity, hits hard. Real build: spring-loaded foam-ball launcher.' },
  arm: { slot: 'top', name: 'Robotic Arm', desc: '4-DOF arm with gripper. Fire = punch, Alt = open/close gripper. Real build: 5 x MG90S servos.' },
  sonar: { slot: 'front', name: 'Sonar Sensor', desc: 'Ultrasonic distance sensor (HC-SR04). Sees what is right in front of you, up to 4 m.' },
  lidar: { slot: 'front', name: 'Spinning LiDAR', desc: '360° laser scanner. Maps the arena and spots enemies all around, up to 6 m.' },
};

export function attachmentsForSlot(slot) {
  return Object.entries(ATTACHMENTS)
    .filter(([, a]) => a.slot === slot || a.slot === 'any')
    .map(([id, a]) => ({ id, ...a }));
}

export function createAttachment(id, robot) {
  switch (id) {
    case 'laser':
      return laserCannon(robot);
    case 'potato':
      return potatoCannon(robot);
    case 'arm':
      return roboticArm(robot);
    case 'sonar':
      return sonar(robot);
    case 'lidar':
      return lidar(robot);
    default:
      return null;
  }
}

const M = {
  dark: new THREE.MeshStandardMaterial({ color: 0x24272e, roughness: 0.6, metalness: 0.3 }),
  metal: new THREE.MeshStandardMaterial({ color: 0xaeb6bf, roughness: 0.35, metalness: 0.7 }),
  red: new THREE.MeshStandardMaterial({ color: 0xff2a2a, emissive: 0xff0000, emissiveIntensity: 1.5 }),
  green: new THREE.MeshStandardMaterial({ color: 0x3ddc84, roughness: 0.5 }),
  tan: new THREE.MeshStandardMaterial({ color: 0xc8a165, roughness: 0.8 }),
  sensor: new THREE.MeshStandardMaterial({ color: 0x1a6fd1, roughness: 0.4 }),
  silver: new THREE.MeshStandardMaterial({ color: 0xdddddd, roughness: 0.25, metalness: 0.9 }),
};

const _p = new THREE.Vector3();
const _d = new THREE.Vector3();
const _q = new THREE.Quaternion();

function mesh(geo, mat) {
  const m = new THREE.Mesh(geo, mat);
  m.castShadow = true;
  return m;
}

/** Shared turret: a pan base plus a pitch group that follows robot.aimPitch. */
function turretBase(robot, color) {
  const group = new THREE.Group();
  const base = mesh(new THREE.CylinderGeometry(0.035, 0.04, 0.02, 20), M.dark);
  base.position.y = 0.01;
  group.add(base);
  const pitch = new THREE.Group();
  pitch.position.y = 0.035;
  group.add(pitch);
  const cheekGeo = new THREE.BoxGeometry(0.008, 0.035, 0.03);
  for (const x of [-0.03, 0.03]) {
    const cheek = mesh(cheekGeo, color ? new THREE.MeshStandardMaterial({ color }) : M.dark);
    cheek.position.set(x, -0.005, 0);
    group.add(cheek);
  }
  return { group, pitch };
}

function muzzle(obj, outPos, outDir) {
  obj.getWorldPosition(outPos);
  obj.getWorldQuaternion(_q);
  outDir.set(0, 0, 1).applyQuaternion(_q).normalize();
}

function laserCannon(robot) {
  const { group, pitch } = turretBase(robot);
  const barrel = mesh(new THREE.CylinderGeometry(0.009, 0.012, 0.11, 14), M.metal);
  barrel.rotation.x = Math.PI / 2;
  barrel.position.z = 0.05;
  pitch.add(barrel);
  const housing = mesh(new THREE.BoxGeometry(0.045, 0.03, 0.06), M.dark);
  pitch.add(housing);
  const lens = mesh(new THREE.SphereGeometry(0.008, 12, 8), M.red);
  lens.position.z = 0.106;
  pitch.add(lens);
  const tip = new THREE.Object3D();
  tip.position.z = 0.115;
  pitch.add(tip);

  const COOLDOWN = 0.26;
  const DAMAGE = 6;
  const RANGE = 9;
  let cool = 0;
  let heat = 0; // overheats if you hold the trigger too long
  let overheated = false;

  return {
    id: 'laser',
    group,
    reset() {
      cool = 0;
      heat = 0;
      overheated = false;
    },
    update(dt, cmd, game) {
      pitch.rotation.x = -robot.aimPitch;
      cool -= dt;
      heat = Math.max(0, heat - dt * 0.35);
      if (overheated && heat < 0.35) overheated = false;
      if (!robot.alive || !cmd.fire || cool > 0 || overheated) return;
      cool = COOLDOWN;
      heat += 0.12;
      if (heat >= 1) overheated = true;
      muzzle(tip, _p, _d);
      const r = game.world.raycast(_p.x, _p.y, _p.z, _d.x, _d.y, _d.z, RANGE, robot.bodyId);
      const end = new THREE.Vector3(r[2], r[3], r[4]);
      game.effects.beam(_p, end, robot.color);
      game.audio.play('laser');
      if (r[1] >= 0) game.hitBody(r[1], DAMAGE, robot, end);
      else if (r[0] < RANGE) game.effects.sparks(end, 0xffaa55, 6);
    },
    hud() {
      return { label: 'Laser', bar: overheated ? 1 : heat, barColor: overheated ? '#ff4d4d' : '#ffb84d', note: overheated ? 'OVERHEATED' : 'Hold FIRE' };
    },
    servos: () => [Math.round(90 + (robot.aimPitch * 180) / Math.PI)],
  };
}

function potatoCannon(robot) {
  const { group, pitch } = turretBase(robot);
  const tube = mesh(new THREE.CylinderGeometry(0.022, 0.022, 0.16, 18, 1, true), M.green);
  tube.material = tube.material.clone();
  tube.material.side = THREE.DoubleSide;
  tube.rotation.x = Math.PI / 2;
  tube.position.z = 0.05;
  pitch.add(tube);
  const chamber = mesh(new THREE.CylinderGeometry(0.03, 0.03, 0.06, 18), M.dark);
  chamber.rotation.x = Math.PI / 2;
  chamber.position.z = -0.04;
  pitch.add(chamber);
  const loaded = mesh(new THREE.SphereGeometry(0.019, 12, 10), M.tan);
  loaded.position.z = 0.1;
  pitch.add(loaded);
  const tip = new THREE.Object3D();
  tip.position.z = 0.135;
  pitch.add(tip);

  const COOLDOWN = 1.0;
  let cool = 0;
  let charge = 0;
  let charging = false;
  const preview = new THREE.Line(new THREE.BufferGeometry().setFromPoints(Array.from({ length: 40 }, () => new THREE.Vector3())), new THREE.LineDashedMaterial({ color: 0xffffff, dashSize: 0.06, gapSize: 0.05, transparent: true, opacity: 0.7 }));
  preview.frustumCulled = false;
  preview.visible = false;

  const speedFor = (c) => 4.5 + 6.5 * c;

  return {
    id: 'potato',
    group,
    reset() {
      cool = 0;
      charge = 0;
      charging = false;
    },
    dispose() {
      preview.removeFromParent();
    },
    update(dt, cmd, game) {
      pitch.rotation.x = -robot.aimPitch;
      cool -= dt;
      loaded.visible = cool <= 0;
      if (!preview.parent && game.showPreview(robot)) game.scene.add(preview);
      if (!robot.alive) {
        charging = false;
        preview.visible = false;
        return;
      }
      if (cmd.fire && cool <= 0) {
        charging = true;
        charge = Math.min(1, charge + dt / 1.1);
      }
      const release = charging && (!cmd.fire || (cmd.fireCharge !== undefined && charge >= cmd.fireCharge));
      if (charging && game.showPreview(robot)) {
        // Trajectory preview (no drag, close enough for aiming)
        muzzle(tip, _p, _d);
        const v = speedFor(charge);
        const pos = preview.geometry.attributes.position;
        for (let i = 0; i < pos.count; i++) {
          const t = i * 0.04;
          pos.setXYZ(i, _p.x + _d.x * v * t, Math.max(0, _p.y + _d.y * v * t - 4.905 * t * t), _p.z + _d.z * v * t);
        }
        pos.needsUpdate = true;
        preview.computeLineDistances();
        preview.visible = true;
      } else preview.visible = false;

      if (release) {
        muzzle(tip, _p, _d);
        const v = speedFor(charge);
        game.world.spawn_projectile(_p.x, _p.y, _p.z, _d.x * v, _d.y * v, _d.z * v, 0.025, 0.02, robot.bodyId, 12 + 14 * charge, 1);
        game.audio.play('potato');
        game.effects.puff(_p);
        charging = false;
        charge = 0;
        cool = COOLDOWN;
        robot.aimKick = 0.06;
      }
    },
    hud() {
      return { label: 'Potato', bar: cool > 0 ? 1 - cool / COOLDOWN : charge, barColor: cool > 0 ? '#888' : '#8fdc5b', note: cool > 0 ? 'Reloading…' : charging ? `Power ${Math.round(charge * 100)}%` : 'Hold FIRE to charge' };
    },
    servos: () => [Math.round(90 + (robot.aimPitch * 180) / Math.PI)],
  };
}

function roboticArm(robot) {
  const UPPER = 0.12;
  const FORE = 0.12;
  const group = new THREE.Group();
  const base = mesh(new THREE.CylinderGeometry(0.03, 0.035, 0.025, 20), M.dark);
  base.position.y = 0.012;
  group.add(base);
  const yaw = new THREE.Group();
  yaw.position.y = 0.035;
  group.add(yaw);
  const shoulder = new THREE.Group();
  yaw.add(shoulder);
  // Links point along +Z (forward) and pitch about X
  const link = (len, w) => {
    const m = mesh(new THREE.BoxGeometry(w, w * 0.7, len), robot.bodyMat);
    m.position.z = len / 2;
    return m;
  };
  shoulder.add(mesh(new THREE.BoxGeometry(0.04, 0.035, 0.022), M.dark));
  shoulder.add(link(UPPER, 0.02));
  const elbow = new THREE.Group();
  elbow.position.z = UPPER;
  shoulder.add(elbow);
  elbow.add(mesh(new THREE.BoxGeometry(0.036, 0.03, 0.02), M.dark));
  elbow.add(link(FORE, 0.017));
  const wrist = new THREE.Group();
  wrist.position.z = FORE;
  elbow.add(wrist);
  wrist.add(mesh(new THREE.BoxGeometry(0.03, 0.022, 0.02), M.dark));
  const fingers = [];
  for (const side of [-1, 1]) {
    const f = new THREE.Group();
    f.position.set(side * 0.008, 0, 0.012);
    const m = mesh(new THREE.BoxGeometry(0.004, 0.016, 0.035), M.metal);
    m.position.z = 0.017;
    f.add(m);
    wrist.add(f);
    fingers.push({ f, side });
  }
  const tip = new THREE.Object3D();
  tip.position.z = 0.04;
  wrist.add(tip);

  let punchT = 1; // 0..1 progress, >=1 idle
  let cool = 0;
  let gripClosed = false;
  let grip = 0;
  let hitThisPunch = false;
  const angles = { base: 0, shoulder: 0, elbow: 0, wrist: 0 };

  return {
    id: 'arm',
    group,
    reset() {
      punchT = 1;
      cool = 0;
      gripClosed = false;
    },
    update(dt, cmd, game) {
      cool -= dt;
      if (robot.alive && cmd.altPressed) gripClosed = !gripClosed;
      if (robot.alive && cmd.fire && cool <= 0) {
        punchT = 0;
        cool = 0.55;
        hitThisPunch = false;
        gripClosed = true;
        game.audio.play('whoosh');
      }
      punchT = Math.min(1, punchT + dt / 0.28);
      const ext = punchT < 1 ? Math.sin(punchT * Math.PI) : 0;

      // Target point in the arm's vertical plane: aim pitch raises/lowers it
      const reach = 0.1 + ext * 0.13 + robot.aimPitch * 0.03;
      const height = -0.02 + robot.aimPitch * 0.17 + ext * 0.02;
      const sol = solve_arm_2link(reach, height, UPPER, FORE);
      angles.shoulder = sol[0];
      angles.elbow = sol[1];
      angles.wrist = -(sol[0] + sol[1]);
      shoulder.rotation.x = -angles.shoulder;
      elbow.rotation.x = -angles.elbow;
      wrist.rotation.x = -angles.wrist;
      if (!robot.alive) {
        shoulder.rotation.x = 0.4;
        elbow.rotation.x = 0.6;
      }

      grip += ((gripClosed ? 1 : 0) - grip) * Math.min(1, dt * 10);
      for (const { f, side } of fingers) f.rotation.y = side * (0.35 - grip * 0.33);

      // Melee hit near the top of the punch
      if (robot.alive && punchT > 0.35 && punchT < 0.75 && !hitThisPunch) {
        tip.getWorldPosition(_p);
        for (const target of game.targets()) {
          if (target === robot || !target.alive) continue;
          const c = target.center(_d);
          if (c.distanceTo(_p) < (target.cfg?.hitRadius ?? 0.17) + 0.04) {
            hitThisPunch = true;
            game.hitBody(target.bodyId, 14, robot, _p.clone());
            game.audio.play('clank');
            break;
          }
        }
      }
    },
    hud() {
      return { label: 'Arm', bar: Math.max(0, 1 - Math.max(0, cool) / 0.55), barColor: '#6fc3ff', note: `Gripper ${gripClosed ? 'closed' : 'open'} · FIRE punch · ALT grip` };
    },
    servos: () => {
      const d = (r) => Math.round(Math.max(0, Math.min(180, 90 + (r * 180) / Math.PI)));
      return [d(angles.base), d(angles.shoulder), d(angles.elbow), d(angles.wrist), Math.round(30 + grip * 70)];
    },
  };
}

function sensorContactUpdate(robot, game, scan, yaw, fov, rays, maxDist) {
  const contacts = [];
  for (let i = 0; i < rays; i++) {
    const id = scan[i * 2 + 1];
    if (id >= 0) {
      const a = rays === 1 ? yaw : yaw - fov / 2 + (fov * i) / (rays - 1);
      contacts.push({ bodyId: id, dist: scan[i * 2], angle: a });
    }
  }
  robot.sensor = { ...(robot.sensor || {}), contacts, maxDist };
}

function sonar(robot) {
  const group = new THREE.Group();
  // Real HC-SR04 from the CAD (modelled in the body frame, so undo the bracket offset)
  const cad = hexapodMesh('sonar_example');
  if (cad) {
    cad.position.fromArray(robot.cfg.frontMount).negate();
    group.add(cad);
  } else {
    const board = mesh(new THREE.BoxGeometry(0.045, 0.02, 0.004), M.sensor);
    group.add(board);
    for (const x of [-0.013, 0.013]) {
      const can = mesh(new THREE.CylinderGeometry(0.008, 0.008, 0.012, 16), M.silver);
      can.rotation.x = Math.PI / 2;
      can.position.set(x, 0, 0.008);
      group.add(can);
    }
  }
  const FOV = (30 * Math.PI) / 180;
  const MAX = 4;
  const RAYS = 9;
  const coneGeo = new THREE.ConeGeometry(Math.tan(FOV / 2), 1, 24, 1, true);
  coneGeo.translate(0, -0.5, 0);
  coneGeo.rotateX(-Math.PI / 2);
  const cone = new THREE.Mesh(coneGeo, new THREE.MeshBasicMaterial({ color: 0x66ccff, transparent: true, opacity: 0.08, depthWrite: false, side: THREE.DoubleSide }));
  cone.position.z = 0.01;
  group.add(cone);
  let t = 0;
  let reading = MAX;
  return {
    id: 'sonar',
    group,
    update(dt, cmd, game) {
      cone.visible = robot.alive && game.showPreview(robot);
      t -= dt;
      if (t > 0 || !robot.alive) return;
      t = 0.08;
      group.getWorldPosition(_p);
      const scan = game.world.scan(_p.x, _p.y, _p.z, robot.yaw, FOV, RAYS, MAX, robot.bodyId);
      reading = MAX;
      for (let i = 0; i < RAYS; i++) reading = Math.min(reading, scan[i * 2]);
      sensorContactUpdate(robot, game, scan, robot.yaw, FOV, RAYS, MAX);
      robot.sensor.kind = 'sonar';
      robot.sensor.reading = reading;
      robot.sensor.fov = FOV;
      cone.scale.setScalar(Math.max(0.05, reading));
    },
    hud() {
      return { label: 'Sonar', bar: 1 - reading / MAX, barColor: reading < 0.5 ? '#ff4d4d' : '#66ccff', note: reading >= MAX ? 'Nothing ahead' : `${reading.toFixed(2)} m ahead` };
    },
  };
}

function lidar(robot) {
  const group = new THREE.Group();
  const mast = mesh(new THREE.CylinderGeometry(0.006, 0.006, 0.07, 8), M.dark);
  mast.position.set(0, 0.035, -0.02);
  group.add(mast);
  const head = new THREE.Group();
  head.position.set(0, 0.075, -0.02);
  group.add(head);
  head.add(mesh(new THREE.CylinderGeometry(0.028, 0.03, 0.022, 24), M.dark));
  const win = mesh(new THREE.BoxGeometry(0.02, 0.01, 0.006), M.red);
  win.position.z = 0.028;
  head.add(win);
  const RAYS = 120;
  const MAX = 6;
  let t = 0;
  let points = new Float32Array(RAYS * 2);
  return {
    id: 'lidar',
    group,
    update(dt, cmd, game) {
      if (robot.alive) head.rotation.y += dt * 18;
      t -= dt;
      if (t > 0 || !robot.alive) return;
      t = 0.1;
      head.getWorldPosition(_p);
      const scan = game.world.scan(_p.x, _p.y, _p.z, robot.yaw, Math.PI * 2 * (1 - 1 / RAYS), RAYS, MAX, robot.bodyId);
      for (let i = 0; i < RAYS; i++) {
        points[i * 2] = scan[i * 2];
        points[i * 2 + 1] = scan[i * 2 + 1];
      }
      sensorContactUpdate(robot, game, scan, robot.yaw, Math.PI * 2 * (1 - 1 / RAYS), RAYS, MAX);
      robot.sensor.kind = 'lidar';
      robot.sensor.points = points;
      robot.sensor.rays = RAYS;
      robot.sensor.fov = Math.PI * 2 * (1 - 1 / RAYS);
    },
    hud() {
      const near = robot.sensor?.contacts?.length ? Math.min(...robot.sensor.contacts.map((c) => c.dist)) : null;
      return { label: 'LiDAR', bar: 1, barColor: '#ff6b6b', note: near !== null ? `Contact ${near.toFixed(1)} m` : 'Scanning 360°' };
    },
  };
}
