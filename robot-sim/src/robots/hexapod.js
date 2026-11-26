// The hexapod's digital twin: 3D model, walking (via the Rust gait), health,
// and two attachment slots (top turret + front sensor bracket).
import * as THREE from 'three';
import { HexapodGait } from '../core.js';
import { HEXAPOD, gaitMounts, legToServo } from './hexapod.config.js';
import { createAttachment } from '../attachments.js';
import { hexapodMesh, hasHexapodMeshes } from './hexapod.meshes.js';
import { HexapodController } from './hexapod.control.js';

const tmpV = new THREE.Vector3();

function mat(color, opts = {}) {
  return new THREE.MeshStandardMaterial({ color, roughness: 0.55, metalness: 0.15, ...opts });
}

const SHARED = {
  servo: mat(0x1d1f24, { roughness: 0.7 }),
  horn: mat(0xe8e8e8, { roughness: 0.4 }),
  aluminium: mat(0xb9c0c8, { metalness: 0.6, roughness: 0.35 }),
  rubber: mat(0x111111, { roughness: 0.95 }),
  pcb: mat(0x0b6b3a, { roughness: 0.6 }),
  battery: mat(0x2b5fbf, { roughness: 0.5 }),
};

/** A box whose origin is at one end so it can hang off a joint. */
function linkBox(len, w, h, material) {
  const m = new THREE.Mesh(new THREE.BoxGeometry(len, h, w), material);
  m.position.x = len / 2;
  m.castShadow = true;
  return m;
}

function servoBox(material = SHARED.servo) {
  const g = new THREE.Group();
  const b = new THREE.Mesh(new THREE.BoxGeometry(0.04, 0.038, 0.02), material);
  b.castShadow = true;
  g.add(b);
  const horn = new THREE.Mesh(new THREE.CylinderGeometry(0.009, 0.009, 0.006, 16), SHARED.horn);
  horn.rotation.x = Math.PI / 2;
  horn.position.set(0.008, 0, 0.013);
  g.add(horn);
  return g;
}

export class Hexapod {
  /**
   * @param {object} o
   * @param {THREE.Scene} o.scene
   * @param {import('../core.js').World} o.world
   * @param {string} o.name
   * @param {number} o.color team colour
   * @param {{top: string, front: string}} o.loadout
   */
  constructor({ scene, world, terrain = null, name, color, loadout, isBot = false }) {
    this.type = 'hexapod';
    this.cfg = HEXAPOD;
    this.name = name;
    this.isBot = isBot;
    this.world = world;
    this.terrain = terrain;
    this.scene = scene;
    this.color = color;
    this.maxHp = 100;
    this.hp = 100;
    this.alive = true;
    this.deathT = 0;
    this.yaw = 0;
    this.pos = new THREE.Vector3();
    this.vel = { x: 0, z: 0, w: 0 };
    this.aimPitch = 0.12; // radians, shared by the turret attachments
    this.hitFlash = 0;
    this.lastHitBy = null;

    const c = this.cfg;
    this.gait = new HexapodGait(new Float32Array(gaitMounts(c)), c.leg.coxa, c.leg.femur, c.leg.tibia, c.leg.footReach, c.standHeight);
    const J = c.jointLimits;
    const r = (d) => (d * Math.PI) / 180;
    this.gait.set_joint_limits(r(J.coxa[0]), r(J.coxa[1]), r(J.femur[0]), r(J.femur[1]), r(J.tibia[0]), r(J.tibia[1]), r(c.tibiaFoldWhenFemurDown ?? J.tibia[0]));
    this.gait.set_reach_window(c.leg.footReachMin, c.leg.footReachMax, c.stepHz.sprint);
    this.gait.set_step(c.control.stepHeight, c.stepHz.walk);
    // the onboard controller (balance, terrain, gait choice, battery), as in the firmware
    this.ctl = new HexapodController(c);
    this.bodyId = world.add_body(c.hitRadius);

    this.root = new THREE.Group();
    this.root.name = name;
    scene.add(this.root);

    this.bodyMat = mat(color, { roughness: 0.45, emissive: 0x000000 });
    this.body = new THREE.Group();
    this.root.add(this.body);
    this.#buildBody();
    this.#buildLegs();

    this.topSlot = new THREE.Group();
    this.topSlot.position.fromArray(c.topMount);
    this.topSlot.position.y += c.body.plateGap / 2 + c.body.thickness;
    this.body.add(this.topSlot);
    this.frontSlot = new THREE.Group();
    this.frontSlot.position.fromArray(c.frontMount);
    this.body.add(this.frontSlot);

    this.attachments = { top: null, front: null };
    this.setLoadout(loadout);
  }

  #buildBody() {
    // Real CAD chassis from Fusion when available
    const cad = hexapodMesh('body', this.bodyMat);
    if (cad) {
      this.body.add(cad);
      this.body.add(hexapodMesh('qr_receiver', this.bodyMat));
      return;
    }
    const b = this.cfg.body;
    // Hexagon-ish plates (an extruded shape matches what we'd laser-cut / 3D print)
    const shape = new THREE.Shape();
    const L = b.length / 2;
    const W = b.width / 2;
    const pts = [
      [-W * 0.75, L],
      [W * 0.75, L],
      [W * 1.15, 0.03],
      [W * 1.15, -0.03],
      [W * 0.75, -L],
      [-W * 0.75, -L],
      [-W * 1.15, -0.03],
      [-W * 1.15, 0.03],
    ];
    shape.moveTo(pts[0][0], pts[0][1]);
    pts.slice(1).forEach(([x, z]) => shape.lineTo(x, z));
    shape.closePath();
    const geo = new THREE.ExtrudeGeometry(shape, { depth: b.thickness, bevelEnabled: true, bevelSize: 0.003, bevelThickness: 0.002, bevelSegments: 2 });
    geo.rotateX(Math.PI / 2); // extrude along -Y, shape lies in XZ
    for (const y of [b.plateGap / 2 + b.thickness, -b.plateGap / 2]) {
      const plate = new THREE.Mesh(geo, this.bodyMat);
      plate.position.y = y;
      plate.castShadow = plate.receiveShadow = true;
      this.body.add(plate);
    }
    // Lobes round each coxa axis (the real plates grow these so the servo tabs have material)
    if (b.lobeRadius) {
      const lobe = new THREE.CylinderGeometry(b.lobeRadius, b.lobeRadius, b.thickness, 32);
      for (const [mx, mz] of this.cfg.mounts) {
        for (const y of [b.plateGap / 2 + b.thickness / 2, -b.plateGap / 2 - b.thickness / 2]) {
          const m = new THREE.Mesh(lobe, this.bodyMat);
          m.position.set(mx, y, mz);
          m.castShadow = m.receiveShadow = true;
          this.body.add(m);
        }
      }
    }
    // Stand-offs between plates
    const post = new THREE.CylinderGeometry(0.004, 0.004, b.plateGap, 8);
    for (const [x, z] of [[-0.035, 0.1], [0.035, 0.1], [-0.035, -0.1], [0.035, -0.1]]) {
      const m = new THREE.Mesh(post, SHARED.aluminium);
      m.position.set(x, 0, z);
      this.body.add(m);
    }
    // Battery + controller board between the plates
    const batt = new THREE.Mesh(new THREE.BoxGeometry(0.045, 0.03, 0.12), SHARED.battery);
    batt.position.set(0, -0.004, -0.02);
    this.body.add(batt);
    const pcb = new THREE.Mesh(new THREE.BoxGeometry(0.06, 0.004, 0.08), SHARED.pcb);
    pcb.position.set(0, 0.017, 0.03);
    this.body.add(pcb);
    // Little "eyes" so you can see which way is forward
    const eyeMat = new THREE.MeshStandardMaterial({ color: 0xffffff, emissive: 0x66ccff, emissiveIntensity: 1.2 });
    for (const x of [-0.022, 0.022]) {
      const eye = new THREE.Mesh(new THREE.SphereGeometry(0.008, 12, 8), eyeMat);
      eye.position.set(x, 0.01, b.length / 2 + 0.002);
      this.body.add(eye);
    }
  }

  #buildLegs() {
    const c = this.cfg;
    const mounts = gaitMounts(c);
    this.legs = [];
    for (let i = 0; i < 6; i++) {
      const [mx, mz, yaw] = [mounts[i * 3], mounts[i * 3 + 1], mounts[i * 3 + 2]];
      const mount = new THREE.Group();
      mount.position.set(mx, 0, mz);
      // gait yaw is measured from +X toward +Z; three.js Y rotation goes the other way
      mount.rotation.y = -yaw;
      this.body.add(mount);

      const coxaJ = new THREE.Group();
      mount.add(coxaJ);
      const femurJ = new THREE.Group();
      femurJ.position.x = c.leg.coxa;
      coxaJ.add(femurJ);
      const tibiaJ = new THREE.Group();
      tibiaJ.position.x = c.leg.femur;
      femurJ.add(tibiaJ);

      if (hasHexapodMeshes()) {
        // Real CAD links (the coxa servo itself is part of the body mesh)
        coxaJ.add(hexapodMesh('coxa', this.bodyMat));
        femurJ.add(hexapodMesh('femur', this.bodyMat));
        tibiaJ.add(hexapodMesh('tibia', this.bodyMat));
      } else {
        const s0 = servoBox();
        s0.rotation.x = Math.PI / 2;
        coxaJ.add(s0);
        coxaJ.add(linkBox(c.leg.coxa, 0.022, 0.008, SHARED.aluminium));
        femurJ.add(servoBox());
        femurJ.add(linkBox(c.leg.femur, 0.024, 0.012, this.bodyMat));
        tibiaJ.add(servoBox());
        const tibGeo = new THREE.CylinderGeometry(0.006, 0.011, c.leg.tibia, 10);
        tibGeo.rotateZ(Math.PI / 2);
        tibGeo.translate(c.leg.tibia / 2, 0, 0);
        const tib = new THREE.Mesh(tibGeo, this.bodyMat);
        tib.castShadow = true;
        tibiaJ.add(tib);
        const foot = new THREE.Mesh(new THREE.SphereGeometry(0.009, 12, 8), SHARED.rubber);
        foot.position.x = c.leg.tibia;
        tibiaJ.add(foot);
      }

      this.legs.push({ coxaJ, femurJ, tibiaJ });
    }
    this.jointAngles = new Float32Array(18);
  }

  setLoadout(loadout) {
    for (const slot of ['top', 'front']) {
      const old = this.attachments[slot];
      if (old) {
        old.dispose?.();
        old.group.removeFromParent();
      }
      const id = loadout?.[slot] ?? 'none';
      const a = createAttachment(id, this);
      this.attachments[slot] = a;
      if (a) (slot === 'top' ? this.topSlot : this.frontSlot).add(a.group);
    }
    // Quick-release adapter plate under whatever is on the top deck (CAD only).
    // It is modelled in the body frame, so undo the slot offset.
    this.adapterPlate?.removeFromParent();
    this.adapterPlate = null;
    if (this.attachments.top) {
      const plate = hexapodMesh('qr_adapter_example', null, (m) => m.startsWith('QR_Adapter'));
      if (plate) {
        plate.position.copy(this.topSlot.position).negate();
        this.topSlot.add(plate);
        this.adapterPlate = plate;
      }
    }
    this.loadout = { top: loadout?.top ?? 'none', front: loadout?.front ?? 'none' };
  }

  spawn(x, z, yaw) {
    this.pos.set(x, 0, z);
    this.yaw = yaw;
    this.vel = { x: 0, z: 0, w: 0 };
    this.ctl.reset();
    this.ctl.att.h = this.ground(x, z) + this.cfg.standHeight;
    this.hp = this.maxHp;
    this.alive = true;
    this.deathT = 0;
    this.body.rotation.set(0, 0, 0);
    this.aimPitch = 0.12;
    Object.values(this.attachments).forEach((a) => a?.reset?.());
    this.#syncTransform();
  }

  damage(amount, from) {
    if (!this.alive) return;
    this.hp = Math.max(0, this.hp - amount);
    this.hitFlash = 1;
    this.lastHitBy = from;
    if (this.hp <= 0) {
      this.alive = false;
      this.deathT = 0;
    }
  }

  /** Muzzle-independent helpers used by bots and the camera. */
  forward(out = new THREE.Vector3()) {
    return out.set(Math.sin(this.yaw), 0, Math.cos(this.yaw));
  }

  /**
   * @param {number} dt
   * @param {object} cmd  {moveX, moveZ, turn, aim, sprint, fire, firePressed, alt, altPressed}
   * @param {object} game shared context for attachments (world, effects, robots, audio)
   */
  update(dt, cmd, game) {
    const c = this.cfg;
    if (!this.alive) {
      this.deathT += dt;
      cmd = { moveX: 0, moveZ: 0, turn: 0, aim: 0 };
    }
    const ctl = this.ctl;
    if (this.alive) ctl.applySwitches(cmd);
    const sprint = !!cmd.sprint && !ctl.terrainMode;
    const speed = sprint ? c.speed.sprint : c.speed.walk;
    let tx = (cmd.moveX || 0) * speed;
    let tz = (cmd.moveZ || 0) * speed;
    let tw = (cmd.turn || 0) * c.speed.turn;
    if (ctl.mode === 'pose' || ctl.fault) tx = tz = tw = 0;

    // Legs: the onboard controller runs the Rust gait (pose, balance, terrain)
    if (this.alive) {
      ctl.step(dt, cmd, this.gait, this.terrain, this.body, { tx, tz, tw });
      this.jointAngles.set(this.gait.angles());
      // the body moves exactly as its stance feet push it (after the command
      // filter and the stance-window rule), so planted feet never slide
      const v = this.gait.velocity();
      this.vel.x = v[0];
      this.vel.z = v[1];
      this.vel.w = v[2];
    } else {
      // Collapse: legs relax upward, body sinks
      const k = 1 - Math.exp(-dt * 8);
      for (let i = 0; i < 6; i++) {
        this.jointAngles[i * 3 + 1] += (0.6 - this.jointAngles[i * 3 + 1]) * k * 0.5;
        this.jointAngles[i * 3 + 2] += (-0.4 - this.jointAngles[i * 3 + 2]) * k * 0.5;
      }
    }
    if (!this.alive) {
      const k = 1 - Math.exp(-dt * 8);
      this.vel.x -= this.vel.x * k;
      this.vel.z -= this.vel.z * k;
      this.vel.w -= this.vel.w * k;
    }

    this.yaw += this.vel.w * dt;
    const s = Math.sin(this.yaw);
    const co = Math.cos(this.yaw);
    let nx = this.pos.x + (this.vel.x * co + this.vel.z * s) * dt;
    let nz = this.pos.z + (-this.vel.x * s + this.vel.z * co) * dt;
    const fixed = this.world.resolve_body(this.bodyId, nx, nz, c.footprintRadius);
    this.pos.x = fixed[0];
    this.pos.z = fixed[1];

    for (let i = 0; i < 6; i++) {
      const L = this.legs[i];
      L.coxaJ.rotation.y = -this.jointAngles[i * 3];
      L.femurJ.rotation.z = this.jointAngles[i * 3 + 1];
      L.tibiaJ.rotation.z = this.jointAngles[i * 3 + 2];
    }

    // Aim pitch (turret attachments read this)
    this.aimPitch = THREE.MathUtils.clamp(this.aimPitch + (cmd.aim || 0) * dt * 1.2, -0.25, 0.9);

    for (const a of Object.values(this.attachments)) a?.update(dt, cmd, game);

    // Hit flash
    this.hitFlash = Math.max(0, this.hitFlash - dt * 4);
    this.bodyMat.emissive.setRGB(this.hitFlash * 0.9, this.hitFlash * 0.25, this.hitFlash * 0.1);

    this.#syncTransform();
  }

  /** Ground height under a point (0 on flat ground). */
  ground(x, z) {
    return this.terrain ? this.terrain.height(x, z) : 0;
  }

  #syncTransform() {
    const c = this.cfg;
    this.root.position.set(this.pos.x, 0, this.pos.z);
    this.root.rotation.y = this.yaw;
    // The body sits where its supporting feet put it (hexapod.control.js); the
    // root stays at ground level 0 and the body carries the height and tilt.
    const a = this.ctl.att;
    let y = a.h;
    this.body.rotation.order = 'YXZ';
    if (!this.alive) {
      const t = Math.min(1, this.deathT * 2);
      y = THREE.MathUtils.lerp(a.h, this.ground(this.pos.x, this.pos.z) + c.body.plateGap / 2 + 0.012, t);
      this.body.rotation.set(-a.pitch - 0.06 * t, a.yaw ?? 0, a.roll + 0.12 * t);
    } else {
      this.body.rotation.set(-a.pitch, a.yaw ?? 0, a.roll);
    }
    this.body.position.y = y;
    this.world.set_body(this.bodyId, this.pos.x, y + 0.02, this.pos.z, this.alive);
  }

  /** World-space centre of the hit sphere. */
  center(out = tmpV) {
    return out.set(this.pos.x, this.body.position.y + 0.02, this.pos.z);
  }

  /** Servo angles (0-180) for the real robot, leg servos first then attachments. */
  servoFrame() {
    const out = [];
    for (let i = 0; i < 6; i++) {
      out.push(...legToServo(i, this.jointAngles[i * 3], this.jointAngles[i * 3 + 1], this.jointAngles[i * 3 + 2]));
    }
    for (const a of Object.values(this.attachments)) if (a?.servos) out.push(...a.servos());
    return out;
  }

  dispose() {
    this.root.removeFromParent();
    this.gait.free();
    this.ctl.dispose();
  }
}
