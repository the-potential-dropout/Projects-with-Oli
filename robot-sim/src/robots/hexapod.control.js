// The hexapod's onboard controller, as the simulator runs it. It mirrors
// firmware/hexapod_controller (docs/CONTROL.md): the same gait engine (in Rust),
// body pose limits, the balance PID on a simulated IMU, foot switches, terrain
// mode, the stability-margin rule, the tip-over fault and the battery monitor.
//
// The body is not animated: every frame the Rust terrain module finds the
// height and tilt that put the supporting feet on the ground (rigid robot,
// legs at the commanded angles), followed through a 60 ms servo lag. So on rough
// ground without terrain mode the robot rocks, and with it the feet find the
// ground and the body rides level, the same as the real one.
import * as THREE from 'three';
import { Balance } from '../core.js';

const tmpVec = new THREE.Vector3();

const DEG = Math.PI / 180;
export const GAITS = ['Tripod', 'Ripple', 'Wave'];
export const MODES = ['manual', 'pose', 'auto'];
const SERVO_LAG = 0.06; // s, first-order response of the STS servos under load
const IMU_NOISE = 0.08 * DEG; // BNO085 game rotation vector, at rest

// 3S 4000 mAh LiPo: open-circuit volts vs state of charge, and pack resistance
const OCV = [
  [0, 9.6],
  [0.05, 10.5],
  [0.15, 10.95],
  [0.3, 11.25],
  [0.5, 11.5],
  [0.7, 11.8],
  [0.9, 12.3],
  [1, 12.6],
];
const PACK_R = 0.045;

function ocv(soc) {
  for (let i = 1; i < OCV.length; i++) {
    if (soc <= OCV[i][0]) {
      const [s0, v0] = OCV[i - 1];
      const [s1, v1] = OCV[i];
      return v0 + ((soc - s0) / (s1 - s0)) * (v1 - v0);
    }
  }
  return OCV[OCV.length - 1][1];
}

const gauss = () => (Math.random() + Math.random() + Math.random() - 1.5) * 1.15;
const approach = (v, target, rate, dt) => v + Math.max(-rate * dt, Math.min(rate * dt, target - v));

export class HexapodController {
  constructor(cfg) {
    this.cfg = cfg;
    const b = cfg.control.balance;
    const p = cfg.control.pose;
    this.balance = new Balance(b.kp, b.ki, b.kd, b.iMaxDeg, b.slewDegPerS, b.dLowpassHz, Math.max(p.rollMax, p.pitchMax));
    this.reset();
  }

  reset() {
    this.mode = 'manual'; // radio SC: manual walk / pose / auto
    this.gaitReq = 0; // radio SB
    this.terrainMode = false; // radio SD
    this.balanceOn = true;
    this.height = this.cfg.standHeight; // requested body height (m)
    this.req = { roll: 0, pitch: 0, yaw: 0 }; // requested pose (rad)
    this.cmdPose = { roll: 0, pitch: 0, yaw: 0 };
    this.servoPose = { roll: 0, pitch: 0, yaw: 0 };
    this.att = { roll: 0, pitch: 0, h: this.cfg.standHeight }; // body world attitude (after the servo lag)
    this.fitAtt = { roll: 0, pitch: 0, h: this.cfg.standHeight };
    this.imu = { roll: 0, pitch: 0, rollRate: 0, pitchRate: 0 };
    this.contacts = 0b111111;
    this.stance = 0b111111;
    this.margin = 0.167;
    this.autoRipple = false;
    this.fault = '';
    this.faultT = 0;
    this.lowMarginT = 0;
    this.battery = { soc: 0.97, v: 12.5, a: 0.8, low: false };
    this.balance.reset();
    this.feetWorld = new Float32Array(12);
  }

  /** Radio-switch style inputs (edges) from the player or the behaviour tree. */
  applySwitches(cmd) {
    if (cmd.cycleMode) this.mode = MODES[(MODES.indexOf(this.mode) + 1) % MODES.length];
    if (cmd.setMode) this.mode = cmd.setMode;
    if (cmd.gait != null) this.gaitReq = cmd.gait;
    if (cmd.gaitStep) this.gaitReq = (this.gaitReq + cmd.gaitStep + 3) % 3;
    if (cmd.toggleTerrain) this.setTerrainMode(!this.terrainMode);
    if (cmd.terrain != null && cmd.terrain !== this.terrainMode) this.setTerrainMode(cmd.terrain);
    if (cmd.toggleBalance) {
      this.balanceOn = !this.balanceOn;
      this.balance.reset();
    }
  }

  setTerrainMode(on) {
    const c = this.cfg.control.terrain;
    this.terrainMode = on;
    // terrain mode raises the body for the higher steps (CONTROL.md §8)
    this.height = on ? c.bodyHeight : this.cfg.standHeight;
    if (on && this.gaitReq === 0) this.gaitReq = 1; // ripple recommended
  }

  /**
   * One control tick: pick the gait, set the pose, run the Rust gait, put the
   * body on the ground, update the IMU, balance, margin and battery.
   * @returns the walking command actually used (pose mode stops walking)
   */
  step(dt, cmd, gait, terrain, body, walk) {
    const c = this.cfg;
    const ctl = c.control;
    const P = ctl.pose;

    // ---- pose request (pose mode: the sticks tilt the body instead of walking)
    let { tx, tz, tw } = walk;
    let want = { roll: 0, pitch: 0, yaw: 0 };
    if (this.mode === 'pose') {
      want = { roll: -(cmd.moveX || 0) * P.rollMax * DEG, pitch: -(cmd.moveZ || 0) * P.pitchMax * DEG, yaw: (cmd.turn || 0) * P.yawMax * DEG };
      tx = tz = tw = 0;
    }
    if (cmd.poseRoll != null) want.roll = cmd.poseRoll;
    if (cmd.posePitch != null) want.pitch = cmd.posePitch;
    for (const k of ['roll', 'pitch', 'yaw']) this.req[k] = approach(this.req[k], want[k], 40 * DEG, dt);
    if (cmd.heightRate) this.height += cmd.heightRate * 0.05 * dt;
    if (cmd.height != null) this.height = cmd.height;
    this.height = Math.max(P.heightMin, Math.min(P.heightMax, this.height));

    // ---- faults: tipped, low battery (the servos relax, CONTROL.md §9)
    if (this.fault) {
      this.faultT += dt;
      tx = tz = tw = 0;
      if (this.fault === 'TIPPED' && this.faultT > 2 && Math.abs(this.imu.roll) < 10 * DEG && Math.abs(this.imu.pitch) < 10 * DEG) this.fault = '';
      if (this.fault === 'LOWBAT' && this.battery.v > 11.0) this.fault = '';
    }

    // ---- gait choice: margin below 25 mm -> ripple until it recovers
    let gaitKind = this.gaitReq;
    if (this.autoRipple && gaitKind === 0) gaitKind = 1;
    gait.set_gait(gaitKind);
    const sprint = !!cmd.sprint && !this.terrainMode;
    const stepH = this.terrainMode ? ctl.terrain.stepHeight : ctl.stepHeight;
    gait.set_step(stepH, sprint ? c.stepHz.sprint : c.stepHz.walk);
    gait.set_terrain(this.terrainMode);
    gait.set_contacts(this.contacts);
    gait.set_attitude(this.imu.roll, this.imu.pitch);
    gait.set_body_height(this.height);

    // ---- balance: PI-D on the IMU, target = the requested pose
    let roll = this.req.roll;
    let pitch = this.req.pitch;
    if (this.balanceOn && !this.fault) {
      this.balance.update(dt, this.req.roll, this.req.pitch, this.imu.roll, this.imu.pitch, this.imu.rollRate, this.imu.pitchRate);
      roll += this.balance.roll();
      pitch += this.balance.pitch();
    }
    // keep within the pose range, and don't tilt further while the margin is low
    roll = Math.max(-P.rollMax * DEG, Math.min(P.rollMax * DEG, roll));
    pitch = Math.max(-P.pitchMax * DEG, Math.min(P.pitchMax * DEG, pitch));
    if (this.margin < ctl.stabilityMarginMin) {
      roll = Math.abs(roll) < Math.abs(this.cmdPose.roll) ? roll : this.cmdPose.roll;
      pitch = Math.abs(pitch) < Math.abs(this.cmdPose.pitch) ? pitch : this.cmdPose.pitch;
    }
    this.cmdPose = { roll, pitch, yaw: this.req.yaw };
    // the servos follow the commanded pose with a small lag (the body itself is
    // rigid on its feet, so contact changes move it at once)
    const k = 1 - Math.exp(-dt / SERVO_LAG);
    const sp = this.servoPose;
    sp.roll += (roll - sp.roll) * k;
    sp.pitch += (pitch - sp.pitch) * k;
    sp.yaw += (this.req.yaw - sp.yaw) * k;
    gait.set_pose(sp.roll, sp.pitch, sp.yaw, 0, 0);
    gait.update(dt, tx, tz, tw);

    // ---- where the body rests on the ground
    const feetBody = gait.feet_body();
    this.stance = gait.stance_mask();
    // where the new foot targets are over the ground (last frame's body pose)
    body.updateWorldMatrix(true, false);
    this.trackFeet(body, feetBody);
    if (terrain) {
      const fit = terrain.fit_body(feetBody, this.feetWorld, this.stance, sp.yaw, this.fitAtt.roll, this.fitAtt.pitch);
      this.fitAtt = { h: fit[0], roll: fit[1], pitch: fit[2] };
      this.contacts = fit[3] | 0;
    } else {
      this.fitAtt = { h: this.height, roll: sp.roll, pitch: sp.pitch };
      this.contacts = this.stance;
    }
    const prev = { roll: this.att.roll, pitch: this.att.pitch };
    this.att.roll = this.fitAtt.roll;
    this.att.pitch = this.fitAtt.pitch;
    this.att.h = this.fitAtt.h;
    this.att.yaw = sp.yaw;

    // ---- simulated BNO085 (game rotation vector + gyro)
    if (dt > 0) {
      this.imu.rollRate = (this.att.roll - prev.roll) / dt + gauss() * 0.3 * DEG;
      this.imu.pitchRate = (this.att.pitch - prev.pitch) / dt + gauss() * 0.3 * DEG;
    }
    this.imu.roll = this.att.roll + gauss() * IMU_NOISE;
    this.imu.pitch = this.att.pitch + gauss() * IMU_NOISE;

    // ---- stability margin of the real stance feet round the CoM
    this.margin = this.#margin(body, feetBody);
    this.lowMarginT = this.margin < 0 ? this.lowMarginT + dt : 0; // outside the support polygon
    if (this.margin < ctl.stabilityMarginMin) {
      this.autoRipple = true;
    } else {
      if (this.margin > ctl.stabilityMarginMin + 0.02) this.autoRipple = false;
    }
    const tipped = Math.abs(this.imu.roll) > ctl.tipDeg * DEG || Math.abs(this.imu.pitch) > ctl.tipDeg * DEG || this.lowMarginT > 0.3;
    if (tipped && !this.fault) {
      this.fault = 'TIPPED';
      this.faultT = 0;
    }

    // ---- battery: 3S 4000 mAh, current from what the legs are doing
    const act = gait.activity();
    const a = 0.75 + 3.2 * act * (sprint ? 1.5 : 1) + (this.terrainMode ? 0.6 : 0) + Math.abs(roll + pitch) * 2;
    const bat = this.battery;
    bat.soc = Math.max(0, bat.soc - (a * dt) / 3600 / 4.0);
    bat.a = a;
    bat.v = ocv(bat.soc) - a * PACK_R;
    if (bat.v < 10.2) {
      bat.lowT = (bat.lowT ?? 0) + dt;
      if (bat.lowT > 3 && !this.fault) {
        this.fault = 'LOWBAT';
        this.faultT = 0;
      }
    } else bat.lowT = 0;

    return { tx, tz, tw };
  }

  /** World positions of the feet, for sampling the ground next frame. */
  trackFeet(body, feetBody) {
    for (let i = 0; i < 6; i++) {
      const p = body.localToWorld(tmpVec.set(feetBody[i * 3], feetBody[i * 3 + 1], feetBody[i * 3 + 2]));
      this.feetWorld[i * 2] = p.x;
      this.feetWorld[i * 2 + 1] = p.z;
    }
  }

  #margin(body, feetBody) {
    // feet that are actually touching, in world X/Z, round the body origin
    body.updateWorldMatrix(true, false);
    const pts = [];
    for (let i = 0; i < 6; i++) {
      if (!(this.contacts & (1 << i))) continue;
      const p = body.localToWorld(tmpVec.set(feetBody[i * 3], feetBody[i * 3 + 1], feetBody[i * 3 + 2]));
      pts.push([p.x, p.z]);
    }
    const o = body.localToWorld(tmpVec.set(0, 0, 0));
    return stabilityMargin(pts, o.x, o.z);
  }

  /** Telemetry line as the robot sends it (CONTROL.md §6): T,roll,pitch,yaw,feet,margin,mode,source,lq */
  telemetry(yaw, source = 2) {
    const mode = this.fault ? 0 : 2;
    return `T,${(this.imu.roll / DEG).toFixed(1)},${(this.imu.pitch / DEG).toFixed(1)},${(yaw / DEG).toFixed(1)},${this.contacts},${Math.round(this.margin * 1000)},${mode},${source},100`;
  }

  dispose() {
    this.balance.free();
  }
}


/** Signed distance (m) of (cx, cz) inside the convex hull of `pts`. Same as the firmware's. */
export function stabilityMargin(pts, cx, cz) {
  const n = pts.length;
  if (n < 3) return -1;
  let mx = 0;
  let mz = 0;
  for (const p of pts) {
    mx += p[0];
    mz += p[1];
  }
  mx /= n;
  mz /= n;
  pts.sort((a, b) => Math.atan2(a[1] - mz, a[0] - mx) - Math.atan2(b[1] - mz, b[0] - mx));
  let m = Infinity;
  for (let k = 0; k < n; k++) {
    const a = pts[k];
    const b = pts[(k + 1) % n];
    const ex = b[0] - a[0];
    const ez = b[1] - a[1];
    const L = Math.hypot(ex, ez);
    if (L < 1e-6) continue;
    m = Math.min(m, (ex * (cz - a[1]) - ez * (cx - a[0])) / L);
  }
  return m;
}
