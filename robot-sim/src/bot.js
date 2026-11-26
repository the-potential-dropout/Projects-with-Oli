// Autonomy: the robot's behaviour tree, as in docs/AUTONOMY.md (battle.xml on
// the brain module): find the opponent -> approach -> strafe-circle -> aim by
// body yaw and pitch -> fire, and retreat when low. The bot opponent runs it,
// and so does your own robot in AUTO mode (radio switch SC, or M).
//
// It drives with exactly the same commands a player sends, including the
// radio switches: it turns on terrain mode and the ripple gait when its
// look-ahead sensor (the VL53L1X on the front bracket) sees rough ground.
import { World } from './core.js';

const LEVELS = {
  easy: { aimNoise: 0.16, reaction: 0.9, turnGain: 1.6, strafe: 0.0, lead: false, fireTol: 1.6 },
  normal: { aimNoise: 0.07, reaction: 0.45, turnGain: 2.6, strafe: 0.45, lead: false, fireTol: 1.0 },
  hard: { aimNoise: 0.025, reaction: 0.15, turnGain: 3.5, strafe: 0.8, lead: true, fireTol: 0.8 },
};

// engage range per weapon (m); the BT's default approach distance is 1.5 m
const PREFERRED_RANGE = { laser: 2.2, potato: 3.0, arm: 0.0, none: 1.5 };
const RETREAT_HP = 0.25; // AUTONOMY.md: retreat when the battery (here: health) is below 25 %
const ROUGH = 0.012; // m of ground change ahead that counts as rough
const MEMORY = 4; // s the opponent's last position is remembered after losing sight

const wrap = (a) => Math.atan2(Math.sin(a), Math.cos(a));
const clamp = (v, lo = -1, hi = 1) => Math.max(lo, Math.min(hi, v));

const OK = 'success';
const FAIL = 'failure';
const RUN = 'running';

export class BotBrain {
  constructor(robot, level = 'normal') {
    this.robot = robot;
    this.setLevel(level);
    this.strafeDir = 1;
    this.strafeTimer = 2;
    this.react = 0;
    this.noise = 0;
    this.noiseTimer = 0;
    this.charging = 0;
    this.blind = 0;
    this.wanderYaw = 0;
    this.lastSeen = null; // {x, z, t}
    this.t = 0;
    this.roughT = 0;
    this.node = 'patrol'; // active leaf, for the HUD
  }

  setLevel(level) {
    this.levelName = level;
    this.L = LEVELS[level] ?? LEVELS.normal;
  }

  think(dt, game) {
    const r = this.robot;
    this.t += dt;
    const cmd = { moveX: 0, moveZ: 0, turn: 0, aim: 0, sprint: false, fire: false, alt: false };
    if (!r.alive) return cmd;
    const bb = this.#blackboard(game);
    this.#terrainSense(dt, game, cmd);

    // root selector: retreat > battle > patrol
    if (this.#retreat(dt, game, bb, cmd) !== FAIL) return cmd;
    if (this.#battle(dt, game, bb, cmd) !== FAIL) return cmd;
    this.#patrol(dt, game, cmd);
    return cmd;
  }

  /* ---------------- blackboard: what the robot knows ---------------- */

  #blackboard(game) {
    const r = this.robot;
    const me = r.center();
    const enemy = game
      .targets()
      .filter((t) => t !== r && t.alive && t.team !== r.team)
      .sort((a, b) => a.pos.distanceTo(r.pos) - b.pos.distanceTo(r.pos))[0];
    const bb = { me, enemy: null, weapon: r.attachments.top?.id ?? 'none' };
    if (!enemy) return bb;
    const ey = enemy.center().y;
    const dist = Math.hypot(enemy.pos.x - me.x, enemy.pos.z - me.z);
    const los = game.world.raycast(me.x, me.y, me.z, enemy.pos.x - me.x, ey - me.y, enemy.pos.z - me.z, dist + 0.5, r.bodyId);
    const canSee = los[1] === enemy.bodyId;
    // the camera / LiDAR see it, or the sonar/LiDAR has it as a contact
    const sensed = canSee || (r.sensor?.contacts ?? []).some((c) => c.bodyId === enemy.bodyId);
    if (sensed) this.lastSeen = { x: enemy.pos.x, z: enemy.pos.z, t: this.t };
    Object.assign(bb, { enemy, ey, dist, canSee, sensed });
    return bb;
  }

  /** Look-ahead sensor: switch to terrain mode + ripple on rough ground, back to tripod on flat. */
  #terrainSense(dt, game, cmd) {
    const r = this.robot;
    const T = game.terrain;
    if (!T) return;
    const f = r.forward();
    const here = T.height(r.pos.x, r.pos.z);
    let worst = 0;
    for (const d of [0.15, 0.3]) worst = Math.max(worst, Math.abs(T.height(r.pos.x + f.x * d, r.pos.z + f.z * d) - here));
    this.roughT = worst > ROUGH ? 2.5 : Math.max(0, this.roughT - dt);
    const want = this.roughT > 0;
    if (want !== r.ctl.terrainMode) {
      cmd.terrain = want;
      cmd.gait = want ? 1 : 0;
    }
  }

  /* ---------------- retreat ---------------- */

  #retreat(dt, game, bb, cmd) {
    const r = this.robot;
    if (!bb.enemy || r.hp / r.maxHp >= RETREAT_HP || this.levelName === 'easy') return FAIL;
    this.node = 'retreat';
    this.#face(bb, cmd);
    const pref = PREFERRED_RANGE[bb.weapon] + 1.2;
    cmd.moveZ = bb.dist < pref ? -1 : 0;
    cmd.sprint = bb.dist < pref * 0.6;
    this.#avoidWalls(game, bb, cmd);
    this.#fire(dt, bb, cmd); // keep shooting while backing off
    return RUN;
  }

  /* ---------------- battle: find -> approach -> strafe -> aim -> fire ---------------- */

  #battle(dt, game, bb, cmd) {
    if (!bb.enemy) return FAIL;
    // find: forget an opponent not seen for a while (go and look where it was)
    if (!bb.sensed && this.lastSeen && this.t - this.lastSeen.t > MEMORY && bb.weapon !== 'potato') {
      this.node = 'search';
      const yaw = Math.atan2(this.lastSeen.x - this.robot.pos.x, this.lastSeen.z - this.robot.pos.z);
      cmd.turn = clamp(wrap(yaw - this.robot.yaw) * 2);
      cmd.moveZ = Math.hypot(this.lastSeen.x - this.robot.pos.x, this.lastSeen.z - this.robot.pos.z) > 0.4 ? 0.7 : 0;
      if (cmd.moveZ === 0) this.lastSeen.t = this.t; // got there: look round again
      this.#avoidWalls(game, bb, cmd);
      return RUN;
    }
    const pref = PREFERRED_RANGE[bb.weapon];
    const err = this.#face(bb, cmd);

    // approach / keep range
    if (bb.dist > pref + 0.35) {
      this.node = 'approach';
      cmd.moveZ = 1;
    } else if (bb.dist < pref - 0.35) {
      this.node = 'back off';
      cmd.moveZ = -0.8;
    } else this.node = 'engage';
    cmd.sprint = bb.dist > pref + 2.5;
    if (Math.abs(err) > 1.2) cmd.moveZ *= 0.3; // turn first, then walk

    // strafe-circle
    this.strafeTimer -= dt;
    if (this.strafeTimer <= 0 && !(this.blind > 1)) {
      this.strafeDir = Math.random() < 0.5 ? -1 : 1;
      this.strafeTimer = 1.5 + Math.random() * 2.5;
    }
    if (bb.dist < pref + 1.5) cmd.moveX = this.strafeDir * this.L.strafe;

    // opponent behind cover: flank round it instead of waiting
    this.blind = bb.canSee ? 0 : this.blind + dt;
    if (this.blind > 1.0 && bb.weapon !== 'potato') {
      this.node = 'flank';
      cmd.moveZ = bb.dist > 1.2 ? 0.6 : 0;
      cmd.moveX = this.strafeDir;
      cmd.sprint = true;
    }
    this.#avoidWalls(game, bb, cmd);
    this.#fire(dt, bb, cmd);
    return RUN;
  }

  /** Aim by body yaw (turn) and turret pitch; returns the yaw error. */
  #face(bb, cmd) {
    const r = this.robot;
    const L = this.L;
    let ex = bb.enemy.pos.x;
    let ez = bb.enemy.pos.z;
    let dist = bb.dist;
    let pitchWanted = Math.atan2(bb.ey - bb.me.y, dist);
    bb.charge = 0.6;
    if (bb.weapon === 'potato') {
      bb.charge = clamp(0.25 + dist / 6, 0.25, 1);
      const v = 4.5 + 6.5 * bb.charge;
      const e = bb.enemy;
      if (L.lead && e.vel) {
        const flight = dist / v;
        const s = Math.sin(e.yaw);
        const c = Math.cos(e.yaw);
        ex += (e.vel.x * c + e.vel.z * s) * flight;
        ez += (-e.vel.x * s + e.vel.z * c) * flight;
        dist = Math.hypot(ex - bb.me.x, ez - bb.me.z);
      }
      const p = World.ballistic_pitch(dist * 1.04, bb.ey - (bb.me.y + 0.05), v);
      pitchWanted = Number.isNaN(p) ? 0.7 : p;
    }
    this.noiseTimer -= 1 / 60;
    if (this.noiseTimer <= 0) {
      this.noise = (Math.random() * 2 - 1) * L.aimNoise;
      this.noiseTimer = 0.5 + Math.random() * 0.6;
    }
    const err = wrap(Math.atan2(ex - bb.me.x, ez - bb.me.z) + this.noise - r.yaw);
    cmd.turn = clamp(err * L.turnGain);
    cmd.aim = clamp((pitchWanted + this.noise * 0.5 - r.aimPitch) * 5);
    bb.yawErr = err;
    bb.pitchErr = pitchWanted - r.aimPitch;
    return err;
  }

  #fire(dt, bb, cmd) {
    const L = this.L;
    const arm = bb.weapon === 'arm';
    const aligned = Math.abs(bb.yawErr) < 0.09 * L.fireTol + (arm ? 0.4 : 0) && Math.abs(bb.pitchErr) < 0.06 * L.fireTol + (arm ? 1 : 0);
    if (bb.canSee && aligned) this.react += dt;
    else this.react = Math.max(0, this.react - dt * 2);
    if (bb.weapon === 'potato') {
      if (this.charging > 0) {
        this.charging -= dt;
        cmd.fire = true;
        cmd.fireCharge = bb.charge;
      } else if (Math.abs(bb.yawErr) < 0.12 * L.fireTol && bb.dist < 7) {
        this.charging = 1.3;
        cmd.fire = true;
        cmd.fireCharge = bb.charge;
      }
    } else if (arm) {
      cmd.fire = this.react > L.reaction * 0.5 && bb.dist < 0.55;
    } else if (bb.weapon === 'laser') {
      cmd.fire = this.react > L.reaction && bb.dist < 8.5;
    }
  }

  /** Don't walk into walls: probe ahead and to the strafe side. */
  #avoidWalls(game, bb, cmd) {
    const r = this.robot;
    const { x: mx, y: my, z: mz } = bb.me;
    const fwd = game.world.raycast(mx, my, mz, Math.sin(r.yaw), 0, Math.cos(r.yaw), 0.5, r.bodyId);
    if (fwd[1] === -1 && fwd[0] < 0.45 && cmd.moveZ > 0) {
      cmd.moveZ = 0;
      cmd.moveX = this.strafeDir;
    }
    const side = r.yaw + (this.strafeDir > 0 ? Math.PI / 2 : -Math.PI / 2);
    const sd = game.world.raycast(mx, my, mz, Math.sin(side), 0, Math.cos(side), 0.45, r.bodyId);
    if (sd[1] === -1 && sd[0] < 0.4) {
      this.strafeDir *= -1;
      this.strafeTimer = 2;
    }
  }

  /* ---------------- patrol ---------------- */

  #patrol(dt, game, cmd) {
    const r = this.robot;
    this.node = 'patrol';
    this.strafeTimer -= dt;
    if (this.strafeTimer <= 0) {
      this.wanderYaw = Math.random() * Math.PI * 2;
      this.strafeTimer = 3;
    }
    cmd.turn = clamp(wrap(this.wanderYaw - r.yaw) * 2);
    cmd.moveZ = 0.5;
    this.#avoidWalls(game, { me: r.center() }, cmd);
    return RUN;
  }
}
