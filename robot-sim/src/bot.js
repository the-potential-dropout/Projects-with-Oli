// Bot brain: drives a robot with exactly the same commands a player sends, so
// anything the player can do the bot can do (and vice versa).
import { World } from './core.js';

const LEVELS = {
  easy: { aimNoise: 0.16, reaction: 0.9, turnGain: 1.6, strafe: 0.0, lead: false, fireTol: 1.6 },
  normal: { aimNoise: 0.07, reaction: 0.45, turnGain: 2.6, strafe: 0.45, lead: false, fireTol: 1.0 },
  hard: { aimNoise: 0.025, reaction: 0.15, turnGain: 3.5, strafe: 0.8, lead: true, fireTol: 0.8 },
};

const PREFERRED_RANGE = { laser: 2.2, potato: 3.0, arm: 0.0, none: 0.5 };

const wrap = (a) => Math.atan2(Math.sin(a), Math.cos(a));
const clamp = (v, lo = -1, hi = 1) => Math.max(lo, Math.min(hi, v));

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
    this.stuck = 0;
    this.wanderYaw = 0;
  }

  setLevel(level) {
    this.levelName = level;
    this.L = LEVELS[level] ?? LEVELS.normal;
  }

  think(dt, game) {
    const r = this.robot;
    const L = this.L;
    const cmd = { moveX: 0, moveZ: 0, turn: 0, aim: 0, sprint: false, fire: false, alt: false };
    if (!r.alive) return cmd;

    const enemy = game.targets().filter((t) => t !== r && t.alive && t.team !== r.team).sort((a, b) => a.pos.distanceTo(r.pos) - b.pos.distanceTo(r.pos))[0];
    if (!enemy) return this.#wander(dt, game, cmd);

    const weapon = r.attachments.top?.id ?? 'none';
    const pref = PREFERRED_RANGE[weapon];
    const me = r.center();
    const mx = me.x;
    const my = me.y;
    const mz = me.z;
    let ex = enemy.pos.x;
    let ez = enemy.pos.z;
    const ey = enemy.center().y;
    let dist = Math.hypot(ex - mx, ez - mz);

    // Potato: pick a launch speed for this range, and lead a moving target
    let pitchWanted = Math.atan2(ey - my, dist);
    let charge = 0.6;
    if (weapon === 'potato') {
      charge = clamp(0.25 + dist / 6, 0.25, 1);
      const v = 4.5 + 6.5 * charge;
      if (L.lead && enemy.vel) {
        const flight = dist / v;
        const s = Math.sin(enemy.yaw);
        const c = Math.cos(enemy.yaw);
        ex += (enemy.vel.x * c + enemy.vel.z * s) * flight;
        ez += (-enemy.vel.x * s + enemy.vel.z * c) * flight;
        dist = Math.hypot(ex - mx, ez - mz);
      }
      // Muzzle sits ~5 cm above the hit centre; small drag means aim a touch high
      const p = World.ballistic_pitch(dist * 1.04, ey - (my + 0.05), v);
      pitchWanted = Number.isNaN(p) ? 0.7 : p;
    }

    // Wobble the aim a bit so easy bots miss sometimes
    this.noiseTimer -= dt;
    if (this.noiseTimer <= 0) {
      this.noise = (Math.random() * 2 - 1) * L.aimNoise;
      this.noiseTimer = 0.5 + Math.random() * 0.6;
    }
    const wantYaw = Math.atan2(ex - mx, ez - mz) + this.noise;
    const err = wrap(wantYaw - r.yaw);
    cmd.turn = clamp(err * L.turnGain);
    cmd.aim = clamp((pitchWanted + this.noise * 0.5 - r.aimPitch) * 5);

    // Range keeping
    const lowHp = r.hp < 30 && this.levelName === 'hard';
    const target = lowHp ? pref + 1.2 : pref;
    if (dist > target + 0.35) cmd.moveZ = 1;
    else if (dist < target - 0.35) cmd.moveZ = -0.8;
    cmd.sprint = dist > target + 2.5 || (lowHp && dist < target);
    if (Math.abs(err) > 1.2) cmd.moveZ *= 0.3; // turn first, then walk

    // Circle-strafe to dodge
    this.strafeTimer -= dt;
    // (keep the same direction while flanking around cover)
    if (this.strafeTimer <= 0 && !(this.blind > 1)) {
      this.strafeDir = Math.random() < 0.5 ? -1 : 1;
      this.strafeTimer = 1.5 + Math.random() * 2.5;
    }
    if (dist < pref + 1.5) cmd.moveX = this.strafeDir * L.strafe;

    // Don't walk into walls: probe ahead and to the strafe side
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

    // Fire when lined up and we can actually see the target
    const los = game.world.raycast(mx, my, mz, ex - mx, ey - my, ez - mz, dist + 0.5, r.bodyId);
    const canSee = los[1] === enemy.bodyId;
    // Target hiding behind cover: walk around it instead of waiting
    this.blind = canSee ? 0 : (this.blind ?? 0) + dt;
    if (this.blind > 1.0 && weapon !== 'potato') {
      cmd.moveZ = dist > 1.2 ? 0.6 : 0;
      cmd.moveX = this.strafeDir;
      cmd.sprint = true;
    }
    const aligned = Math.abs(err) < 0.09 * L.fireTol + (weapon === 'arm' ? 0.4 : 0) && Math.abs(pitchWanted - r.aimPitch) < 0.06 * L.fireTol + (weapon === 'arm' ? 1 : 0);
    if (canSee && aligned) this.react += dt;
    else this.react = Math.max(0, this.react - dt * 2);

    if (weapon === 'potato') {
      if (this.charging > 0) {
        this.charging -= dt;
        cmd.fire = true;
        cmd.fireCharge = charge;
      } else if (Math.abs(err) < 0.12 * L.fireTol && dist < 7) {
        this.charging = 1.3;
        cmd.fire = true;
        cmd.fireCharge = charge;
      }
    } else if (weapon === 'arm') {
      cmd.fire = this.react > L.reaction * 0.5 && dist < 0.55;
    } else if (weapon === 'laser') {
      cmd.fire = this.react > L.reaction && dist < 8.5;
    }
    return cmd;
  }

  #wander(dt, game, cmd) {
    const r = this.robot;
    this.strafeTimer -= dt;
    if (this.strafeTimer <= 0) {
      this.wanderYaw = Math.random() * Math.PI * 2;
      this.strafeTimer = 3;
    }
    cmd.turn = clamp(wrap(this.wanderYaw - r.yaw) * 2);
    cmd.moveZ = 0.5;
    return cmd;
  }
}
