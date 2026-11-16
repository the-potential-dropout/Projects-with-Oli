// Game: owns the renderer, the Rust physics world, the robots, the camera and
// the HUD, and runs the main loop.
import * as THREE from 'three';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js';
import { World } from './core.js';
import { buildArena, ARENA_HALF, SPAWN_Z, TargetDummy } from './arena.js';
import { robotById } from './robots/index.js';
import { Effects, Audio } from './fx.js';
import { Input } from './input.js';
import { BotBrain } from './bot.js';
import { TwinPanel } from './twin.js';

const PLAYER_COLOR = 0xff7a1a;
const BOT_COLOR = 0x3aa0ff;
const CAMERA_MODES = ['chase', 'close', 'top', 'orbit'];
const isTouch = matchMedia('(pointer: coarse)').matches;

export class Game {
  constructor() {
    this.canvas = document.getElementById('view');
    this.state = 'menu';
    this.hq = !isTouch;
    this.#setupRenderer();
    this.effects = new Effects(this.scene);
    this.audio = new Audio();
    this.input = new Input(this.canvas, document.getElementById('touch'));
    this.twin = new TwinPanel(document.getElementById('twin'));
    this.camMode = 'chase';
    this.robots = [];
    this.dummies = [];
    this.bots = [];
    this.player = null;
    this.settings = null;
    this.over = false;
    this.clock = new THREE.Clock();
    this.#bindHud();
  }

  #setupRenderer() {
    const r = new THREE.WebGLRenderer({ canvas: this.canvas, antialias: true, powerPreference: 'high-performance' });
    r.outputColorSpace = THREE.SRGBColorSpace;
    r.toneMapping = THREE.ACESFilmicToneMapping;
    r.toneMappingExposure = 1.05;
    this.renderer = r;
    this.scene = new THREE.Scene();
    this.scene.background = new THREE.Color(0x1b1e23);
    this.scene.fog = new THREE.Fog(0x1b1e23, 9, 22);
    this.camera = new THREE.PerspectiveCamera(60, 1, 0.02, 60);
    this.camera.position.set(0.8, 0.6, -1.2);

    this.scene.add(new THREE.HemisphereLight(0xdfe8ff, 0x30302a, 1.1));
    const sun = new THREE.DirectionalLight(0xffffff, 2.2);
    sun.position.set(3, 6, 2);
    sun.shadow.mapSize.set(2048, 2048);
    const s = ARENA_HALF + 1;
    Object.assign(sun.shadow.camera, { left: -s, right: s, top: s, bottom: -s, near: 0.5, far: 20 });
    sun.shadow.bias = -0.0005;
    sun.shadow.normalBias = 0.01;
    this.scene.add(sun);
    this.sun = sun;

    this.orbit = new OrbitControls(this.camera, this.canvas);
    this.orbit.enabled = false;
    this.orbit.enableDamping = true;
    this.orbit.maxPolarAngle = Math.PI * 0.49;
    this.orbit.minDistance = 0.3;
    this.orbit.maxDistance = 12;

    this.applyQuality();
    const resize = () => {
      const w = innerWidth;
      const h = innerHeight;
      this.renderer.setSize(w, h, false);
      this.camera.aspect = w / h;
      this.camera.fov = w < h ? 72 : 60;
      this.camera.updateProjectionMatrix();
    };
    addEventListener('resize', resize);
    addEventListener('orientationchange', () => setTimeout(resize, 200));
    resize();
  }

  applyQuality() {
    const dpr = Math.min(devicePixelRatio || 1, this.hq ? 2 : 1.5);
    this.renderer.setPixelRatio(dpr);
    this.renderer.shadowMap.enabled = this.hq;
    this.renderer.shadowMap.type = THREE.PCFSoftShadowMap;
    this.sun.castShadow = this.hq;
    this.scene.traverse((o) => {
      if (o.isMesh && o.material && !o.material.transparent) {
        o.castShadow = this.hq;
        o.receiveShadow = this.hq;
      }
      if (o.material) o.material.needsUpdate = true;
    });
  }

  /* ---------------- world & robots ---------------- */

  #resetWorld() {
    for (const r of this.robots) r.dispose();
    for (const d of this.dummies) d.dispose();
    this.arena?.removeFromParent();
    this.world?.free();
    this.effects.clear();
    this.robots = [];
    this.dummies = [];
    this.bots = [];
    this.world = new World(ARENA_HALF);
    this.arena = buildArena(this.scene, this.world, { shadows: this.hq });
  }

  #makeRobot(robotId, opts) {
    const def = robotById(robotId);
    const r = def.create({ scene: this.scene, world: this.world, ...opts });
    r.team = opts.team;
    this.robots.push(r);
    if (this.hq) r.root.traverse((o) => o.isMesh && (o.castShadow = true));
    return r;
  }

  /** Garage preview: the player's robot standing on a turntable. */
  showGarage(settings) {
    this.state = 'menu';
    this.settings = settings;
    this.#resetWorld();
    this.player = this.#makeRobot(settings.robot, { name: 'You', color: PLAYER_COLOR, loadout: settings.you, team: 'player' });
    this.player.spawn(0, -SPAWN_Z, 0);
    this.over = false;
    this.input.releasePointer();
    document.getElementById('hud').classList.add('hidden');
    this.twin.toggle(false);
  }

  updateGarageLoadout(settings) {
    this.settings = settings;
    this.player?.setLoadout(settings.you);
  }

  startMatch(settings) {
    this.settings = settings;
    this.#resetWorld();
    this.player = this.#makeRobot(settings.robot, { name: 'You', color: PLAYER_COLOR, loadout: settings.you, team: 'player' });
    this.player.spawn(0, -SPAWN_Z, 0);
    if (settings.mode === 'battle') {
      const bot = this.#makeRobot(settings.robot, { name: 'Bot', color: BOT_COLOR, loadout: settings.bot, team: 'bot', isBot: true });
      bot.spawn(0, SPAWN_Z, Math.PI);
      this.bots.push(new BotBrain(bot, settings.botLevel));
    } else {
      for (const [x, z] of [[-0.8, 0.6], [0.8, 0.6], [0, 2.6], [-2.5, -0.8], [2.5, -0.8]]) {
        const d = new TargetDummy(this.scene, this.world, x, z);
        d.face_toward(0, -SPAWN_Z);
        this.dummies.push(d);
      }
    }
    this.over = false;
    this.state = 'playing';
    document.getElementById('hud').classList.remove('hidden');
    document.getElementById('hp-bot').classList.toggle('hidden', settings.mode !== 'battle');
    document.body.classList.toggle('touch', isTouch);
    document.getElementById('touch').classList.toggle('hidden', !isTouch);
    this.banner(settings.mode === 'battle' ? 'FIGHT!' : 'FREE DRIVE', settings.mode === 'battle' ? '' : 'Knock over the targets', 1.4);
    this.camBlend = 0;
  }

  /** Things that can be hit (robots + practice targets). */
  targets() {
    return [...this.robots, ...this.dummies];
  }

  showPreview(robot) {
    return robot === this.player;
  }

  hitBody(bodyId, dmg, from, point) {
    const t = this.targets().find((x) => x.bodyId === bodyId);
    if (!t || !t.alive) return;
    t.damage(dmg, from);
    this.effects.sparks(point ?? t.center(new THREE.Vector3()), from?.color ?? 0xffcc66, 12);
    this.audio.play('hit');
    if (!t.alive && t.type) {
      this.audio.play('boom');
      for (let i = 0; i < 12; i++) this.effects.sparks(t.center(new THREE.Vector3()), 0xff6a00, 4);
    }
  }

  /* ---------------- HUD ---------------- */

  #bindHud() {
    this.hud = {
      you: document.querySelector('#hp-you .bar i'),
      bot: document.querySelector('#hp-bot .bar i'),
      status: document.getElementById('status'),
      banner: document.getElementById('banner'),
      radar: document.getElementById('radar'),
    };
    this.radarCtx = this.hud.radar.getContext('2d');
    this.bannerT = 0;
    document.getElementById('btn-cam').addEventListener('click', () => this.cycleCamera());
    document.getElementById('btn-twin').addEventListener('click', () => this.twin.toggle());
    document.getElementById('btn-full').addEventListener('click', () => toggleFullscreen());
    if (!document.fullscreenEnabled && !document.webkitFullscreenEnabled) document.getElementById('btn-full').classList.add('hidden');
    addEventListener('keydown', (e) => {
      if (this.state !== 'playing') return;
      if (e.code === 'KeyC') this.cycleCamera();
      if (e.code === 'KeyT') this.twin.toggle();
    });
  }

  cycleCamera() {
    const i = CAMERA_MODES.indexOf(this.camMode);
    this.camMode = CAMERA_MODES[(i + 1) % CAMERA_MODES.length];
    this.orbit.enabled = this.camMode === 'orbit';
    if (this.orbit.enabled) {
      this.input.releasePointer();
      this.orbit.target.copy(this.player.pos).setY(0.1);
    }
    this.banner('', `Camera: ${this.camMode}`, 0.8, true);
  }

  banner(title, sub = '', time = 1.5, small = false) {
    const b = this.hud.banner;
    b.innerHTML = small ? `<small>${sub}</small>` : `${title}${sub ? `<small>${sub}</small>` : ''}`;
    b.classList.add('show');
    this.bannerT = time;
  }

  endBanner(won) {
    const b = this.hud.banner;
    b.innerHTML = `<span style="color:${won ? 'var(--ok)' : 'var(--bad)'}">${won ? 'VICTORY' : 'DEFEATED'}</span><small>${won ? 'Bot destroyed' : 'Your hexapod is down'}</small><button id="again">Rematch</button><button id="garage">Garage</button>`;
    b.classList.add('show');
    this.bannerT = Infinity;
    b.querySelector('#again').addEventListener('click', () => this.startMatch(this.settings));
    b.querySelector('#garage').addEventListener('click', () => this.onGarage?.());
    this.input.releasePointer();
  }

  #updateHud(dt) {
    const p = this.player;
    this.hud.you.style.width = `${(p.hp / p.maxHp) * 100}%`;
    const bot = this.bots[0]?.robot;
    if (bot) this.hud.bot.style.width = `${(bot.hp / bot.maxHp) * 100}%`;

    let html = '';
    for (const a of Object.values(p.attachments)) {
      if (!a?.hud) continue;
      const h = a.hud();
      html += `<div class="row"><span class="lbl">${h.label}</span><span class="mini"><i style="width:${Math.round(h.bar * 100)}%;background:${h.barColor}"></i></span><span class="note">${h.note}</span></div>`;
    }
    if (!html) html = '<div class="row"><span class="note">No attachments. Open the Garage to add some.</span></div>';
    if (this.hud.status.innerHTML !== html) this.hud.status.innerHTML = html;

    if (this.bannerT !== Infinity) {
      this.bannerT -= dt;
      if (this.bannerT <= 0) this.hud.banner.classList.remove('show');
    }
    this.#drawRadar();
  }

  #drawRadar() {
    const c = this.radarCtx;
    const W = this.hud.radar.width;
    const R = W / 2;
    const p = this.player;
    c.clearRect(0, 0, W, W);
    const sensor = p.sensor;
    const hasSensor = !!p.attachments.front;
    const range = sensor?.maxDist ?? 4;
    const scale = (R - 6) / range;
    // Radar is "heading up": rotate world so the robot faces up
    const toScreen = (wx, wz) => {
      const dx = wx - p.pos.x;
      const dz = wz - p.pos.z;
      const s = Math.sin(-p.yaw);
      const co = Math.cos(-p.yaw);
      const lx = dx * co + dz * s;
      const lz = -dx * s + dz * co;
      return [R - lx * scale, R - lz * scale];
    };
    c.strokeStyle = 'rgba(255,255,255,0.12)';
    c.beginPath();
    c.arc(R, R, R / 2, 0, Math.PI * 2);
    c.stroke();
    if (!hasSensor) {
      c.fillStyle = 'rgba(255,255,255,0.45)';
      c.font = '12px system-ui, sans-serif';
      c.textAlign = 'center';
      c.fillText('No sensor', R, R + 4);
    } else if (sensor) {
      if (sensor.kind === 'lidar' && sensor.points) {
        c.fillStyle = 'rgba(255,107,107,0.85)';
        for (let i = 0; i < sensor.rays; i++) {
          const d = sensor.points[i * 2];
          if (d >= range - 0.01) continue;
          const a = p.yaw - sensor.fov / 2 + (sensor.fov * i) / (sensor.rays - 1);
          const [x, y] = toScreen(p.pos.x + Math.sin(a) * d, p.pos.z + Math.cos(a) * d);
          c.fillRect(x - 1, y - 1, 2, 2);
        }
      }
      if (sensor.kind === 'sonar') {
        c.fillStyle = 'rgba(102,204,255,0.18)';
        c.beginPath();
        c.moveTo(R, R);
        c.arc(R, R, sensor.reading * scale, -Math.PI / 2 - sensor.fov / 2, -Math.PI / 2 + sensor.fov / 2);
        c.closePath();
        c.fill();
      }
      for (const ct of sensor.contacts ?? []) {
        const [x, y] = toScreen(p.pos.x + Math.sin(ct.angle) * ct.dist, p.pos.z + Math.cos(ct.angle) * ct.dist);
        const t = this.targets().find((q) => q.bodyId === ct.bodyId);
        c.fillStyle = t?.team === 'bot' ? '#3aa0ff' : '#ffffff';
        c.beginPath();
        c.arc(x, y, 4, 0, Math.PI * 2);
        c.fill();
      }
    }
    // Me
    c.fillStyle = '#ff7a1a';
    c.beginPath();
    c.moveTo(R, R - 7);
    c.lineTo(R - 5, R + 5);
    c.lineTo(R + 5, R + 5);
    c.closePath();
    c.fill();
  }

  /* ---------------- camera ---------------- */

  #updateCamera(dt) {
    const p = this.player;
    if (!p) return;
    const cam = this.camera;
    if (this.state === 'menu') {
      // Slow turntable around the robot, framed to the right of the menu
      const t = performance.now() / 1000;
      const a = t * 0.25;
      const target = p.pos.clone().add(new THREE.Vector3(0, 0.1, 0));
      const narrow = innerWidth < 640;
      cam.position.set(target.x + Math.sin(a) * 0.75, target.y + 0.35, target.z + Math.cos(a) * 0.75);
      cam.lookAt(target);
      if (!narrow) {
        // shift the robot to the right so the menu doesn't cover it
        const right = new THREE.Vector3().setFromMatrixColumn(cam.matrixWorld, 0);
        cam.position.addScaledVector(right, -0.28);
      }
      return;
    }
    if (this.camMode === 'orbit') {
      this.orbit.target.lerp(new THREE.Vector3(p.pos.x, 0.1, p.pos.z), 0.1);
      this.orbit.update();
      return;
    }
    const fwd = p.forward(new THREE.Vector3());
    let want;
    let look;
    if (this.camMode === 'top') {
      want = new THREE.Vector3(p.pos.x, 3.2, p.pos.z - 1.2);
      look = new THREE.Vector3(p.pos.x, 0, p.pos.z + 0.2);
    } else {
      const close = this.camMode === 'close';
      // Phones held upright see less side-to-side, so pull the camera back
      const portrait = cam.aspect < 1 ? 1.6 : 1;
      const back = (close ? 0.55 : 0.95) * portrait;
      const up = (close ? 0.22 : 0.42) * (portrait > 1 ? 1.3 : 1) + p.aimPitch * 0.15;
      want = p.pos.clone().addScaledVector(fwd, -back).add(new THREE.Vector3(0, up, 0));
      look = p.pos.clone().addScaledVector(fwd, portrait > 1 ? 0.2 : 0.6).add(new THREE.Vector3(0, 0.12 + p.aimPitch * 0.4, 0));
      // Keep the camera inside the arena walls; lift it when squeezed against one
      const lim = ARENA_HALF - 0.05;
      const over = Math.max(0, Math.abs(want.x) - lim, Math.abs(want.z) - lim);
      want.x = THREE.MathUtils.clamp(want.x, -lim, lim);
      want.z = THREE.MathUtils.clamp(want.z, -lim, lim);
      want.y += over * 0.25;
    }
    const k = 1 - Math.exp(-dt * 7);
    cam.position.lerp(want, this.camBlend < 1 ? Math.max(k, 0.06) : k);
    this.camLook = this.camLook ?? look.clone();
    this.camLook.lerp(look, Math.min(1, k * 1.6));
    cam.lookAt(this.camLook);
    this.camBlend = Math.min(1, (this.camBlend ?? 0) + dt);
  }

  /* ---------------- main loop ---------------- */

  pause(on) {
    if (this.state === 'menu') return;
    this.state = on ? 'paused' : 'playing';
    document.getElementById('pause').classList.toggle('hidden', !on);
    this.input.enabled = !on;
    if (on) this.input.releasePointer();
  }

  start() {
    const loop = () => {
      requestAnimationFrame(loop);
      const dt = Math.min(0.05, this.clock.getDelta());
      this.frame(dt);
    };
    loop();
  }

  frame(dt) {
    if (!this.world) return;
    const playing = this.state === 'playing';
    if (this.state === 'menu') {
      this.player?.update(dt, { moveX: 0, moveZ: 0, turn: 0, aim: 0 }, this);
    } else if (playing) {
      const cmd = this.input.read(dt);
      if (cmd.pause) this.pause(true);
      this.player.update(dt, this.over ? {} : cmd, this);
      for (const b of this.bots) b.robot.update(dt, this.over ? {} : b.think(dt, this), this);
      for (const d of this.dummies) d.update(dt);

      const events = this.world.step(dt);
      for (let i = 0; i < events.length; i += 8) {
        const [kind, target, dmg, x, y, z, , owner] = events.subarray(i, i + 8);
        const pos = new THREE.Vector3(x, y, z);
        if (kind === 1) this.hitBody(target, dmg, this.robots.find((r) => r.bodyId === owner), pos);
        else this.effects.splat(pos);
      }
      this.effects.drawProjectiles(this.world.projectile_state());

      // Smoke from badly damaged robots
      for (const r of this.robots) if (r.hp < 35 && Math.random() < (r.alive ? 0.15 : 0.4)) this.effects.smoke(r.center(new THREE.Vector3()));

      if (this.settings.mode === 'battle' && !this.over) {
        const bot = this.bots[0].robot;
        if (!this.player.alive || !bot.alive) {
          this.over = true;
          setTimeout(() => this.endBanner(this.player.alive), 900);
        }
      }
      this.#updateHud(dt);
      this.twin.update(dt, this.player);
    }
    this.effects.update(playing ? dt : 0);
    this.#updateCamera(dt);
    this.renderer.render(this.scene, this.camera);
  }
}

function toggleFullscreen() {
  const d = document;
  const el = d.documentElement;
  if (d.fullscreenElement || d.webkitFullscreenElement) (d.exitFullscreen || d.webkitExitFullscreen)?.call(d);
  else (el.requestFullscreen || el.webkitRequestFullscreen)?.call(el)?.catch?.(() => {});
}
