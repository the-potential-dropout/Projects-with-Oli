// Visual effects (laser beams, sparks, potato splats) and tiny synthesized
// sound effects, so the app has no media files to download.
import * as THREE from 'three';

export class Effects {
  constructor(scene) {
    this.scene = scene;
    this.beams = [];
    this.particles = [];
    this.splats = [];

    // Particle pool rendered as one Points object (cheap on phones)
    this.maxP = 400;
    this.pGeo = new THREE.BufferGeometry();
    this.pPos = new Float32Array(this.maxP * 3);
    this.pCol = new Float32Array(this.maxP * 3);
    this.pGeo.setAttribute('position', new THREE.BufferAttribute(this.pPos, 3));
    this.pGeo.setAttribute('color', new THREE.BufferAttribute(this.pCol, 3));
    this.points = new THREE.Points(this.pGeo, new THREE.PointsMaterial({ size: 0.035, vertexColors: true, transparent: true, depthWrite: false }));
    this.points.frustumCulled = false;
    scene.add(this.points);

    this.beamGeo = new THREE.CylinderGeometry(1, 1, 1, 8, 1, true);
    this.beamGeo.translate(0, 0.5, 0);
    this.beamGeo.rotateX(Math.PI / 2);

    this.potatoGeo = new THREE.SphereGeometry(0.025, 12, 10);
    this.potatoMat = new THREE.MeshStandardMaterial({ color: 0xc8a165, roughness: 0.85 });
    this.projMeshes = [];
    this.splatGeo = new THREE.CircleGeometry(0.06, 12);
    this.splatGeo.rotateX(-Math.PI / 2);
  }

  beam(from, to, color) {
    const len = from.distanceTo(to);
    const m = new THREE.Mesh(this.beamGeo, new THREE.MeshBasicMaterial({ color, transparent: true, opacity: 0.9, depthWrite: false, blending: THREE.AdditiveBlending }));
    m.position.copy(from);
    m.lookAt(to);
    m.scale.set(0.006, 0.006, len);
    this.scene.add(m);
    this.beams.push({ m, t: 0.09 });
  }

  sparks(pos, color = 0xffcc66, n = 14) {
    const c = new THREE.Color(color);
    for (let i = 0; i < n; i++) {
      this.particles.push({
        p: pos.clone(),
        v: new THREE.Vector3((Math.random() - 0.5) * 2.2, Math.random() * 2.2, (Math.random() - 0.5) * 2.2),
        c,
        t: 0.35 + Math.random() * 0.3,
      });
    }
  }

  puff(pos) {
    const c = new THREE.Color(0xdddddd);
    for (let i = 0; i < 10; i++) {
      this.particles.push({ p: pos.clone(), v: new THREE.Vector3((Math.random() - 0.5) * 0.6, Math.random() * 0.6, (Math.random() - 0.5) * 0.6), c, t: 0.4, drag: true });
    }
  }

  smoke(pos) {
    const c = new THREE.Color(0x444444);
    this.particles.push({ p: pos.clone(), v: new THREE.Vector3((Math.random() - 0.5) * 0.1, 0.35 + Math.random() * 0.2, (Math.random() - 0.5) * 0.1), c, t: 1.2, drag: true });
  }

  splat(pos) {
    const m = new THREE.Mesh(this.splatGeo, new THREE.MeshBasicMaterial({ color: 0xe8d9a8, transparent: true, opacity: 0.85, depthWrite: false }));
    m.position.set(pos.x, 0.002 + Math.random() * 0.001, pos.z);
    m.scale.setScalar(0.7 + Math.random() * 0.6);
    this.scene.add(m);
    this.splats.push({ m, t: 5 });
    this.sparks(pos, 0xe8d9a8, 10);
  }

  /** Draw projectiles from the Rust world's flat [x,y,z,kind]*n buffer. */
  drawProjectiles(state) {
    const n = state.length / 4;
    while (this.projMeshes.length < n) {
      const m = new THREE.Mesh(this.potatoGeo, this.potatoMat);
      m.castShadow = true;
      this.scene.add(m);
      this.projMeshes.push(m);
    }
    for (let i = 0; i < this.projMeshes.length; i++) {
      const m = this.projMeshes[i];
      m.visible = i < n;
      if (i < n) {
        m.position.set(state[i * 4], state[i * 4 + 1], state[i * 4 + 2]);
        m.rotation.x += 0.3;
      }
    }
  }

  update(dt) {
    for (let i = this.beams.length - 1; i >= 0; i--) {
      const b = this.beams[i];
      b.t -= dt;
      b.m.material.opacity = Math.max(0, b.t / 0.09);
      if (b.t <= 0) {
        b.m.removeFromParent();
        b.m.material.dispose();
        this.beams.splice(i, 1);
      }
    }
    for (let i = this.splats.length - 1; i >= 0; i--) {
      const s = this.splats[i];
      s.t -= dt;
      s.m.material.opacity = Math.min(0.85, s.t);
      if (s.t <= 0) {
        s.m.removeFromParent();
        s.m.material.dispose();
        this.splats.splice(i, 1);
      }
    }
    let n = 0;
    for (let i = this.particles.length - 1; i >= 0; i--) {
      const p = this.particles[i];
      p.t -= dt;
      if (p.t <= 0 || n >= this.maxP) {
        this.particles.splice(i, 1);
        continue;
      }
      if (p.drag) p.v.multiplyScalar(1 - dt * 2);
      else p.v.y -= 6 * dt;
      p.p.addScaledVector(p.v, dt);
      if (p.p.y < 0) {
        p.p.y = 0;
        p.v.y *= -0.3;
      }
      this.pPos.set([p.p.x, p.p.y, p.p.z], n * 3);
      this.pCol.set([p.c.r, p.c.g, p.c.b], n * 3);
      n++;
    }
    this.pGeo.setDrawRange(0, n);
    this.pGeo.attributes.position.needsUpdate = true;
    this.pGeo.attributes.color.needsUpdate = true;
  }

  clear() {
    for (const b of this.beams) b.m.removeFromParent();
    for (const s of this.splats) s.m.removeFromParent();
    this.beams = [];
    this.splats = [];
    this.particles = [];
    this.drawProjectiles(new Float32Array(0));
  }
}

export class Audio {
  constructor() {
    this.ctx = null;
    this.muted = false;
    this.last = {};
    // Browsers (especially iOS Safari) only allow sound after a user gesture
    const unlock = () => {
      try {
        if (!this.ctx) {
          const AC = window.AudioContext || window.webkitAudioContext;
          if (AC) this.ctx = new AC();
        }
        this.ctx?.resume?.();
      } catch {
        this.ctx = null;
      }
    };
    addEventListener('pointerdown', unlock);
    addEventListener('keydown', unlock);
  }

  play(name) {
    if (this.muted || !this.ctx || this.ctx.state !== 'running') return;
    const now = this.ctx.currentTime;
    if (this.last[name] && now - this.last[name] < 0.05) return;
    this.last[name] = now;
    const o = this.ctx.createOscillator();
    const g = this.ctx.createGain();
    o.connect(g);
    g.connect(this.ctx.destination);
    const set = (type, f0, f1, dur, vol) => {
      o.type = type;
      o.frequency.setValueAtTime(f0, now);
      o.frequency.exponentialRampToValueAtTime(f1, now + dur);
      g.gain.setValueAtTime(vol, now);
      g.gain.exponentialRampToValueAtTime(0.0001, now + dur);
      o.start(now);
      o.stop(now + dur + 0.02);
    };
    switch (name) {
      case 'laser':
        return set('sawtooth', 1400, 300, 0.12, 0.06);
      case 'potato':
        return set('triangle', 180, 50, 0.25, 0.25);
      case 'hit':
        return set('square', 300, 90, 0.12, 0.08);
      case 'clank':
        return set('square', 900, 200, 0.1, 0.08);
      case 'whoosh':
        return set('sine', 500, 150, 0.15, 0.05);
      case 'boom':
        return set('sawtooth', 120, 30, 0.7, 0.2);
      default:
        return set('sine', 660, 660, 0.08, 0.05);
    }
  }
}
