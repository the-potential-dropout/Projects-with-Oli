// The battle arena: floor, walls, cover obstacles and practice targets.
// Every obstacle is registered in the Rust world so robots bump into it and
// lasers/potatoes/sensors see it.
import * as THREE from 'three';

export const ARENA_HALF = 3; // metres, so the arena is 6 m x 6 m
export const SPAWN_Z = 2.5; // player spawns at -SPAWN_Z, bot at +SPAWN_Z

// [centerX, centerZ, sizeX, height, sizeZ, kind]
// Low walls (18 cm) block a hexapod's view and shots; potatoes arc over them.
const LAYOUT = [
  [0, 0, 0.5, 0.5, 0.5, 'pillar'],
  [1.25, 1.2, 0.4, 0.4, 0.4, 'crate'],
  [-1.25, 1.2, 0.4, 0.4, 0.4, 'crate'],
  [1.25, -1.2, 0.4, 0.4, 0.4, 'crate'],
  [-1.25, -1.2, 0.4, 0.4, 0.4, 'crate'],
  [2.1, 0, 0.12, 0.18, 1.1, 'wall'],
  [-2.1, 0, 0.12, 0.18, 1.1, 'wall'],
  [0, 1.85, 1.0, 0.18, 0.12, 'wall'],
  [0, -1.85, 1.0, 0.18, 0.12, 'wall'],
];

function gridTexture() {
  const c = document.createElement('canvas');
  c.width = c.height = 256;
  const g = c.getContext('2d');
  g.fillStyle = '#3a3f47';
  g.fillRect(0, 0, 256, 256);
  g.strokeStyle = '#4b525c';
  g.lineWidth = 2;
  for (let i = 0; i <= 256; i += 64) {
    g.beginPath();
    g.moveTo(i, 0);
    g.lineTo(i, 256);
    g.moveTo(0, i);
    g.lineTo(256, i);
    g.stroke();
  }
  g.strokeStyle = '#444a53';
  g.lineWidth = 1;
  for (let i = 0; i <= 256; i += 16) {
    g.beginPath();
    g.moveTo(i, 0);
    g.lineTo(i, 256);
    g.moveTo(0, i);
    g.lineTo(256, i);
    g.stroke();
  }
  const t = new THREE.CanvasTexture(c);
  t.wrapS = t.wrapT = THREE.RepeatWrapping;
  t.repeat.set(ARENA_HALF * 2, ARENA_HALF * 2); // one big square per metre
  t.anisotropy = 4;
  t.colorSpace = THREE.SRGBColorSpace;
  return t;
}

function crateTexture() {
  const c = document.createElement('canvas');
  c.width = c.height = 128;
  const g = c.getContext('2d');
  g.fillStyle = '#a0753f';
  g.fillRect(0, 0, 128, 128);
  g.strokeStyle = '#6e4d26';
  g.lineWidth = 10;
  g.strokeRect(5, 5, 118, 118);
  g.beginPath();
  g.moveTo(5, 5);
  g.lineTo(123, 123);
  g.stroke();
  const t = new THREE.CanvasTexture(c);
  t.colorSpace = THREE.SRGBColorSpace;
  return t;
}

export function buildArena(scene, world, { shadows }) {
  const group = new THREE.Group();
  scene.add(group);

  const floor = new THREE.Mesh(new THREE.PlaneGeometry(ARENA_HALF * 2, ARENA_HALF * 2), new THREE.MeshStandardMaterial({ map: gridTexture(), roughness: 0.9 }));
  floor.rotation.x = -Math.PI / 2;
  floor.receiveShadow = shadows;
  group.add(floor);

  // Outer ground so the world doesn't end at the arena edge
  const outer = new THREE.Mesh(new THREE.CircleGeometry(40, 48), new THREE.MeshStandardMaterial({ color: 0x23262b, roughness: 1 }));
  outer.rotation.x = -Math.PI / 2;
  outer.position.y = -0.002;
  group.add(outer);

  // Perimeter barrier (visual; the Rust world treats the edge as a wall)
  const barrierMat = new THREE.MeshStandardMaterial({ color: 0xffc23d, roughness: 0.6 });
  const stripeMat = new THREE.MeshStandardMaterial({ color: 0x1b1b1b, roughness: 0.6 });
  const h = 0.25;
  for (const [x, z, sx, sz] of [
    [0, ARENA_HALF + 0.05, ARENA_HALF * 2 + 0.2, 0.1],
    [0, -ARENA_HALF - 0.05, ARENA_HALF * 2 + 0.2, 0.1],
    [ARENA_HALF + 0.05, 0, 0.1, ARENA_HALF * 2],
    [-ARENA_HALF - 0.05, 0, 0.1, ARENA_HALF * 2],
  ]) {
    const w = new THREE.Mesh(new THREE.BoxGeometry(sx, h, sz), barrierMat);
    w.position.set(x, h / 2, z);
    w.castShadow = w.receiveShadow = shadows;
    group.add(w);
    const s = new THREE.Mesh(new THREE.BoxGeometry(sx + 0.002, 0.04, sz + 0.002), stripeMat);
    s.position.set(x, h - 0.04, z);
    group.add(s);
  }

  world.clear_obstacles();
  const crateMat = new THREE.MeshStandardMaterial({ map: crateTexture(), roughness: 0.8 });
  const pillarMat = new THREE.MeshStandardMaterial({ color: 0x5b6470, roughness: 0.7, metalness: 0.2 });
  const wallMat = new THREE.MeshStandardMaterial({ color: 0x8d99a6, roughness: 0.7 });
  for (const [x, z, sx, sy, sz, kind] of LAYOUT) {
    const m = new THREE.Mesh(new THREE.BoxGeometry(sx, sy, sz), kind === 'crate' ? crateMat : kind === 'pillar' ? pillarMat : wallMat);
    m.position.set(x, sy / 2, z);
    m.castShadow = m.receiveShadow = shadows;
    group.add(m);
    world.add_obstacle(x - sx / 2, 0, z - sz / 2, x + sx / 2, sy, z + sz / 2);
  }
  return group;
}

/** A practice target for Free Drive mode: knocks over when hit, then resets. */
export class TargetDummy {
  constructor(scene, world, x, z) {
    this.team = 'targets';
    this.cfg = { hitRadius: 0.13 };
    this.world = world;
    this.pos = new THREE.Vector3(x, 0, z);
    this.maxHp = 30;
    this.hp = 30;
    this.alive = true;
    this.respawn = 0;
    this.bodyId = world.add_body(0.13);
    this.group = new THREE.Group();
    this.group.position.copy(this.pos);
    const post = new THREE.Mesh(new THREE.CylinderGeometry(0.012, 0.012, 0.18, 8), new THREE.MeshStandardMaterial({ color: 0x777777 }));
    post.position.y = 0.09;
    this.group.add(post);
    this.mat = new THREE.MeshStandardMaterial({ color: 0xffffff, emissive: 0x000000 });
    const face = new THREE.Mesh(new THREE.CylinderGeometry(0.12, 0.12, 0.025, 24), this.mat);
    face.rotation.x = Math.PI / 2;
    face.position.y = 0.3;
    this.face = face;
    this.group.add(face);
    const ring = new THREE.Mesh(new THREE.RingGeometry(0.04, 0.08, 24), new THREE.MeshBasicMaterial({ color: 0xff3b3b, side: THREE.DoubleSide }));
    ring.position.set(0, 0.3, 0.014);
    this.group.add(ring);
    this.group.traverse((o) => (o.castShadow = true));
    scene.add(this.group);
    world.set_body(this.bodyId, x, 0.3, z, true);
  }

  center(out = new THREE.Vector3()) {
    return out.set(this.pos.x, 0.3, this.pos.z);
  }

  damage(amount) {
    if (!this.alive) return;
    this.hp -= amount;
    this.mat.emissive.setHex(0xff4400);
    if (this.hp <= 0) {
      this.alive = false;
      this.respawn = 3;
      this.world.set_body(this.bodyId, this.pos.x, 0.3, this.pos.z, false);
    }
  }

  face_toward(x, z) {
    this.group.rotation.y = Math.atan2(x - this.pos.x, z - this.pos.z);
  }

  update(dt) {
    this.mat.emissive.multiplyScalar(Math.max(0, 1 - dt * 6));
    if (!this.alive) {
      this.group.rotation.x = Math.min(Math.PI / 2, this.group.rotation.x + dt * 6);
      this.respawn -= dt;
      if (this.respawn <= 0) {
        this.alive = true;
        this.hp = this.maxHp;
        this.group.rotation.x = 0;
        this.world.set_body(this.bodyId, this.pos.x, 0.3, this.pos.z, true);
      }
    }
  }

  dispose() {
    this.group.removeFromParent();
    this.world.set_body(this.bodyId, 0, -100, 0, false);
  }
}
