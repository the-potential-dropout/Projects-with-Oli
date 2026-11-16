//! Arena physics: projectiles (potatoes, foam darts...), robot hit spheres,
//! box obstacles, ray casts for lasers and distance sensors, and keeping
//! robots inside the arena and out of walls.

use wasm_bindgen::prelude::*;

const GRAVITY: f32 = -9.81;

#[derive(Clone, Copy)]
struct Aabb {
    min: [f32; 3],
    max: [f32; 3],
}

#[derive(Clone, Copy)]
struct Body {
    pos: [f32; 3],
    radius: f32,
    active: bool,
}

#[derive(Clone, Copy)]
struct Projectile {
    pos: [f32; 3],
    vel: [f32; 3],
    radius: f32,
    drag: f32,
    owner: i32,
    damage: f32,
    kind: u32,
    life: f32,
}

/// Event kinds written by `step`.
pub const EVENT_HIT_BODY: f32 = 1.0;
pub const EVENT_HIT_GROUND: f32 = 2.0;
pub const EVENT_HIT_WALL: f32 = 3.0;

#[wasm_bindgen]
pub struct World {
    half_size: f32,
    obstacles: Vec<Aabb>,
    bodies: Vec<Body>,
    projectiles: Vec<Projectile>,
    events: Vec<f32>,
}

#[wasm_bindgen]
impl World {
    /// Square arena from -half_size to +half_size on X and Z.
    #[wasm_bindgen(constructor)]
    pub fn new(half_size: f32) -> World {
        World { half_size, obstacles: Vec::new(), bodies: Vec::new(), projectiles: Vec::new(), events: Vec::new() }
    }

    pub fn clear_obstacles(&mut self) {
        self.obstacles.clear();
    }

    pub fn add_obstacle(&mut self, minx: f32, miny: f32, minz: f32, maxx: f32, maxy: f32, maxz: f32) {
        self.obstacles.push(Aabb { min: [minx, miny, minz], max: [maxx, maxy, maxz] });
    }

    /// Register a robot hit sphere. Returns its id.
    pub fn add_body(&mut self, radius: f32) -> usize {
        self.bodies.push(Body { pos: [0.0; 3], radius, active: true });
        self.bodies.len() - 1
    }

    pub fn set_body(&mut self, id: usize, x: f32, y: f32, z: f32, active: bool) {
        if let Some(b) = self.bodies.get_mut(id) {
            b.pos = [x, y, z];
            b.active = active;
        }
    }

    pub fn clear_projectiles(&mut self) {
        self.projectiles.clear();
    }

    #[allow(clippy::too_many_arguments)]
    pub fn spawn_projectile(&mut self, x: f32, y: f32, z: f32, vx: f32, vy: f32, vz: f32, radius: f32, drag: f32, owner: i32, damage: f32, kind: u32) {
        self.projectiles.push(Projectile { pos: [x, y, z], vel: [vx, vy, vz], radius, drag, owner, damage, kind, life: 6.0 });
    }

    pub fn projectile_count(&self) -> usize {
        self.projectiles.len()
    }

    /// [x, y, z, kind] * n for rendering.
    pub fn projectile_state(&self) -> Vec<f32> {
        let mut out = Vec::with_capacity(self.projectiles.len() * 4);
        for p in &self.projectiles {
            out.extend_from_slice(&[p.pos[0], p.pos[1], p.pos[2], p.kind as f32]);
        }
        out
    }

    /// Advance all projectiles. Uses sub-steps so fast projectiles can't tunnel
    /// through thin robots. Returns events as
    /// [event_kind, target_body (or -1), damage, x, y, z, projectile_kind, owner] * n.
    pub fn step(&mut self, dt: f32) -> Vec<f32> {
        self.events.clear();
        let mut i = 0;
        while i < self.projectiles.len() {
            let mut p = self.projectiles[i];
            let speed = (p.vel[0] * p.vel[0] + p.vel[1] * p.vel[1] + p.vel[2] * p.vel[2]).sqrt();
            let substeps = ((speed * dt) / (p.radius.max(0.01))).ceil().clamp(1.0, 32.0) as u32;
            let h = dt / substeps as f32;
            let mut dead = false;

            for _ in 0..substeps {
                // Quadratic air drag + gravity (semi-implicit Euler)
                let s = (p.vel[0] * p.vel[0] + p.vel[1] * p.vel[1] + p.vel[2] * p.vel[2]).sqrt();
                for k in 0..3 {
                    p.vel[k] -= p.drag * s * p.vel[k] * h;
                }
                p.vel[1] += GRAVITY * h;
                for k in 0..3 {
                    p.pos[k] += p.vel[k] * h;
                }

                if let Some(id) = self.hit_body(&p) {
                    self.push_event(EVENT_HIT_BODY, id as f32, &p);
                    dead = true;
                    break;
                }
                if p.pos[1] - p.radius <= 0.0 {
                    p.pos[1] = p.radius;
                    self.push_event(EVENT_HIT_GROUND, -1.0, &p);
                    dead = true;
                    break;
                }
                if self.hits_wall(&p) {
                    self.push_event(EVENT_HIT_WALL, -1.0, &p);
                    dead = true;
                    break;
                }
            }

            p.life -= dt;
            if dead || p.life <= 0.0 {
                self.projectiles.swap_remove(i);
            } else {
                self.projectiles[i] = p;
                i += 1;
            }
        }
        self.events.clone()
    }

    /// Cast a ray. Returns [distance, body_id or -1 (obstacle/wall), hit_x, hit_y, hit_z].
    /// `ignore` is a body id to skip (the robot firing).
    #[allow(clippy::too_many_arguments)]
    pub fn raycast(&self, ox: f32, oy: f32, oz: f32, dx: f32, dy: f32, dz: f32, max_dist: f32, ignore: i32) -> Vec<f32> {
        let (t, id) = self.cast([ox, oy, oz], normalize([dx, dy, dz]), max_dist, ignore);
        let d = normalize([dx, dy, dz]);
        vec![t, id as f32, ox + d[0] * t, oy + d[1] * t, oz + d[2] * t]
    }

    /// Horizontal fan of rays, like an ultrasonic sweep or a 2D LiDAR.
    /// Returns [distance, body_id] * rays, from -fov/2 to +fov/2 around `yaw`.
    #[allow(clippy::too_many_arguments)]
    pub fn scan(&self, ox: f32, oy: f32, oz: f32, yaw: f32, fov: f32, rays: u32, max_dist: f32, ignore: i32) -> Vec<f32> {
        let mut out = Vec::with_capacity(rays as usize * 2);
        let n = rays.max(1);
        for r in 0..n {
            let a = if n == 1 { yaw } else { yaw - fov * 0.5 + fov * (r as f32) / ((n - 1) as f32) };
            let dir = [a.sin(), 0.0, a.cos()];
            let (t, id) = self.cast([ox, oy, oz], dir, max_dist, ignore);
            out.push(t);
            out.push(id as f32);
        }
        out
    }

    /// Push a robot footprint (circle on the XZ plane) out of obstacles,
    /// other robots and the arena walls. Returns the corrected [x, z].
    pub fn resolve_body(&self, id: usize, x: f32, z: f32, radius: f32) -> Vec<f32> {
        let (mut x, mut z) = (x, z);
        for _ in 0..3 {
            for o in &self.obstacles {
                let cx = x.clamp(o.min[0], o.max[0]);
                let cz = z.clamp(o.min[2], o.max[2]);
                let (dx, dz) = (x - cx, z - cz);
                let d2 = dx * dx + dz * dz;
                if d2 < radius * radius {
                    if d2 > 1e-8 {
                        let d = d2.sqrt();
                        x = cx + dx / d * radius;
                        z = cz + dz / d * radius;
                    } else {
                        // Centre inside the box: push out along the shortest axis.
                        let pushes = [(o.min[0] - radius - x, 0.0), (o.max[0] + radius - x, 0.0), (0.0, o.min[2] - radius - z), (0.0, o.max[2] + radius - z)];
                        let best = pushes.iter().min_by(|a, b| (a.0.abs() + a.1.abs()).partial_cmp(&(b.0.abs() + b.1.abs())).unwrap()).unwrap();
                        x += best.0;
                        z += best.1;
                    }
                }
            }
            for (j, b) in self.bodies.iter().enumerate() {
                if j == id || !b.active {
                    continue;
                }
                let (dx, dz) = (x - b.pos[0], z - b.pos[2]);
                let min = radius + b.radius;
                let d2 = dx * dx + dz * dz;
                if d2 < min * min && d2 > 1e-8 {
                    let d = d2.sqrt();
                    // Each robot moves half the overlap.
                    let push = (min - d) * 0.5;
                    x += dx / d * push;
                    z += dz / d * push;
                }
            }
            let lim = self.half_size - radius;
            x = x.clamp(-lim, lim);
            z = z.clamp(-lim, lim);
        }
        vec![x, z]
    }

    /// Ballistic aiming helper: launch pitch (radians, low arc) needed to hit
    /// a target `dist` metres away and `dh` metres higher at launch speed `v`,
    /// ignoring drag. Returns NaN if out of range.
    pub fn ballistic_pitch(dist: f32, dh: f32, v: f32) -> f32 {
        let g = -GRAVITY;
        let v2 = v * v;
        let disc = v2 * v2 - g * (g * dist * dist + 2.0 * dh * v2);
        if disc < 0.0 {
            return f32::NAN;
        }
        ((v2 - disc.sqrt()) / (g * dist)).atan()
    }
}

impl World {
    fn push_event(&mut self, kind: f32, target: f32, p: &Projectile) {
        self.events.extend_from_slice(&[kind, target, p.damage, p.pos[0], p.pos[1], p.pos[2], p.kind as f32, p.owner as f32]);
    }

    fn hit_body(&self, p: &Projectile) -> Option<usize> {
        for (id, b) in self.bodies.iter().enumerate() {
            if !b.active || id as i32 == p.owner {
                continue;
            }
            let d2: f32 = (0..3).map(|k| (p.pos[k] - b.pos[k]).powi(2)).sum();
            let r = b.radius + p.radius;
            if d2 <= r * r {
                return Some(id);
            }
        }
        None
    }

    fn hits_wall(&self, p: &Projectile) -> bool {
        if p.pos[0].abs() > self.half_size + 2.0 || p.pos[2].abs() > self.half_size + 2.0 {
            return true;
        }
        self.obstacles.iter().any(|o| (0..3).all(|k| p.pos[k] + p.radius >= o.min[k] && p.pos[k] - p.radius <= o.max[k]))
    }

    fn cast(&self, o: [f32; 3], d: [f32; 3], max_dist: f32, ignore: i32) -> (f32, i32) {
        let mut best = max_dist;
        let mut hit = -2; // -2 = nothing, -1 = obstacle/ground/wall, >=0 body id

        // Ground plane
        if d[1] < -1e-6 {
            let t = -o[1] / d[1];
            if t >= 0.0 && t < best {
                best = t;
                hit = -1;
            }
        }
        // Arena walls (infinitely tall planes at +-half_size)
        for axis in [0usize, 2] {
            if d[axis].abs() > 1e-6 {
                let wall = if d[axis] > 0.0 { self.half_size } else { -self.half_size };
                let t = (wall - o[axis]) / d[axis];
                if t >= 0.0 && t < best {
                    best = t;
                    hit = -1;
                }
            }
        }
        for b in &self.obstacles {
            if let Some(t) = ray_aabb(o, d, b) {
                if t < best {
                    best = t;
                    hit = -1;
                }
            }
        }
        for (id, b) in self.bodies.iter().enumerate() {
            if !b.active || id as i32 == ignore {
                continue;
            }
            if let Some(t) = ray_sphere(o, d, b.pos, b.radius) {
                if t < best {
                    best = t;
                    hit = id as i32;
                }
            }
        }
        (best, hit)
    }
}

fn normalize(v: [f32; 3]) -> [f32; 3] {
    let l = (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt().max(1e-9);
    [v[0] / l, v[1] / l, v[2] / l]
}

fn ray_aabb(o: [f32; 3], d: [f32; 3], b: &Aabb) -> Option<f32> {
    let mut tmin = 0.0f32;
    let mut tmax = f32::INFINITY;
    for k in 0..3 {
        if d[k].abs() < 1e-9 {
            if o[k] < b.min[k] || o[k] > b.max[k] {
                return None;
            }
        } else {
            let inv = 1.0 / d[k];
            let (mut t1, mut t2) = ((b.min[k] - o[k]) * inv, (b.max[k] - o[k]) * inv);
            if t1 > t2 {
                core::mem::swap(&mut t1, &mut t2);
            }
            tmin = tmin.max(t1);
            tmax = tmax.min(t2);
            if tmin > tmax {
                return None;
            }
        }
    }
    Some(tmin)
}

fn ray_sphere(o: [f32; 3], d: [f32; 3], c: [f32; 3], r: f32) -> Option<f32> {
    let oc = [o[0] - c[0], o[1] - c[1], o[2] - c[2]];
    let b = oc[0] * d[0] + oc[1] * d[1] + oc[2] * d[2];
    let cc = oc[0] * oc[0] + oc[1] * oc[1] + oc[2] * oc[2] - r * r;
    let disc = b * b - cc;
    if disc < 0.0 {
        return None;
    }
    let s = disc.sqrt();
    let t = -b - s;
    if t >= 0.0 {
        Some(t)
    } else if -b + s >= 0.0 {
        Some(0.0)
    } else {
        None
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn projectile_hits_robot_in_its_path() {
        let mut w = World::new(5.0);
        let target = w.add_body(0.15);
        let shooter = w.add_body(0.15);
        w.set_body(target, 0.0, 0.2, 2.0, true);
        w.set_body(shooter, 0.0, 0.2, 0.0, true);
        w.spawn_projectile(0.0, 0.25, 0.3, 0.0, 0.5, 9.0, 0.03, 0.0, shooter as i32, 25.0, 1);
        let mut hit = false;
        for _ in 0..120 {
            let ev = w.step(1.0 / 60.0);
            if ev.chunks(8).any(|e| e[0] == EVENT_HIT_BODY && e[1] == target as f32) {
                hit = true;
                break;
            }
        }
        assert!(hit);
    }

    #[test]
    fn raycast_stops_at_obstacle_before_robot() {
        let mut w = World::new(5.0);
        let target = w.add_body(0.15);
        w.set_body(target, 0.0, 0.2, 3.0, true);
        w.add_obstacle(-0.5, 0.0, 1.0, 0.5, 1.0, 1.2);
        let r = w.raycast(0.0, 0.2, 0.0, 0.0, 0.0, 1.0, 10.0, -1);
        assert!((r[0] - 1.0).abs() < 1e-4);
        assert_eq!(r[1], -1.0);
    }

    #[test]
    fn ballistic_pitch_lands_on_target() {
        let v = 8.0;
        let pitch = World::ballistic_pitch(4.0, 0.0, v);
        let flight = 4.0 / (v * pitch.cos());
        let y = v * pitch.sin() * flight - 0.5 * 9.81 * flight * flight;
        assert!(y.abs() < 1e-3);
    }

    #[test]
    fn robots_are_pushed_out_of_obstacles() {
        let mut w = World::new(5.0);
        let id = w.add_body(0.2);
        w.add_obstacle(-0.5, 0.0, -0.5, 0.5, 1.0, 0.5);
        let p = w.resolve_body(id, 0.6, 0.0, 0.2);
        assert!(p[0] >= 0.7 - 1e-4);
    }
}
