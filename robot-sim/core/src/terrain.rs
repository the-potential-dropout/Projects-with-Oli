//! Uneven ground for the arena, and where a robot's body ends up standing on it.
//!
//! `Terrain` is a height field (bilinear between samples). `fit_body` finds the
//! body height, roll and pitch that put the supporting feet on the ground: the
//! robot is rigid with its legs at the commanded joint angles, so on uneven
//! ground without terrain adaptation it tilts and rocks exactly like the real one.

use crate::gait::{rotate, Pose};
use wasm_bindgen::prelude::*;

#[derive(Clone, Default)]
pub struct Grid {
    pub n: usize,
    pub cell: f32,
    pub half: f32,
    pub h: Vec<f32>,
}

impl Grid {
    pub fn height(&self, x: f32, z: f32) -> f32 {
        if self.n < 2 {
            return 0.0;
        }
        let max = (self.n - 1) as f32;
        let gx = ((x + self.half) / self.cell).clamp(0.0, max - 1e-4);
        let gz = ((z + self.half) / self.cell).clamp(0.0, max - 1e-4);
        let (ix, iz) = (gx.floor() as usize, gz.floor() as usize);
        let (fx, fz) = (gx - ix as f32, gz - iz as f32);
        let at = |i: usize, j: usize| self.h[j * self.n + i];
        let a = at(ix, iz) * (1.0 - fx) + at(ix + 1, iz) * fx;
        let b = at(ix, iz + 1) * (1.0 - fx) + at(ix + 1, iz + 1) * fx;
        a * (1.0 - fz) + b * fz
    }
}

fn hash(i: i32, j: i32, seed: u32) -> f32 {
    let mut h = (i as u32).wrapping_mul(374761393) ^ (j as u32).wrapping_mul(668265263) ^ seed.wrapping_mul(2246822519);
    h = (h ^ (h >> 13)).wrapping_mul(1274126177);
    h ^= h >> 16;
    (h & 0xffff) as f32 / 65535.0
}

fn value_noise(x: f32, z: f32, seed: u32) -> f32 {
    let (ix, iz) = (x.floor() as i32, z.floor() as i32);
    let (fx, fz) = (x - ix as f32, z - iz as f32);
    let s = |t: f32| t * t * (3.0 - 2.0 * t);
    let (u, v) = (s(fx), s(fz));
    let a = hash(ix, iz, seed) * (1.0 - u) + hash(ix + 1, iz, seed) * u;
    let b = hash(ix, iz + 1, seed) * (1.0 - u) + hash(ix + 1, iz + 1, seed) * u;
    (a * (1.0 - v) + b * v) * 2.0 - 1.0
}

#[wasm_bindgen]
pub struct Terrain {
    grid: Grid,
}

#[wasm_bindgen]
impl Terrain {
    /// Square arena of +-`half` metres with samples every `cell` metres, flat.
    #[wasm_bindgen(constructor)]
    pub fn new(half: f32, cell: f32) -> Terrain {
        let n = ((2.0 * half / cell).round() as usize) + 1;
        Terrain { grid: Grid { n, cell, half, h: vec![0.0; n * n] } }
    }

    pub(crate) fn grid(&self) -> &Grid {
        &self.grid
    }

    pub fn flatten(&mut self) {
        self.grid.h.iter_mut().for_each(|v| *v = 0.0);
    }

    /// Rough ground: rolling bumps, a stepped pad, a ramp and rocks. Keeps a
    /// flat apron of `keep_flat` metres round the two spawn points (0, +-spawn_z)
    /// and along the walls.
    pub fn generate_rough(&mut self, seed: u32, amplitude: f32, spawn_z: f32, keep_flat: f32) {
        let g = &mut self.grid;
        let half = g.half;
        for j in 0..g.n {
            for i in 0..g.n {
                let x = -half + i as f32 * g.cell;
                let z = -half + j as f32 * g.cell;
                // rolling ground: two octaves of smooth noise
                let mut h = value_noise(x * 1.1, z * 1.1, seed) * 0.65 + value_noise(x * 3.3, z * 3.3, seed + 7) * 0.35;
                h *= amplitude;
                // a gentle hill with a 12 deg flank on the left side
                let (hx, hz) = (x - 1.4, z + 0.2);
                let r = (hx * hx + hz * hz).sqrt();
                h += (0.07 - r * 12f32.to_radians().tan()).max(0.0);
                // stepped pad on the right: 3 terraces of 20 mm, 0.2 m deep (within
                // the leg's reach in terrain mode, see docs/CONTROL.md §8)
                let (px, pz) = (x + 1.5, z - 0.3);
                let d = px.abs().max(pz.abs());
                if d < 0.6 {
                    h += 0.02 * (((0.6 - d) / 0.2).floor() + 1.0).min(3.0);
                }
                // scattered rocks (low cones)
                for k in 0..10 {
                    let rx = (hash(k, 1, seed) * 2.0 - 1.0) * (half - 0.6);
                    let rz = (hash(k, 2, seed) * 2.0 - 1.0) * (half - 0.6);
                    let rr = 0.08 + hash(k, 3, seed) * 0.1;
                    let dd = ((x - rx).powi(2) + (z - rz).powi(2)).sqrt();
                    if dd < rr {
                        h += 0.035 * (1.0 - dd / rr);
                    }
                }
                // fade to flat round the spawns and the walls
                let ds = (x * x + (z.abs() - spawn_z).powi(2)).sqrt();
                let dw = half - x.abs().max(z.abs());
                let f = ((ds - keep_flat) / 0.35).clamp(0.0, 1.0) * ((dw - 0.15) / 0.35).clamp(0.0, 1.0);
                g.h[j * g.n + i] = h * f;
            }
        }
    }

    pub fn height(&self, x: f32, z: f32) -> f32 {
        self.grid.height(x, z)
    }

    /// Heights under several points: [x, z] * n -> [y] * n.
    pub fn heights_at(&self, xz: &[f32]) -> Vec<f32> {
        xz.chunks(2).map(|p| self.grid.height(p[0], p[1])).collect()
    }

    pub fn samples(&self) -> usize {
        self.grid.n
    }

    pub fn cell(&self) -> f32 {
        self.grid.cell
    }

    /// All heights, row-major (z rows, x columns), for building the ground mesh.
    pub fn heights(&self) -> Vec<f32> {
        self.grid.h.clone()
    }

    /// Where the body sits. `feet_body` = [x, y, z] * 6 foot positions in the
    /// posed body frame, `feet_xz` = [x, z] * 6 world positions of the feet (from
    /// the last frame, for sampling the ground), `support` = bit mask of legs
    /// that may carry weight (stance), `yaw` = the pose yaw (rad).
    /// Returns [height of the body origin, roll, pitch, contact mask,
    /// lowest-foot clearance].
    #[allow(clippy::too_many_arguments)]
    pub fn fit_body(&self, feet_body: &[f32], feet_xz: &[f32], support: u32, yaw: f32, roll0: f32, pitch0: f32) -> Vec<f32> {
        let ground: Vec<f32> = (0..6).map(|i| self.grid.height(feet_xz[i * 2], feet_xz[i * 2 + 1])).collect();
        fit(feet_body, &ground, support, yaw, roll0, pitch0)
    }
}

/// Body height / roll / pitch so the supporting feet touch `ground` (least
/// squares, Gauss-Newton). Feet that end up clearly above the ground are
/// dropped from the support set (the body rocks onto the others).
pub fn fit(feet_body: &[f32], ground: &[f32], support: u32, yaw: f32, roll0: f32, pitch0: f32) -> Vec<f32> {
    let foot_y = |i: usize, r: f32, p: f32| -> f32 {
        let pose = Pose { roll: r, pitch: p, yaw, shift_x: 0.0, shift_z: 0.0 };
        rotate(&pose, [feet_body[i * 3], feet_body[i * 3 + 1], feet_body[i * 3 + 2]])[1]
    };
    let mut set: Vec<usize> = (0..6).filter(|&i| support & (1 << i) != 0).collect();
    if set.len() < 3 {
        set = (0..6).collect();
    }
    let (mut h, mut r, mut p) = (0.0f32, roll0, pitch0);
    for _round in 0..6 {
        for _it in 0..4 {
            // residual_i = h + y_i(r, p) - ground_i
            let mut ata = [[0.0f32; 3]; 3];
            let mut atb = [0.0f32; 3];
            for &i in &set {
                let y = foot_y(i, r, p);
                let e = 1e-3;
                let dr = (foot_y(i, r + e, p) - y) / e;
                let dp = (foot_y(i, r, p + e) - y) / e;
                let j = [1.0, dr, dp];
                let res = h + y - ground[i];
                for a in 0..3 {
                    for b in 0..3 {
                        ata[a][b] += j[a] * j[b];
                    }
                    atb[a] -= j[a] * res;
                }
            }
            // light damping keeps 3-foot fits well conditioned
            for (a, row) in ata.iter_mut().enumerate() {
                row[a] += 1e-6;
            }
            if let Some(d) = solve3(ata, atb) {
                h += d[0];
                r = (r + d[1]).clamp(-0.8, 0.8);
                p = (p + d[2]).clamp(-0.8, 0.8);
            }
        }
        // a foot that would sink into the ground carries weight: add the deepest one;
        // otherwise a supporting foot left hanging carries nothing: drop the highest
        // (keep at least 3), then refit
        let mut deepest = None;
        let mut deep_gap = -0.002;
        for i in (0..6).filter(|i| !set.contains(i)) {
            let gap = h + foot_y(i, r, p) - ground[i];
            if gap < deep_gap {
                deep_gap = gap;
                deepest = Some(i);
            }
        }
        if let Some(d) = deepest {
            set.push(d);
            continue;
        }
        let mut worst = None;
        let mut worst_gap = 0.004;
        for &i in &set {
            let gap = h + foot_y(i, r, p) - ground[i];
            if gap > worst_gap {
                worst_gap = gap;
                worst = Some(i);
            }
        }
        match worst {
            Some(w) if set.len() > 3 => set.retain(|&i| i != w),
            _ => break,
        }
    }
    // no foot may sink into the ground: lift the body onto the lowest one
    let mut lowest = f32::MAX;
    for i in 0..6 {
        lowest = lowest.min(h + foot_y(i, r, p) - ground[i]);
    }
    if lowest < -0.002 {
        h -= lowest + 0.002;
    }
    // the supporting feet carry the body; any other foot within 4 mm touches too
    let mut contact = set.iter().fold(0u32, |m, &i| m | (1 << i));
    for i in 0..6 {
        if h + foot_y(i, r, p) - ground[i] < 0.004 {
            contact |= 1 << i;
        }
    }
    vec![h, r, p, contact as f32, lowest]
}

fn solve3(a: [[f32; 3]; 3], b: [f32; 3]) -> Option<[f32; 3]> {
    let det = |m: [[f32; 3]; 3]| {
        m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1]) - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0]) + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0])
    };
    let d = det(a);
    if d.abs() < 1e-12 {
        return None;
    }
    let mut out = [0.0; 3];
    for (k, o) in out.iter_mut().enumerate() {
        let mut m = a;
        for row in 0..3 {
            m[row][k] = b[row];
        }
        *o = det(m) / d;
    }
    Some(out)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn flat_feet(h: f32) -> Vec<f32> {
        let mut f = Vec::new();
        for (x, z) in [(-0.15f32, 0.17f32), (-0.18, 0.0), (-0.15, -0.17), (0.15, 0.17), (0.18, 0.0), (0.15, -0.17)] {
            f.extend_from_slice(&[x, -h, z]);
        }
        f
    }

    #[test]
    fn flat_ground_gives_stand_height_and_level_body() {
        let r = fit(&flat_feet(0.09), &[0.0; 6], 0b111111, 0.0, 0.0, 0.0);
        assert!((r[0] - 0.09).abs() < 1e-4 && r[1].abs() < 1e-4 && r[2].abs() < 1e-4);
        assert_eq!(r[3] as u32, 0b111111);
    }

    #[test]
    fn slope_under_the_feet_tilts_the_body() {
        // ground rising 10 deg toward +X (the robot's left): left side goes up = + roll
        let feet = flat_feet(0.09);
        let g: Vec<f32> = (0..6).map(|i| feet[i * 3] * 10f32.to_radians().tan()).collect();
        let r = fit(&feet, &g, 0b111111, 0.0, 0.0, 0.0);
        assert!((r[1].to_degrees() - 10.0).abs() < 0.3, "roll {}", r[1].to_degrees());
        assert!(r[2].abs() < 0.01);
    }

    #[test]
    fn a_bump_under_one_foot_rocks_the_tripod() {
        let mut feet = flat_feet(0.09);
        for i in [1, 3, 5] {
            feet[i * 3 + 1] += 0.03; // tripod B in the air
        }
        let mut g = [0.0f32; 6];
        g[0] = 0.03; // front-right foot on a 30 mm rock
        let r = fit(&feet, &g, 0b010101, 0.0, 0.0, 0.0);
        // tripod A (0, 2, 4) stands on it: front-right corner up -> nose up, right side up (roll -)
        assert!(r[2] > 0.02 && r[1] < -0.02, "roll {} pitch {}", r[1], r[2]);
        assert_eq!(r[3] as u32 & 0b010101, 0b010101);
    }

    #[test]
    fn rough_terrain_keeps_spawns_flat() {
        let mut t = Terrain::new(3.0, 0.03);
        t.generate_rough(7, 0.02, 2.5, 0.45);
        assert!(t.height(0.0, 2.5).abs() < 1e-4 && t.height(0.0, -2.5).abs() < 1e-4);
        let max = t.heights().into_iter().fold(0.0f32, f32::max);
        assert!(max > 0.04 && max < 0.2, "max {max}");
    }
}
