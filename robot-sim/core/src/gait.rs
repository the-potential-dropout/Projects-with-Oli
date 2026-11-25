//! Tripod gait for a six-legged robot.
//!
//! Leg order (looking down, robot facing +Z):
//!   0 front-right, 1 middle-right, 2 rear-right,
//!   3 front-left,  4 middle-left,  5 rear-left
//! Tripod A = {0, 2, 4}, tripod B = {1, 3, 5}. While one tripod is in the air
//! the other three feet push the body along.
//!
//! "Right" is -X in three.js when the robot faces +Z, so right-side legs have
//! negative X mount positions.

use crate::ik::solve_leg_limited;
use wasm_bindgen::prelude::*;

const LEGS: usize = 6;
const TRIPOD_A: [bool; LEGS] = [true, false, true, false, true, false];

#[wasm_bindgen]
pub struct HexapodGait {
    coxa: f32,
    femur: f32,
    tibia: f32,
    mount: [[f32; 2]; LEGS], // (x, z) of each coxa joint in the body frame
    mount_yaw: [f32; LEGS],  // direction each leg points, radians about +Y
    neutral: [[f32; 2]; LEGS],
    body_height: f32,
    step_height: f32,
    step_hz: f32,
    phase: f32,
    activity: f32, // 0 = standing still, 1 = walking
    vel: [f32; 3], // smoothed (vx, vz, yaw rate) in body frame
    feet: [f32; LEGS * 3],
    angles: [f32; LEGS * 3],
    clamped: u32,
    min_tibia: f32, // most the tibia may fold (radians, negative)
}

#[wasm_bindgen]
impl HexapodGait {
    /// `mounts` = [x, z, yaw] * 6 for the coxa joints in the body frame.
    /// `foot_reach` = horizontal distance from the coxa joint to the foot at rest.
    #[wasm_bindgen(constructor)]
    pub fn new(mounts: &[f32], coxa: f32, femur: f32, tibia: f32, foot_reach: f32, body_height: f32) -> HexapodGait {
        let mut mount = [[0.0; 2]; LEGS];
        let mut mount_yaw = [0.0; LEGS];
        let mut neutral = [[0.0; 2]; LEGS];
        for i in 0..LEGS {
            let (x, z, yaw) = (mounts[i * 3], mounts[i * 3 + 1], mounts[i * 3 + 2]);
            mount[i] = [x, z];
            mount_yaw[i] = yaw;
            neutral[i] = [x + foot_reach * yaw.cos(), z + foot_reach * yaw.sin()];
        }
        let mut g = HexapodGait {
            coxa,
            femur,
            tibia,
            mount,
            mount_yaw,
            neutral,
            body_height,
            step_height: body_height * 0.45,
            step_hz: 1.6,
            phase: 0.0,
            activity: 0.0,
            vel: [0.0; 3],
            feet: [0.0; LEGS * 3],
            angles: [0.0; LEGS * 3],
            clamped: 0,
            min_tibia: -core::f32::consts::PI,
        };
        g.update(0.0, 0.0, 0.0, 0.0);
        g
    }

    pub fn set_body_height(&mut self, h: f32) {
        self.body_height = h;
    }

    /// Joint limit from the CAD: the tibia may not fold further than this
    /// (radians, relative to the femur; Mk1 = -135 deg).
    pub fn set_tibia_limit(&mut self, min_tibia: f32) {
        self.min_tibia = min_tibia;
    }

    pub fn set_step(&mut self, step_height: f32, step_hz: f32) {
        self.step_height = step_height;
        self.step_hz = step_hz.max(0.1);
    }

    /// Advance the gait. `vx`, `vz` are the commanded body velocities (m/s, body
    /// frame, +Z forward, +X left) and `yaw_rate` is rad/s (+ = turn left).
    pub fn update(&mut self, dt: f32, vx: f32, vz: f32, yaw_rate: f32) {
        // Smooth the command so the feet don't snap when input changes.
        let k = 1.0 - (-dt * 8.0).exp();
        self.vel[0] += (vx - self.vel[0]) * k;
        self.vel[1] += (vz - self.vel[1]) * k;
        self.vel[2] += (yaw_rate - self.vel[2]) * k;

        let moving = self.vel[0].abs() + self.vel[1].abs() + self.vel[2].abs() * 0.15 > 0.004;
        let target_activity = if moving { 1.0 } else { 0.0 };
        self.activity += (target_activity - self.activity) * (1.0 - (-dt * 5.0).exp());

        // Keep stepping while walking, and let an in-progress step finish
        // when stopping so feet settle back on the ground.
        let next = self.phase + dt * self.step_hz;
        if self.activity > 0.02 {
            self.phase = next % 1.0;
        } else {
            // Finish the current half-cycle, then park on the boundary (all six
            // feet down) until the robot moves again.
            let at_rest = (self.phase * 2.0).fract() < 1e-5;
            if !at_rest {
                if (next * 2.0).floor() > (self.phase * 2.0).floor() {
                    self.phase = ((next * 2.0).floor() * 0.5) % 1.0;
                } else {
                    self.phase = next;
                }
            }
        }

        // Time each foot spends on the ground per cycle.
        let stance_time = 0.5 / self.step_hz;
        self.clamped = 0;

        for i in 0..LEGS {
            let [nx, nz] = self.neutral[i];
            // Velocity of the body at this foot = v + w x r (w about +Y).
            // During stance the planted foot slides the opposite way.
            let dvx = self.vel[0] + self.vel[2] * nz;
            let dvz = self.vel[1] - self.vel[2] * nx;
            let half_x = dvx * stance_time * 0.5;
            let half_z = dvz * stance_time * 0.5;

            let local = if TRIPOD_A[i] { self.phase } else { (self.phase + 0.5) % 1.0 };
            let (s, lift) = if local < 0.5 {
                // swing: move from back (-half) to front (+half) through the air
                let u = local / 0.5;
                let ease = u * u * (3.0 - 2.0 * u);
                (ease * 2.0 - 1.0, (core::f32::consts::PI * u).sin())
            } else {
                // stance: foot planted, slides from front to back relative to body
                let u = (local - 0.5) / 0.5;
                (1.0 - 2.0 * u, 0.0)
            };

            let fx = nx + half_x * s;
            let fz = nz + half_z * s;
            let fy = -self.body_height + lift * self.step_height * self.activity.min(1.0);

            self.feet[i * 3] = fx;
            self.feet[i * 3 + 1] = fy;
            self.feet[i * 3 + 2] = fz;

            // Body frame -> leg frame (rotate by -mount_yaw about +Y)
            let (mx, mz) = (self.mount[i][0], self.mount[i][1]);
            let (dx, dz) = (fx - mx, fz - mz);
            let (c, sn) = (self.mount_yaw[i].cos(), self.mount_yaw[i].sin());
            let lx = dx * c + dz * sn;
            let lz = -dx * sn + dz * c;
            let a = solve_leg_limited(lx, fy, lz, self.coxa, self.femur, self.tibia, self.min_tibia);
            if a.clamped {
                self.clamped += 1;
            }
            self.angles[i * 3] = a.coxa;
            self.angles[i * 3 + 1] = a.femur;
            self.angles[i * 3 + 2] = a.tibia;
        }
    }

    /// [coxa, femur, tibia] * 6, radians.
    pub fn angles(&self) -> Vec<f32> {
        self.angles.to_vec()
    }

    /// Foot positions in the body frame, [x, y, z] * 6.
    pub fn feet(&self) -> Vec<f32> {
        self.feet.to_vec()
    }

    pub fn phase(&self) -> f32 {
        self.phase
    }

    /// Body bob (m) to make walking look alive: small dip at each step change.
    pub fn bob(&self) -> f32 {
        let p = (self.phase * 2.0) % 1.0;
        -(core::f32::consts::PI * 2.0 * p).cos().abs() * 0.004 * self.activity
    }

    /// Number of legs whose last target was out of reach (useful for tuning).
    pub fn clamped_legs(&self) -> u32 {
        self.clamped
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn test_gait() -> HexapodGait {
        let mut mounts = Vec::new();
        for (x, z) in [(-0.06, 0.09), (-0.075, 0.0), (-0.06, -0.09), (0.06, 0.09), (0.075, 0.0), (0.06, -0.09)] {
            let yaw = (z as f32).atan2(x as f32);
            mounts.extend_from_slice(&[x, z, yaw]);
        }
        HexapodGait::new(&mounts, 0.03, 0.06, 0.09, 0.11, 0.07)
    }

    #[test]
    fn standing_still_keeps_all_feet_on_ground_and_reachable() {
        let mut g = test_gait();
        for _ in 0..120 {
            g.update(1.0 / 60.0, 0.0, 0.0, 0.0);
        }
        let feet = g.feet();
        for i in 0..6 {
            assert!((feet[i * 3 + 1] + 0.07).abs() < 1e-3);
        }
        assert_eq!(g.clamped_legs(), 0);
    }

    #[test]
    fn walking_lifts_exactly_one_tripod_at_a_time() {
        let mut g = test_gait();
        for _ in 0..200 {
            g.update(1.0 / 60.0, 0.0, 0.15, 0.0);
            let f = g.feet();
            let up: Vec<bool> = (0..6).map(|i| f[i * 3 + 1] > -0.0695).collect();
            let a_up = up[0] || up[2] || up[4];
            let b_up = up[1] || up[3] || up[5];
            assert!(!(a_up && b_up), "both tripods in the air");
        }
        assert_eq!(g.clamped_legs(), 0);
    }

    #[test]
    fn sprint_strafe_respects_tibia_limit() {
        // Mk1 numbers: strafing at a sprint used to fold the tibia to about -148 deg
        let mut mounts = Vec::new();
        for (x, z, splay) in [(-0.06f32, 0.11f32, 35f32), (-0.08, 0.0, 0.0), (-0.06, -0.11, -35.0), (0.06, 0.11, 35.0), (0.08, 0.0, 0.0), (0.06, -0.11, -35.0)] {
            let yaw = if x < 0.0 { core::f32::consts::PI - splay.to_radians() } else { splay.to_radians() };
            mounts.extend_from_slice(&[x, z, yaw]);
        }
        let limit = -135f32.to_radians();
        let mut g = HexapodGait::new(&mounts, 0.045, 0.075, 0.12, 0.13, 0.09);
        g.set_tibia_limit(limit);
        g.set_step(0.045, 2.6);
        for _ in 0..600 {
            g.update(1.0 / 60.0, 0.48, 0.0, 0.0);
            let a = g.angles();
            for i in 0..6 {
                assert!(a[i * 3 + 2] >= limit - 1e-3, "tibia {} deg", a[i * 3 + 2].to_degrees());
            }
        }
    }
}
