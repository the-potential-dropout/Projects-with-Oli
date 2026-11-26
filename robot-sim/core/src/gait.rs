//! Gait engine, body pose and terrain adaptation for a six-legged robot.
//!
//! This is a line-for-line mirror of the robot's own firmware
//! (`firmware/hexapod_controller/kinematics.cpp`, docs/CONTROL.md §5, §7, §8),
//! so the simulator and the real robot compute the same joint angles.
//!
//! Leg order (looking down, robot facing +Z):
//!   0 front-right, 1 middle-right, 2 rear-right,
//!   3 front-left,  4 middle-left,  5 rear-left
//!
//! Gaits: 0 tripod (3 feet always down), 1 ripple (4), 2 wave (5).
//!
//! Frames: the *level frame* is attached to the body origin (coxa-joint height)
//! but stays level with the world; the *body frame* is tilted by the pose.
//! Pose rotation R = Ry(yaw)·Rx(pitch)·Rz(roll): + roll = left side up,
//! + pitch = nose up, + yaw = turn left.

use crate::ik::solve_leg_limited;
use wasm_bindgen::prelude::*;

const LEGS: usize = 6;
const PI: f32 = core::f32::consts::PI;

const OFFS: [[f32; LEGS]; 3] = [
    [0.0, 0.5, 0.0, 0.5, 0.0, 0.5],                                     // tripod
    [2.0 / 3.0, 1.0 / 3.0, 0.0, 1.0 / 6.0, 5.0 / 6.0, 0.5],             // ripple
    [2.0 / 6.0, 1.0 / 6.0, 0.0, 5.0 / 6.0, 4.0 / 6.0, 3.0 / 6.0],       // wave
];
const SWING: [f32; 3] = [0.5, 1.0 / 3.0, 1.0 / 6.0];

#[derive(Clone, Copy, Default)]
pub struct Pose {
    pub roll: f32,
    pub pitch: f32,
    pub yaw: f32,
    pub shift_x: f32,
    pub shift_z: f32,
}

/// Rotate a body-frame vector into the level frame (R · v).
pub fn rotate(p: &Pose, v: [f32; 3]) -> [f32; 3] {
    let (cr, sr, cp, sp, cy, sy) = (p.roll.cos(), p.roll.sin(), p.pitch.cos(), p.pitch.sin(), p.yaw.cos(), p.yaw.sin());
    let a = [cr * v[0] - sr * v[1], sr * v[0] + cr * v[1], v[2]];
    let b = [a[0], cp * a[1] + sp * a[2], -sp * a[1] + cp * a[2]];
    [cy * b[0] + sy * b[2], b[1], -sy * b[0] + cy * b[2]]
}

/// Level frame -> body frame (Rᵀ · v).
pub fn rotate_inv(p: &Pose, v: [f32; 3]) -> [f32; 3] {
    let (cr, sr, cp, sp, cy, sy) = (p.roll.cos(), p.roll.sin(), p.pitch.cos(), p.pitch.sin(), p.yaw.cos(), p.yaw.sin());
    let b = [cy * v[0] - sy * v[2], v[1], sy * v[0] + cy * v[2]];
    let a = [b[0], cp * b[1] - sp * b[2], sp * b[1] + cp * b[2]];
    [cr * a[0] + sr * a[1], -sr * a[0] + cr * a[1], a[2]]
}

/// Static stability margin: signed distance from (com_x, com_z) to the edges of
/// the convex polygon through `feet` ([x, z] pairs). Negative = outside.
pub fn stability_margin(feet: &[[f32; 2]], com_x: f32, com_z: f32) -> f32 {
    let n = feet.len();
    if n < 3 {
        return -1.0;
    }
    let (mut cx, mut cz) = (0.0, 0.0);
    for f in feet {
        cx += f[0];
        cz += f[1];
    }
    cx /= n as f32;
    cz /= n as f32;
    let mut idx: Vec<usize> = (0..n).collect();
    idx.sort_by(|&a, &b| {
        let aa = (feet[a][1] - cz).atan2(feet[a][0] - cx);
        let ab = (feet[b][1] - cz).atan2(feet[b][0] - cx);
        aa.partial_cmp(&ab).unwrap_or(core::cmp::Ordering::Equal)
    });
    let mut m = f32::MAX;
    for k in 0..n {
        let a = feet[idx[k]];
        let b = feet[idx[(k + 1) % n]];
        let (ex, ez) = (b[0] - a[0], b[1] - a[1]);
        let l = (ex * ex + ez * ez).sqrt();
        if l < 1e-6 {
            continue;
        }
        m = m.min((ex * (com_z - a[1]) - ez * (com_x - a[0])) / l);
    }
    m
}

#[wasm_bindgen]
pub struct HexapodGait {
    coxa: f32,
    femur: f32,
    tibia: f32,
    mount: [[f32; 2]; LEGS],
    mount_yaw: [f32; LEGS],
    neutral: [[f32; 2]; LEGS],
    foot_reach: f32,
    // joint limits (radians, sim convention)
    lim_coxa: [f32; 2],
    lim_femur: [f32; 2],
    lim_tibia: [f32; 2],
    tibia_fold_femur_down: f32,
    min_reach: f32, // femur-axis -> foot distance with the tibia at its fold limit
    // stance-window rule
    reach_min: f32,
    reach_max: f32,
    sprint_hz: f32,
    // command
    body_height: f32,
    step_height: f32,
    step_hz: f32,
    pose: Pose,
    attitude: Option<[f32; 2]>, // measured world roll/pitch (IMU), for terrain learning
    want_gait: usize,
    terrain: bool,
    contact: [bool; LEGS],
    // state
    gait: usize,
    phase: f32,
    activity: f32,
    vel: [f32; 3],
    ground: [f32; LEGS],
    stance: [bool; LEGS],
    feet: [f32; LEGS * 3],
    feet_body: [f32; LEGS * 3],
    angles: [f32; LEGS * 3],
    clamped: u32,
    speed_scale: f32,
}

#[wasm_bindgen]
impl HexapodGait {
    /// `mounts` = [x, z, yaw] * 6 for the coxa joints in the body frame.
    /// `foot_reach` = horizontal distance from the coxa joint to the neutral foot.
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
            foot_reach,
            lim_coxa: [-PI, PI],
            lim_femur: [-PI, PI],
            lim_tibia: [-PI, PI],
            tibia_fold_femur_down: -PI,
            min_reach: 0.0,
            reach_min: 0.0,
            reach_max: f32::MAX,
            sprint_hz: 1e3,
            body_height,
            step_height: body_height * 0.3,
            step_hz: 1.6,
            pose: Pose::default(),
            attitude: None,
            want_gait: 0,
            terrain: false,
            contact: [true; LEGS],
            gait: 0,
            phase: 0.0,
            activity: 0.0,
            vel: [0.0; 3],
            ground: [0.0; LEGS],
            stance: [true; LEGS],
            feet: [0.0; LEGS * 3],
            feet_body: [0.0; LEGS * 3],
            angles: [0.0; LEGS * 3],
            clamped: 0,
            speed_scale: 1.0,
        };
        g.update_min_reach();
        g.update(0.0, 0.0, 0.0, 0.0);
        g
    }

    fn update_min_reach(&mut self) {
        let (f, t) = (self.femur, self.tibia);
        let k = (PI + self.lim_tibia[0]).max(0.0);
        self.min_reach = (f * f + t * t - 2.0 * f * t * k.cos()).max(0.0).sqrt() + 2e-4;
    }

    pub fn set_body_height(&mut self, h: f32) {
        self.body_height = h;
    }

    /// Joint limit from the CAD: the tibia may not fold further than this (radians).
    pub fn set_tibia_limit(&mut self, min_tibia: f32) {
        self.lim_tibia[0] = min_tibia;
        self.tibia_fold_femur_down = self.tibia_fold_femur_down.max(min_tibia);
        self.update_min_reach();
    }

    /// All joint limits (radians). `tibia_fold_femur_down` is the coupled limit
    /// that applies while the femur points below level.
    #[allow(clippy::too_many_arguments)]
    pub fn set_joint_limits(&mut self, coxa_min: f32, coxa_max: f32, femur_min: f32, femur_max: f32, tibia_min: f32, tibia_max: f32, tibia_fold_femur_down: f32) {
        self.lim_coxa = [coxa_min, coxa_max];
        self.lim_femur = [femur_min, femur_max];
        self.lim_tibia = [tibia_min, tibia_max];
        self.tibia_fold_femur_down = tibia_fold_femur_down;
        self.update_min_reach();
    }

    /// Stance-window rule (load check): stance feet stay between `min` and `max`
    /// from the coxa axis; the gait steps faster (up to `sprint_hz`), then slows down.
    pub fn set_reach_window(&mut self, min: f32, max: f32, sprint_hz: f32) {
        self.reach_min = min;
        self.reach_max = max;
        self.sprint_hz = sprint_hz;
    }

    pub fn set_step(&mut self, step_height: f32, step_hz: f32) {
        self.step_height = step_height;
        self.step_hz = step_hz.max(0.1);
    }

    /// Requested gait: 0 tripod, 1 ripple, 2 wave. It takes effect at the next
    /// phase boundary.
    pub fn set_gait(&mut self, kind: u32) {
        self.want_gait = (kind as usize).min(2);
    }

    /// Body pose in radians / metres (after the balance controller).
    pub fn set_pose(&mut self, roll: f32, pitch: f32, yaw: f32, shift_x: f32, shift_z: f32) {
        self.pose = Pose { roll, pitch, yaw, shift_x, shift_z };
    }

    /// Measured body roll / pitch (IMU, rad). Touchdown heights are then taken in
    /// the true level frame: without it, a body that is not where it was told to
    /// be learns its own tilt into the ground offsets (front feet "find" higher
    /// ground, rear feet search lower) and the tilt grows.
    pub fn set_attitude(&mut self, roll: f32, pitch: f32) {
        self.attitude = Some([roll, pitch]);
    }

    /// Terrain mode: touchdown search and per-leg ground offsets.
    pub fn set_terrain(&mut self, on: bool) {
        self.terrain = on;
    }

    /// Foot switches, bit i = leg i touching the ground.
    pub fn set_contacts(&mut self, mask: u32) {
        for i in 0..LEGS {
            self.contact[i] = mask & (1 << i) != 0;
        }
    }

    /// Height of leg i's foot below the body origin in the true level frame.
    fn touchdown_height(&self, i: usize) -> f32 {
        match self.attitude {
            Some([r, p]) => {
                let actual = Pose { roll: r, pitch: p, yaw: self.pose.yaw, shift_x: 0.0, shift_z: 0.0 };
                let fb = [self.feet_body[i * 3], self.feet_body[i * 3 + 1], self.feet_body[i * 3 + 2]];
                crate::gait::rotate(&actual, fb)[1]
            }
            None => self.feet[i * 3 + 1],
        }
    }

    fn leg_phase(&self, leg: usize) -> f32 {
        let p = self.phase + OFFS[self.gait][leg];
        p - p.floor()
    }

    /// Advance the gait. `vx`, `vz` = body velocity (m/s, +Z forward, +X left),
    /// `yaw_rate` rad/s (+ = turn left).
    pub fn update(&mut self, dt: f32, vx: f32, vz: f32, yaw_rate: f32) {
        let k = 1.0 - (-dt * 8.0).exp();
        self.vel[0] += (vx - self.vel[0]) * k;
        self.vel[1] += (vz - self.vel[1]) * k;
        self.vel[2] += (yaw_rate - self.vel[2]) * k;
        let moving = self.vel[0].abs() + self.vel[1].abs() + self.vel[2].abs() * 0.15 > 0.004;
        let target = if moving { 1.0 } else { 0.0 };
        self.activity += (target - self.activity) * (1.0 - (-dt * 5.0).exp());

        let mut swing = SWING[self.gait];
        let mut step_hz = self.step_hz;
        let mut stance_time = (1.0 - swing) / step_hz;

        // stance-window rule
        let mut scale = 1.0f32;
        for pass in 0..2 {
            scale = 1.0;
            for i in 0..LEGS {
                let [nx, nz] = self.neutral[i];
                let hx = (self.vel[0] + self.vel[2] * nz) * stance_time * 0.5;
                let hz = (self.vel[1] - self.vel[2] * nx) * stance_time * 0.5;
                for sg in [-1.0f32, 1.0] {
                    let fx = nx + sg * hx - self.mount[i][0];
                    let fz = nz + sg * hz - self.mount[i][1];
                    let dr = (fx * fx + fz * fz).sqrt() - self.foot_reach;
                    let lim = if dr > 0.0 { self.reach_max - self.foot_reach } else { self.foot_reach - self.reach_min };
                    if dr.abs() > lim && dr.abs() > 1e-9 {
                        scale = scale.min(lim / dr.abs());
                    }
                }
            }
            if scale < 1.0 && pass == 0 && step_hz < self.sprint_hz {
                step_hz = self.sprint_hz.min(step_hz / scale.max(0.3));
                stance_time = (1.0 - swing) / step_hz;
            } else {
                break;
            }
        }
        self.speed_scale = scale;
        if scale < 1.0 {
            for v in self.vel.iter_mut() {
                *v *= scale;
            }
        }

        let next = self.phase + dt * step_hz;
        if self.activity > 0.02 {
            self.phase = next - next.floor();
        } else {
            let n = 1.0 / swing;
            let at_rest = (self.phase * n).rem_euclid(1.0) < 1e-5;
            if !at_rest {
                if (next * n).floor() > (self.phase * n).floor() {
                    self.phase = ((next * n).floor() / n).rem_euclid(1.0);
                } else {
                    self.phase = next;
                }
            } else if self.want_gait != self.gait {
                self.gait = self.want_gait;
            }
        }
        if self.activity > 0.02 && self.want_gait != self.gait && (self.phase / swing).rem_euclid(1.0) < dt * step_hz / swing {
            self.gait = self.want_gait;
        }
        swing = SWING[self.gait];
        stance_time = (1.0 - swing) / step_hz;

        self.clamped = 0;
        let mut mean_ground = 0.0;
        let mut n_stance = 0;
        let act = self.activity.min(1.0);
        for i in 0..LEGS {
            let [nx, nz] = self.neutral[i];
            let hx = (self.vel[0] + self.vel[2] * nz) * stance_time * 0.5;
            let hz = (self.vel[1] - self.vel[2] * nx) * stance_time * 0.5;
            let local = self.leg_phase(i);
            let in_swing = local < swing;
            let (s, mut lift);
            if in_swing {
                let u = local / swing;
                let ease = u * u * (3.0 - 2.0 * u);
                s = ease * 2.0 - 1.0;
                lift = (PI * u).sin();
                if self.terrain && u > 0.8 && self.contact[i] {
                    // early touchdown: this foot's ground is here
                    self.ground[i] = self.touchdown_height(i) + self.body_height;
                    lift = 0.0;
                }
            } else {
                let u = (local - swing) / (1.0 - swing);
                s = 1.0 - 2.0 * u;
                lift = 0.0;
                if self.terrain && !self.contact[i] && u < 0.5 {
                    // late touchdown: search down (bounded)
                    self.ground[i] = (self.ground[i] - TD_SEARCH_SPEED * dt).max(-TD_SEARCH);
                }
            }
            // a foot counts as down while it is in stance or not (yet) lifted
            self.stance[i] = !in_swing || lift * self.step_height * act < 0.001;
            if !self.terrain {
                self.ground[i] *= 1.0 - (dt * 2.0).min(1.0);
            }
            self.ground[i] = self.ground[i].clamp(-TD_SEARCH, STEP_UP_MAX);
            if self.stance[i] {
                mean_ground += self.ground[i];
                n_stance += 1;
            }

            let fx = nx + hx * s;
            let fy = -self.body_height + self.ground[i] + lift * self.step_height * act;
            let fz = nz + hz * s;
            self.feet[i * 3] = fx;
            self.feet[i * 3 + 1] = fy;
            self.feet[i * 3 + 2] = fz;

            // level frame -> posed body frame -> leg frame
            let mut pb = rotate_inv(&self.pose, [fx - self.pose.shift_x, fy, fz - self.pose.shift_z]);
            let (dx, dz) = (pb[0] - self.mount[i][0], pb[2] - self.mount[i][1]);
            let (c, sn) = (self.mount_yaw[i].cos(), self.mount_yaw[i].sin());
            let lx = dx * c + dz * sn;
            let lz = -dx * sn + dz * c;
            // lift cap: a foot raised close to the body would fold the tibia past its limit
            let horiz = (lx * lx + lz * lz).sqrt() - self.coxa;
            if horiz < self.min_reach {
                pb[1] = pb[1].min(-(self.min_reach * self.min_reach - horiz * horiz).max(0.0).sqrt());
            }
            self.feet_body[i * 3] = pb[0];
            self.feet_body[i * 3 + 1] = pb[1];
            self.feet_body[i * 3 + 2] = pb[2];
            let a = solve_leg_limited(lx, pb[1], lz, self.coxa, self.femur, self.tibia, self.lim_tibia[0]);
            let coxa = a.coxa.clamp(self.lim_coxa[0], self.lim_coxa[1]);
            let femur = a.femur.clamp(self.lim_femur[0], self.lim_femur[1]);
            let tmin = if femur < 0.0 { self.tibia_fold_femur_down } else { self.lim_tibia[0] };
            let tibia = a.tibia.clamp(tmin, self.lim_tibia[1]);
            if a.clamped || coxa != a.coxa || femur != a.femur || tibia != a.tibia {
                self.clamped |= 1 << i;
            }
            self.angles[i * 3] = coxa;
            self.angles[i * 3 + 1] = femur;
            self.angles[i * 3 + 2] = tibia;
        }
        // terrain following: re-centre the offsets so the body rides parallel to the ground
        if self.terrain && n_stance > 0 {
            mean_ground /= n_stance as f32;
            let d = mean_ground * (dt * 1.5).min(1.0);
            for g in self.ground.iter_mut() {
                *g -= d;
            }
        }
    }

    /// [coxa, femur, tibia] * 6, radians.
    pub fn angles(&self) -> Vec<f32> {
        self.angles.to_vec()
    }

    /// Foot targets in the level frame, [x, y, z] * 6.
    pub fn feet(&self) -> Vec<f32> {
        self.feet.to_vec()
    }

    /// Foot targets in the posed body frame, [x, y, z] * 6 (after the lift cap).
    pub fn feet_body(&self) -> Vec<f32> {
        self.feet_body.to_vec()
    }

    /// Per-leg ground offsets learned at touchdown (m, + = higher ground).
    pub fn ground_offsets(&self) -> Vec<f32> {
        self.ground.to_vec()
    }

    /// Bit i = leg i in stance.
    pub fn stance_mask(&self) -> u32 {
        (0..LEGS).filter(|&i| self.stance[i]).fold(0, |m, i| m | (1 << i))
    }

    pub fn phase(&self) -> f32 {
        self.phase
    }

    /// The gait actually running (may lag `set_gait` until a phase boundary).
    pub fn current_gait(&self) -> u32 {
        self.gait as u32
    }

    /// The body velocity the stance feet are actually pushing (vx, vz, yaw rate),
    /// after the command filter and the stance-window rule. Move the body by this
    /// and the planted feet don't slide.
    pub fn velocity(&self) -> Vec<f32> {
        self.vel.to_vec()
    }

    pub fn activity(&self) -> f32 {
        self.activity
    }

    /// < 1 when the stance-window rule is slowing the body down.
    pub fn speed_scale(&self) -> f32 {
        self.speed_scale
    }

    /// Static stability margin (m) of the stance feet in the level frame, for a
    /// centre of mass projected at (com_x, com_z).
    pub fn stability_margin(&self, com_x: f32, com_z: f32) -> f32 {
        let pts: Vec<[f32; 2]> = (0..LEGS).filter(|&i| self.stance[i]).map(|i| [self.feet[i * 3], self.feet[i * 3 + 2]]).collect();
        stability_margin(&pts, com_x, com_z)
    }

    /// Bit i = leg i's last target was out of reach or hit a joint limit.
    pub fn clamped_mask(&self) -> u32 {
        self.clamped
    }

    /// Number of legs whose last target was clamped.
    pub fn clamped_legs(&self) -> u32 {
        self.clamped.count_ones()
    }

    /// Body bob (m), unused while the body follows the feet.
    pub fn bob(&self) -> f32 {
        let p = (self.phase * 2.0) % 1.0;
        -(PI * 2.0 * p).cos().abs() * 0.004 * self.activity
    }
}

const TD_SEARCH_SPEED: f32 = 0.15; // m/s
const TD_SEARCH: f32 = 0.05; // step-down limit
const STEP_UP_MAX: f32 = 0.04;

/// Body levelling: PI on the IMU attitude error, D on the gyro rate, slew and
/// range limited. Same law as `hk::Balance` in the firmware (CONTROL.md §7).
#[wasm_bindgen]
pub struct Balance {
    kp: f32,
    ki: f32,
    kd: f32,
    i_max: f32,
    slew: f32,
    d_hz: f32,
    limit: f32,
    i: [f32; 2],
    d: [f32; 2],
    out: [f32; 2],
}

#[wasm_bindgen]
impl Balance {
    /// Gains as in hexapod.config.js `control.balance`; angles in degrees.
    #[wasm_bindgen(constructor)]
    pub fn new(kp: f32, ki: f32, kd: f32, i_max_deg: f32, slew_deg_s: f32, d_lowpass_hz: f32, limit_deg: f32) -> Balance {
        Balance {
            kp,
            ki,
            kd,
            i_max: i_max_deg.to_radians(),
            slew: slew_deg_s.to_radians(),
            d_hz: d_lowpass_hz,
            limit: limit_deg.to_radians(),
            i: [0.0; 2],
            d: [0.0; 2],
            out: [0.0; 2],
        }
    }

    pub fn reset(&mut self) {
        self.i = [0.0; 2];
        self.d = [0.0; 2];
        self.out = [0.0; 2];
    }

    /// target / measured roll and pitch (rad), gyro rates (rad/s).
    #[allow(clippy::too_many_arguments)]
    pub fn update(&mut self, dt: f32, t_roll: f32, t_pitch: f32, m_roll: f32, m_pitch: f32, r_roll: f32, r_pitch: f32) {
        if dt <= 0.0 {
            return;
        }
        let e = [t_roll - m_roll, t_pitch - m_pitch];
        let rate = [r_roll, r_pitch];
        let ib = self.i_max / self.ki.max(1e-3);
        let a = 1.0 - (-2.0 * PI * self.d_hz * dt).exp();
        for k in 0..2 {
            self.i[k] = (self.i[k] + e[k] * dt).clamp(-ib, ib);
            self.d[k] += (-rate[k] - self.d[k]) * a;
            let u = self.kp * e[k] + self.ki * self.i[k] + self.kd * self.d[k];
            let s = self.slew * dt;
            self.out[k] += (u - self.out[k]).clamp(-s, s);
            self.out[k] = self.out[k].clamp(-self.limit, self.limit);
        }
    }

    pub fn roll(&self) -> f32 {
        self.out[0]
    }

    pub fn pitch(&self) -> f32 {
        self.out[1]
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

    /// The real Mk1 numbers from hexapod.config.js.
    fn mk1() -> HexapodGait {
        let mut mounts = Vec::new();
        for (x, z, splay) in [(-0.06f32, 0.11f32, 35f32), (-0.08, 0.0, 0.0), (-0.06, -0.11, -35.0), (0.06, 0.11, 35.0), (0.08, 0.0, 0.0), (0.06, -0.11, -35.0)] {
            let yaw = if x < 0.0 { PI - splay.to_radians() } else { splay.to_radians() };
            mounts.extend_from_slice(&[x, z, yaw]);
        }
        let mut g = HexapodGait::new(&mounts, 0.045, 0.075, 0.12, 0.1, 0.09);
        let r = |d: f32| d.to_radians();
        g.set_joint_limits(r(-45.0), r(45.0), r(-10.0), r(70.0), r(-135.0), r(30.0), r(-132.0));
        g.set_reach_window(0.08, 0.115, 2.6);
        g.set_step(0.025, 1.7);
        g
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
            assert!(!((up[0] || up[2] || up[4]) && (up[1] || up[3] || up[5])), "both tripods in the air");
        }
        assert_eq!(g.clamped_legs(), 0);
    }

    #[test]
    fn ripple_and_wave_keep_four_and_five_feet_down() {
        for (kind, min_down) in [(1u32, 4u32), (2, 5)] {
            let mut g = mk1();
            g.set_gait(kind);
            for _ in 0..30 {
                g.update(1.0 / 100.0, 0.0, 0.0, 0.0);
            }
            assert_eq!(g.current_gait(), kind);
            for _ in 0..600 {
                g.update(1.0 / 100.0, 0.0, 0.15, 0.0);
                assert!(g.stance_mask().count_ones() >= min_down, "gait {kind}: {} down", g.stance_mask().count_ones());
            }
        }
    }

    #[test]
    fn sprint_turn_respects_joint_limits_and_stance_window() {
        let mut g = mk1();
        g.set_step(0.025, 2.6);
        let tmin = -135f32.to_radians();
        for _ in 0..600 {
            g.update(1.0 / 60.0, 0.3, 0.48, 1.6);
            let a = g.angles();
            let f = g.feet();
            for i in 0..6 {
                assert!(a[i * 3 + 2] >= tmin - 1e-3, "tibia {} deg", a[i * 3 + 2].to_degrees());
                assert!(a[i * 3 + 1] <= 70f32.to_radians() + 1e-4);
                if g.stance_mask() & (1 << i) != 0 {
                    let (dx, dz) = (f[i * 3] - g.mount[i][0], f[i * 3 + 2] - g.mount[i][1]);
                    let r = (dx * dx + dz * dz).sqrt();
                    assert!((0.0795..=0.1155).contains(&r), "leg {i} reach {r}");
                }
            }
        }
    }

    #[test]
    fn rolling_the_body_lowers_one_side() {
        let mut g = mk1();
        g.set_pose(10f32.to_radians(), 0.0, 0.0, 0.0, 0.0);
        g.update(0.01, 0.0, 0.0, 0.0);
        let fb = g.feet_body();
        // + roll = left side up, so left feet are further below the (tilted) body plane
        assert!(fb[4 * 3 + 1] < fb[1 * 3 + 1] - 0.02, "left {} right {}", fb[4 * 3 + 1], fb[1 * 3 + 1]);
        // feet stay put in the level frame
        let f = g.feet();
        assert!((f[4 * 3 + 1] - f[1 * 3 + 1]).abs() < 1e-6);
    }

    #[test]
    fn tripod_margin_on_flat_ground_matches_control_md() {
        let mut g = mk1();
        // mid-swing of tripod A: only B (legs 1, 3, 5) in stance
        for _ in 0..1000 {
            g.update(0.005, 0.0, 0.02, 0.0);
            if g.stance_mask() == 0b101010 {
                break;
            }
        }
        assert_eq!(g.stance_mask(), 0b101010);
        let m = g.stability_margin(0.0, 0.0);
        assert!((m - 0.083).abs() < 0.012, "margin {m}");
    }

    #[test]
    fn balance_levels_a_12_degree_slope() {
        // on the ground: body world roll = slope + the servos' response to the
        // commanded roll (first-order, 60 ms, as the sim's servo model); the
        // gyro sees the body's real rate (CONTROL.md §7)
        let mut b = Balance::new(0.6, 2.0, 0.05, 15.0, 60.0, 8.0, 15.0);
        let slope = 12f32.to_radians();
        let dt = 0.01;
        let (mut y, mut meas, mut prev) = (0.0f32, slope, slope);
        let mut worst_late = 0.0f32;
        for step in 0..800 {
            let rate = (meas - prev) / dt;
            prev = meas;
            b.update(dt, 0.0, 0.0, meas, 0.0, rate, 0.0);
            y += (b.roll() - y) * (1.0 - (-dt / 0.06f32).exp());
            meas = slope + y;
            if step == 400 {
                assert!(meas.to_degrees().abs() < 0.3, "after 4 s: {} deg", meas.to_degrees());
            }
            if step > 400 {
                worst_late = worst_late.max(meas.abs());
            }
        }
        assert!(worst_late.to_degrees() < 0.3, "chatter {} deg", worst_late.to_degrees());
        assert!(meas.to_degrees().abs() < 0.05, "after 8 s {} deg", meas.to_degrees());
    }
}
