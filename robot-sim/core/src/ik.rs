//! Inverse kinematics for a 3-joint insect-style leg.
//!
//! Leg frame: origin at the coxa joint, +X points straight out from the body,
//! +Z points forward along the body, +Y is up.
//!
//! Returned angles (radians):
//! * coxa  – rotation about the vertical axis, 0 = pointing straight out
//! * femur – elevation of the femur above horizontal (positive = up)
//! * tibia – bend of the tibia relative to the femur (negative = folded down)

use wasm_bindgen::prelude::*;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct LegAngles {
    pub coxa: f32,
    pub femur: f32,
    pub tibia: f32,
    /// true if the target was out of reach and had to be clamped
    pub clamped: bool,
}

pub fn solve_leg(x: f32, y: f32, z: f32, coxa_len: f32, femur_len: f32, tibia_len: f32) -> LegAngles {
    solve_leg_limited(x, y, z, coxa_len, femur_len, tibia_len, -core::f32::consts::PI)
}

/// Like `solve_leg`, but never folds the tibia further than `min_tibia`
/// (radians, e.g. -135 deg for the Mk1 CAD, where the tibia bridge would hit
/// the coxa and the femur's servo cradle). The femur-to-foot distance only
/// depends on the tibia angle, so the limit becomes a minimum reach; the
/// foot then lands slightly further out along the same line.
#[allow(clippy::too_many_arguments)]
pub fn solve_leg_limited(x: f32, y: f32, z: f32, coxa_len: f32, femur_len: f32, tibia_len: f32, min_tibia: f32) -> LegAngles {
    let coxa = z.atan2(x);
    let horiz = (x * x + z * z).sqrt() - coxa_len;
    let mut d = (horiz * horiz + y * y).sqrt();

    // Knee interior angle k = pi + tibia, and d^2 = f^2 + t^2 - 2 f t cos(k)
    let k_min = (core::f32::consts::PI + min_tibia).max(0.0);
    let fold_d = (femur_len * femur_len + tibia_len * tibia_len - 2.0 * femur_len * tibia_len * k_min.cos()).max(0.0).sqrt();
    let min_d = fold_d.max((femur_len - tibia_len).abs()) + 1e-4;
    let max_d = femur_len + tibia_len - 1e-4;
    let mut clamped = false;
    if d > max_d {
        d = max_d;
        clamped = true;
    } else if d < min_d {
        d = min_d;
        clamped = true;
    }

    let to_foot = y.atan2(horiz);
    let cos_a = ((femur_len * femur_len + d * d - tibia_len * tibia_len) / (2.0 * femur_len * d)).clamp(-1.0, 1.0);
    let femur = to_foot + cos_a.acos();
    let cos_k = ((femur_len * femur_len + tibia_len * tibia_len - d * d) / (2.0 * femur_len * tibia_len)).clamp(-1.0, 1.0);
    let tibia = cos_k.acos() - core::f32::consts::PI;

    LegAngles { coxa, femur, tibia, clamped }
}

/// Forward kinematics, used by tests and by the JS side to check servo maps.
pub fn forward_leg(a: &LegAngles, coxa_len: f32, femur_len: f32, tibia_len: f32) -> [f32; 3] {
    let r = coxa_len + femur_len * a.femur.cos() + tibia_len * (a.femur + a.tibia).cos();
    let y = femur_len * a.femur.sin() + tibia_len * (a.femur + a.tibia).sin();
    [r * a.coxa.cos(), y, r * a.coxa.sin()]
}

/// Solve many legs at once. `targets` is [x, y, z] * n, the result is
/// [coxa, femur, tibia] * n.
#[wasm_bindgen]
pub fn solve_legs(targets: &[f32], coxa_len: f32, femur_len: f32, tibia_len: f32) -> Vec<f32> {
    let mut out = Vec::with_capacity(targets.len());
    for t in targets.chunks_exact(3) {
        let a = solve_leg(t[0], t[1], t[2], coxa_len, femur_len, tibia_len);
        out.extend_from_slice(&[a.coxa, a.femur, a.tibia]);
    }
    out
}

/// Planar 2-link IK used by the robotic arm (shoulder + elbow), returning
/// [shoulder, elbow, clamped] with the elbow bent upward. Shoulder is the
/// elevation above horizontal, elbow is relative to the upper arm.
#[wasm_bindgen]
pub fn solve_arm_2link(reach: f32, height: f32, upper: f32, fore: f32) -> Vec<f32> {
    // Same geometry as a leg with no coxa: the "knee up" solution of a leg
    // is the "elbow up" solution of an arm.
    let a = solve_leg(reach, height, 0.0, 0.0, upper, fore);
    vec![a.femur, a.tibia, if a.clamped { 1.0 } else { 0.0 }]
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ik_round_trips_through_fk() {
        let (c, f, t) = (0.03, 0.06, 0.09);
        for &(x, y, z) in &[(0.10, -0.06, 0.0), (0.09, -0.05, 0.04), (0.12, -0.03, -0.05)] {
            let a = solve_leg(x, y, z, c, f, t);
            assert!(!a.clamped);
            let p = forward_leg(&a, c, f, t);
            assert!((p[0] - x).abs() < 1e-4 && (p[1] - y).abs() < 1e-4 && (p[2] - z).abs() < 1e-4, "{:?} vs {:?}", p, (x, y, z));
        }
    }

    #[test]
    fn tibia_never_folds_past_its_limit() {
        let limit = -135f32.to_radians();
        // Foot tucked in close to the femur axis would need about -150 deg
        let free = solve_leg(0.085, -0.04, 0.0, 0.045, 0.075, 0.12);
        assert!(free.tibia < limit);
        let a = solve_leg_limited(0.085, -0.04, 0.0, 0.045, 0.075, 0.12, limit);
        assert!(a.clamped);
        assert!(a.tibia >= limit - 1e-3, "tibia {}", a.tibia.to_degrees());
        // A normal standing foot is untouched by the limit
        let s = solve_leg_limited(0.13, -0.09, 0.0, 0.045, 0.075, 0.12, limit);
        assert!(!s.clamped);
        assert_eq!(s, solve_leg(0.13, -0.09, 0.0, 0.045, 0.075, 0.12));
    }

    #[test]
    fn unreachable_target_is_clamped() {
        let a = solve_leg(1.0, 0.0, 0.0, 0.03, 0.06, 0.09);
        assert!(a.clamped);
    }
}
