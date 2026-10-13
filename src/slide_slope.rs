//! Finite slope target composition from 1400EA640, separate from actor admission.
//! Normal is committed Physics3F0 (finalizer720), support is410bit4. LAB supplies
//! signed ongoing gravity and initialized +Y up. Scalar sqrt normalization is a
//! readable approximation to native packed RSQRT refinement, not bitwise parity.
use glam::Vec3;
const EPS: f32 = 1.0e-6;
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Path {
    OrdinaryInput,
    ZeroTarget,
    Downhill,
}
#[derive(Clone, Copy, Debug)]
pub struct Selection {
    pub target: Vec3,
    pub acceleration: f32,
    pub turn_rate: f32,
    pub path: Path,
}
fn nonzero(v: Vec3) -> bool {
    v.abs().max_element() > EPS
}
/// Caller handles direct-target bypass BEFORE this function. Deceleration and
/// cap are intentionally not arguments: the slope block does not replace them.
/// Nonfinite input rejection is a LAB safety policy, not native diagnostics.
#[allow(clippy::too_many_arguments)]
pub fn select(
    mask: u32,
    packet_a0: bool,
    normal: Vec3,
    signed_gravity: f32,
    slide_acceleration: f32,
    input: Vec3,
    acceleration: f32,
    turn_rate: f32,
) -> Selection {
    let mut out = Selection {
        target: input,
        acceleration,
        turn_rate,
        path: Path::OrdinaryInput,
    };
    if !normal.is_finite() || !signed_gravity.is_finite() || !slide_acceleration.is_finite() {
        out.target = Vec3::ZERO;
        out.path = Path::ZeroTarget;
        return out;
    }
    // Missing/zero normal selects ordinary transformed input, not the zero path.
    if mask & 4 == 0 || !packet_a0 || !nonzero(normal) {
        return out;
    }
    out.target = Vec3::ZERO;
    out.path = Path::ZeroTarget;
    let cross = normal.cross(Vec3::Y);
    if !nonzero(cross) {
        return out;
    }
    let double_cross = normal.cross(cross);
    let length = double_cross.length();
    if !length.is_finite() || length == 0. {
        return out;
    }
    let direction = double_cross * (1. / length);
    let q = signed_gravity * direction.y;
    if q <= 0.1 || !q.is_finite() {
        return out;
    }
    let coefficient = (q / signed_gravity).abs() * slide_acceleration;
    out.acceleration = coefficient;
    out.turn_rate = coefficient;
    let projected = Vec3::new(q * direction.x, 0., q * direction.z);
    let length = projected.length();
    let target = if length > EPS {
        projected * (1. / length)
    } else {
        let component = projected.abs().max_element();
        if component > 0. {
            let scaled = projected * (1. / component);
            let length = scaled.length();
            if length > EPS {
                scaled * (1. / length)
            } else {
                scaled
            }
        } else {
            projected
        }
    };
    if target.is_finite() {
        out.target = target;
        out.path = Path::Downhill;
    }
    out
}
#[cfg(test)]
mod tests {
    use super::*;
    fn run(mask: u32, a0: bool, n: Vec3, g: f32) -> Selection {
        select(mask, a0, n, g, 6., Vec3::Z, 12., 8.5)
    }
    #[test]
    fn source_gates_keep_fallback_distinct_from_zero() {
        for (mask, a0, n) in [
            (0, true, Vec3::Y),
            (3, true, Vec3::Y),
            (4, false, Vec3::Y),
            (4, true, Vec3::ZERO),
            (4, true, Vec3::splat(EPS)),
        ] {
            let r = run(mask, a0, n, -24.);
            assert_eq!(r.path, Path::OrdinaryInput);
            assert_eq!(r.target, Vec3::Z);
            assert_eq!(r.acceleration, 12.);
        }
        for n in [Vec3::Y, Vec3::NEG_Y, Vec3::new(EPS, 1., 0.)] {
            let r = run(4, true, n, -24.);
            assert_eq!(r.path, Path::ZeroTarget);
            assert_eq!(r.target, Vec3::ZERO);
            assert_eq!(r.turn_rate, 8.5);
        }
        assert_eq!(
            run(
                4,
                true,
                Vec3::new(f32::from_bits(EPS.to_bits() + 1), 1., 0.),
                -24.
            )
            .path,
            Path::ZeroTarget
        ); // q below .1
    }
    #[test]
    fn signed_gravity_threshold_and_coefficient_replacement() {
        // Unit horizontal normal gives d=-Y, so q=-g exactly in this scalar port.
        for q in [0., f32::from_bits(0.1f32.to_bits() - 1), 0.1] {
            assert_eq!(run(4, true, Vec3::X, -q).path, Path::ZeroTarget);
        }
        let r = run(4, true, Vec3::X, -f32::from_bits(0.1f32.to_bits() + 1));
        assert_eq!(r.path, Path::Downhill);
        assert_eq!(r.acceleration, 6.);
        assert_eq!(r.target, Vec3::ZERO);
        let n = Vec3::new(0., 4., -1.).normalize();
        let r = run(0xff, true, n, -24.);
        assert_eq!(r.path, Path::Downhill);
        assert!(r.target.z < -0.9999);
        assert!((r.acceleration - 6. / 17f32.sqrt()).abs() < 1e-5);
        assert_eq!(r.acceleration, r.turn_rate);
        assert_eq!(run(4, true, n, 24.).path, Path::ZeroTarget);
        assert_eq!(run(4, true, n, 0.).path, Path::ZeroTarget);
        let zero = select(4, true, n, -24., 0., Vec3::Z, 12., 8.5);
        assert_eq!(zero.path, Path::Downhill);
        assert_eq!(zero.acceleration, 0.);
        assert_eq!(zero.turn_rate, 0.);
        assert!(zero.target.z < 0.);
    }
    #[test]
    fn invalid_is_labeled_lab_zero_policy() {
        for n in [Vec3::splat(f32::NAN), Vec3::splat(f32::INFINITY)] {
            assert_eq!(run(4, true, n, -24.).path, Path::ZeroTarget);
        }
    }
}
