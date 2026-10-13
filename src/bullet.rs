//! Selected recovered direction arithmetic; scene/input origins remain lab policy.
use glam::Vec3;

// Recovered normalization rescales small nonzero vectors before squaring again.
// This preserves aim direction when the initial squared length underflows.
fn normalize_direction(mut value: Vec3) -> Vec3 {
    let length = value.length();
    if length > 1e-6 {
        return value * length.recip();
    }
    let largest = value.abs().max_element();
    if largest > 0.0 {
        value *= largest.recip();
        let length = value.length();
        if length > 1e-6 {
            value *= length.recip();
        }
    }
    value
}

/// Native 141565bd0 structure. `airborne` substitutes for actor flag 0x8000;
/// that equivalence and the supplied actor anchor are prototype approximations.
/// Zero/invalid forward fallback is an original robustness choice.
pub fn bullet_direction(forward: Vec3, anchor: Vec3, target: Option<Vec3>, airborne: bool) -> Vec3 {
    let base = if forward.is_finite() && forward.length_squared() > 1e-12 {
        forward.normalize_or_zero()
    } else {
        Vec3::NEG_Z
    };
    direction_with_target(
        base,
        anchor,
        target.filter(|p| p.is_finite() && anchor.is_finite()),
        airborne,
    )
}

/// Recovered direction arithmetic with an explicitly supplied base vector.
///
/// Unlike the lab camera wrapper, this does not normalize or validate `base`.
/// `flag_set` is the native Boolean gate; its live actor meaning is not supplied
/// here. The global up vector is modeled at its recovered initial value, +Y.
pub fn bullet_direction_from_base(base: Vec3, anchor: Vec3, target: Vec3, flag_set: bool) -> Vec3 {
    direction_with_target(base, anchor, Some(target), flag_set)
}

fn direction_with_target(base: Vec3, anchor: Vec3, target: Option<Vec3>, flag_set: bool) -> Vec3 {
    let mut direction = base;
    if let Some(target) = target {
        if normalize_direction(target - anchor).dot(base) > 0.5 {
            direction = normalize_direction(target + Vec3::Y - anchor);
        }
    }
    // Gate uses ORIGINAL base Y, not corrected direction Y. This assignment
    // can lower the corrected Y; replacing it with max(Y,.1) changes the rule.
    if !flag_set && base.y <= 0.1 {
        direction.y = 0.1;
        direction = normalize_direction(direction);
    }
    direction
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn target_correction_uses_offset_before_normalizing() {
        let anchor = Vec3::Y * 2.;
        let target = anchor + Vec3::new(0., 2., -4.);
        let base = (target - anchor).normalize();
        let actual = bullet_direction(base, anchor, Some(target), true);
        assert!(actual.distance(Vec3::new(0., 3., -4.).normalize()) < 1e-6);
    }
    #[test]
    fn behind_target_is_ignored() {
        assert_eq!(
            bullet_direction(Vec3::NEG_Z, Vec3::ZERO, Some(Vec3::Z), true),
            Vec3::NEG_Z
        );
    }
    #[test]
    fn original_base_y_controls_ground_lift_even_after_target_correction() {
        let anchor = Vec3::ZERO;
        let expected = Vec3::new(0., 0.1, -0.5_f32.sqrt()).normalize();
        assert!(
            bullet_direction(Vec3::NEG_Z, anchor, Some(Vec3::NEG_Z), false).distance(expected)
                < 1e-6
        );
    }
    #[test]
    fn straight_down_ground_aim_launches_up() {
        assert!(
            bullet_direction(Vec3::NEG_Y, Vec3::Y * 1.8, Some(Vec3::ZERO), false).distance(Vec3::Y)
                < 1e-6
        );
        assert!(
            bullet_direction(Vec3::NEG_Y, Vec3::Y * 1.8, Some(Vec3::ZERO), true)
                .distance(Vec3::NEG_Y)
                < 1e-6
        );
    }
    #[test]
    fn absent_or_coincident_target_preserves_air_direction() {
        let base = Vec3::new(1., 0.2, 0.).normalize();
        assert!(bullet_direction(base, Vec3::ZERO, None, true).distance(base) < 1e-6);
        assert!(bullet_direction(base, Vec3::ZERO, Some(Vec3::ZERO), true).distance(base) < 1e-6);
    }
    #[test]
    fn tiny_nonzero_target_still_enters_aim_correction() {
        let target = Vec3::NEG_Z * 1e-30;
        assert!(
            bullet_direction(Vec3::NEG_Z, Vec3::ZERO, Some(target), true).distance(Vec3::Y) < 1e-6
        );
        assert!(
            bullet_direction(Vec3::NEG_Z, Vec3::ZERO, Some(target), false).distance(Vec3::Y) < 1e-6
        );
    }
}
