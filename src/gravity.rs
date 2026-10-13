//! Selected native named-gravity modifier arithmetic (140f05a00).
//! Active entries/configuration and their lifecycle remain caller responsibilities.

/// Negative remaining time represents an indefinite modifier. Finite modifiers
/// fade to 1 over the final `fade_window`; native file initialization is 0.5.
/// Invalid inputs return neutral gravity as an original robustness policy.
pub fn modifier_factor(multiplier: f32, remaining: f32, fade_window: f32) -> f32 {
    if !multiplier.is_finite()
        || !remaining.is_finite()
        || !fade_window.is_finite()
        || multiplier < 0.
        || fade_window <= 0.
    {
        return 1.;
    }
    if remaining < 0. {
        multiplier
    } else {
        1. + (multiplier - 1.) * remaining.clamp(0., fade_window) / fade_window
    }
}

/// Aim entry scales the separate vertical scalar by available resource.
/// Caller supplies stat-adjusted target; 0.25 is the native file initializer.
pub fn aim_entry_vertical(vertical: f32, resource: f32, target: f32) -> f32 {
    vertical * (1. + (target - 1.) * resource.clamp(0., 1.))
}

/// Native aim phase uses the vertical scalar after any entry adjustment.
/// This is not the normalized resource or total vector speed. The threshold 1
/// is in native speed units; lab unit/state/scheduling mappings remain assumed.
pub fn aim_gravity_factor(vertical: f32, target: f32) -> f32 {
    1. + (target - 1.) * (1. - vertical).clamp(0., 1.)
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn finite_modifier_fades_to_neutral() {
        assert!((modifier_factor(0.65, 0.55, 0.5) - 0.65).abs() < 1e-6);
        assert!((modifier_factor(0.65, 0.25, 0.5) - 0.825).abs() < 1e-6);
        assert_eq!(modifier_factor(0.65, 0., 0.5), 1.);
    }
    #[test]
    fn indefinite_modifier_and_product_preserve_both_effects() {
        assert_eq!(modifier_factor(0.65, -1., 0.5), 0.65);
        let combined = modifier_factor(0.65, 0.25, 0.5) * modifier_factor(0.075, -1., 0.5);
        assert!((combined - 0.061875).abs() < 1e-6);
    }
    #[test]
    fn increased_gravity_also_fades_and_bad_window_is_neutral() {
        assert_eq!(modifier_factor(2., 0.25, 0.5), 1.5);
        assert_eq!(modifier_factor(2., 0.25, 0.), 1.);
        assert_eq!(modifier_factor(f32::NAN, 0.25, 0.5), 1.);
    }

    #[test]
    fn aim_entry_depends_on_resource_and_preserves_scalar_sign() {
        assert_eq!(aim_entry_vertical(8., 1., 0.25), 2.);
        assert_eq!(aim_entry_vertical(8., 0.5, 0.25), 5.);
        assert_eq!(aim_entry_vertical(-8., 1., 0.25), -2.);
        assert_eq!(aim_entry_vertical(8., 0., 0.25), 8.);
    }

    #[test]
    fn aim_gravity_blends_on_vertical_speed_not_resource() {
        for vertical in [-8., 0.] {
            assert!((aim_gravity_factor(vertical, 0.075) - 0.075).abs() < 1e-6);
        }
        assert!((aim_gravity_factor(0.5, 0.075) - 0.5375).abs() < 1e-6);
        assert_eq!(aim_gravity_factor(1., 0.075), 1.);
        assert_eq!(aim_gravity_factor(8., 0.075), 1.);
    }
}
