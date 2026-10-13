//! Original finite wall-launch kernel derived from native1402C4860.
//! This returns the vector before the unresolved actor service hook140DD0150,
//! and a HEIGHT target for the separate sqrt/max launch helper. Native contact,
//! admission, event ordering, runtime stats and relative orientation are external.
//! Standard normalization/cosine express the mathematics, not native SIMD bits.
use glam::Vec3;
use serde::{Deserialize, Serialize};

const EPSILON: f32 = 1e-6;

/// Constructor assumptions, not proven active gameplay configuration.
#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
#[serde(default)]
pub struct WallJumpTuning {
    /// Raw mAirJumpSpeed (B48), also used by the facing branch.
    pub speed_multiplier: f32,
    /// Effective named WallJump multiplier, including its stat blend when known.
    /// None assumes identity modifiers, yielding the raw speed multiplier.
    pub modified_speed_multiplier: Option<f32>,
    pub height_multiplier: f32,
    /// Effective statA5 result; the facing branch uses its separate raw factor.
    pub modified_height_multiplier: Option<f32>,
    pub perpendicular_height_multiplier: f32,
    pub kick_off: f32,
    pub perpendicular_kick_off: f32,
    pub max_speed: f32,
    pub perpendicular_thrust: f32,
    pub along_wall_angle_degrees: f32,
    pub normal_cooldown: f32,
    pub positive_side_cooldown: f32,
    pub negative_side_cooldown: f32,
    pub facing_cooldown: f32,
    pub minimum_timing_multiplier: f32,
    pub timing_window: f32,
    pub facing_gravity_scale: f32,
    pub facing_gravity_duration: f32,
    pub side_gravity_scale: f32,
    pub gravity_duration: f32,
    pub old_school_wall_run: bool,
    pub use_input_direction: bool,
}
impl Default for WallJumpTuning {
    fn default() -> Self {
        Self {
            speed_multiplier: 1.,
            modified_speed_multiplier: None,
            height_multiplier: 2.5,
            modified_height_multiplier: None,
            perpendicular_height_multiplier: 1.,
            kick_off: 0.,
            perpendicular_kick_off: 0.1,
            max_speed: 20.,
            perpendicular_thrust: 1.5,
            along_wall_angle_degrees: 45.,
            normal_cooldown: 1.,
            positive_side_cooldown: 1.,
            negative_side_cooldown: 1.,
            facing_cooldown: 1.,
            minimum_timing_multiplier: 0.1,
            timing_window: 0.5,
            facing_gravity_scale: 0.5,
            facing_gravity_duration: 1.,
            side_gravity_scale: 0.5,
            gravity_duration: 1.,
            old_school_wall_run: false,
            use_input_direction: true,
        }
    }
}
impl WallJumpTuning {
    /// Lab robustness boundary; native malformed configuration is not emulated.
    pub fn is_valid(&self) -> bool {
        [
            self.speed_multiplier,
            self.height_multiplier,
            self.perpendicular_height_multiplier,
            self.kick_off,
            self.perpendicular_kick_off,
            self.max_speed,
            self.perpendicular_thrust,
            self.along_wall_angle_degrees,
            self.normal_cooldown,
            self.positive_side_cooldown,
            self.negative_side_cooldown,
            self.facing_cooldown,
            self.minimum_timing_multiplier,
            self.timing_window,
            self.facing_gravity_scale,
            self.side_gravity_scale,
        ]
        .iter()
        .all(|v| v.is_finite() && *v >= 0.)
            && self.timing_window > 0.
            && self.facing_gravity_duration.is_finite()
            && self.gravity_duration.is_finite()
            && [
                self.modified_speed_multiplier,
                self.modified_height_multiplier,
            ]
            .iter()
            .all(|v| v.is_none_or(|n| n.is_finite() && n >= 0.))
    }
}

#[derive(Clone, Copy, Debug)]
pub struct WallJumpInput {
    /// Native vector channel, excluding the separate launch/gravity scalar.
    pub old_velocity: Vec3,
    pub input_direction: Vec3,
    /// Caller-supplied analogue of actorC0; its live orientation is not proved.
    pub fallback_direction: Vec3,
    pub wall_normal: Vec3,
    /// Caller analogue of ClimbB0. Native forced mode takes its XZ components.
    pub wall_tangent: Vec3,
    pub in_wall_state: bool,
    /// Caller ages this signed timer. f32::MAX is the native initial sentinel.
    pub old_timer: f32,
    pub base_speed: f32,
    pub base_jump_height: f32,
}
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct WallLaunch {
    pub velocity: Vec3,
    /// Height for sqrt(2*launch_gravity*height), preserving stronger scalar.
    pub height: f32,
    pub cooldown: f32,
    pub gravity_scale: f32,
    pub gravity_duration: f32,
    pub subtype: u8,
}

/// Returns None for the native positive finite cooldown gate or invalid lab data.
/// The caller owns admission/budgets and the unresolved post-vector service hook.
/// Y-up is an explicit lab mapping of the initialized native world-up vector.
pub fn wall_launch(i: WallJumpInput, t: &WallJumpTuning) -> Option<WallLaunch> {
    if !t.is_valid()
        || !i.old_timer.is_finite()
        || !i.base_speed.is_finite()
        || i.base_speed < 0.
        || !i.base_jump_height.is_finite()
        || i.base_jump_height < 0.
        || [
            i.old_velocity,
            i.input_direction,
            i.fallback_direction,
            i.wall_normal,
            i.wall_tangent,
        ]
        .iter()
        .any(|v| !v.is_finite())
        || (i.old_timer > 0. && i.old_timer < f32::MAX)
    {
        return None;
    }
    let d = if i.input_direction.abs().max_element() <= 0.1 {
        normalize_safe(i.fallback_direction)
    } else {
        i.input_direction
    };
    let side = i.wall_normal.cross(Vec3::Y).dot(d);
    let nd = i.wall_normal.dot(d);
    let timer_scale = if i.old_timer < f32::MAX {
        1. + i.old_timer.abs().clamp(0., t.timing_window) / t.timing_window
            * (t.minimum_timing_multiplier - 1.)
    } else {
        1.
    };
    let forced = t.old_school_wall_run && !t.use_input_direction;
    let mut q = d + t.kick_off * i.wall_normal;
    let mut speed = i.base_speed * t.modified_speed_multiplier.unwrap_or(t.speed_multiplier);
    let mut height =
        i.base_jump_height * t.modified_height_multiplier.unwrap_or(t.height_multiplier);
    let mut subtype = 1;
    let mut gravity_scale = 1.;
    let mut gravity_duration = t.gravity_duration;
    let cooldown;
    if nd < -0.8 && !forced {
        q = t.perpendicular_kick_off * i.wall_normal;
        speed = i.base_speed * t.speed_multiplier;
        height = i.base_jump_height * t.perpendicular_height_multiplier;
        cooldown = t.facing_cooldown;
        subtype = 2;
        gravity_scale = t.facing_gravity_scale;
        gravity_duration = t.facing_gravity_duration;
    } else if (side.abs() <= t.along_wall_angle_degrees.to_radians().cos() && nd >= 0.) || forced {
        cooldown = t.normal_cooldown;
        if forced {
            q = i.wall_normal;
            if i.in_wall_state {
                q += i.wall_tangent * Vec3::new(1., 0., 1.);
            }
            q = normalize_safe(q);
        }
    } else {
        gravity_scale = t.side_gravity_scale;
        if side < 0. {
            cooldown = t.negative_side_cooldown;
            subtype = 4;
        } else {
            cooldown = t.positive_side_cooldown;
            subtype = 3;
        }
    }
    let old_length = i.old_velocity.length();
    if !old_length.is_finite() || !side.is_finite() || !nd.is_finite() {
        return None;
    }
    let retained = if old_length > EPSILON {
        let a = i.old_velocity / old_length;
        a * a.dot(q).max(0.)
    } else {
        i.old_velocity
    };
    let mut velocity = q * speed * (1. + (t.perpendicular_thrust - 1.) * nd.abs()) + retained;
    let length = velocity.length();
    if !length.is_finite() {
        return None;
    }
    if length > EPSILON {
        velocity *= t.max_speed.min(length) / length;
    }
    height = (timer_scale * height).max(0.);
    if !velocity.is_finite() || !height.is_finite() {
        return None;
    }
    Some(WallLaunch {
        velocity,
        height,
        cooldown,
        gravity_scale,
        gravity_duration,
        subtype,
    })
}

fn normalize_safe(v: Vec3) -> Vec3 {
    let largest = v.abs().max_element();
    if largest == 0. {
        Vec3::ZERO
    } else {
        let scaled = v / largest;
        scaled / scaled.length()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    fn input() -> WallJumpInput {
        WallJumpInput {
            old_velocity: Vec3::ZERO,
            input_direction: Vec3::X,
            fallback_direction: Vec3::X,
            wall_normal: Vec3::X,
            wall_tangent: Vec3::ZERO,
            in_wall_state: false,
            old_timer: f32::MAX,
            base_speed: 8.,
            base_jump_height: 4.,
        }
    }
    fn tuning() -> WallJumpTuning {
        WallJumpTuning {
            perpendicular_thrust: 1.,
            height_multiplier: 0.25,
            perpendicular_height_multiplier: 0.75,
            perpendicular_kick_off: 0.25,
            normal_cooldown: 0.1,
            positive_side_cooldown: 0.2,
            negative_side_cooldown: 0.3,
            facing_cooldown: 0.4,
            ..Default::default()
        }
    }
    #[test]
    fn normalized_old_projection_drops_speed_magnitude_and_rejects_opposition() {
        let mut i = input();
        let t = tuning();
        for speed in [2., 20., 200.] {
            i.old_velocity = Vec3::X * speed;
            assert_eq!(wall_launch(i, &t).unwrap().velocity, Vec3::X * 9.);
        }
        for old in [-Vec3::X * 100., Vec3::Z * 100.] {
            i.old_velocity = old;
            assert_eq!(wall_launch(i, &t).unwrap().velocity, Vec3::X * 8.);
        }
    }
    #[test]
    fn facing_branch_uses_raw_multipliers_and_height_not_scalar_speed() {
        let i = WallJumpInput {
            input_direction: -Vec3::X,
            ..input()
        };
        let t = WallJumpTuning {
            modified_speed_multiplier: Some(10.),
            modified_height_multiplier: Some(20.),
            ..tuning()
        };
        let r = wall_launch(i, &t).unwrap();
        assert_eq!(r.velocity, Vec3::X * 2.);
        assert_eq!(r.height, 3.);
        assert_eq!(
            (r.subtype, r.cooldown, r.gravity_scale, r.gravity_duration),
            (2, 0.4, 0.5, 1.)
        );
        let a = wall_launch(input(), &t).unwrap();
        assert_eq!(a.height, 80.);
        assert_eq!(a.velocity.length(), 20.);
    }
    #[test]
    fn cosine_and_side_sign_choose_distinct_cooldowns() {
        let t = WallJumpTuning {
            along_wall_angle_degrees: 60.,
            ..tuning()
        };
        for (d, subtype, cooldown) in [(Vec3::Z, 3, 0.2), (-Vec3::Z, 4, 0.3), (Vec3::X, 1, 0.1)] {
            let r = wall_launch(
                WallJumpInput {
                    input_direction: d,
                    ..input()
                },
                &t,
            )
            .unwrap();
            assert_eq!((r.subtype, r.cooldown), (subtype, cooldown));
        }
        // cos60=.5, whereas sin60>.8: this distinguishes cosine from sine.
        let r = wall_launch(
            WallJumpInput {
                input_direction: Vec3::new(0.8, 0., 0.6),
                ..input()
            },
            &t,
        )
        .unwrap();
        assert_eq!(r.subtype, 3);
    }
    #[test]
    fn timer_gate_and_deviation_scale_apply_to_height() {
        let t = WallJumpTuning {
            minimum_timing_multiplier: 0.5,
            timing_window: 0.4,
            ..tuning()
        };
        assert!(
            wall_launch(
                WallJumpInput {
                    old_timer: 0.1,
                    ..input()
                },
                &t
            )
            .is_none()
        );
        for (timer, height) in [(f32::MAX, 1.), (0., 1.), (-0.2, 0.75), (-10., 0.5)] {
            let r = wall_launch(
                WallJumpInput {
                    old_timer: timer,
                    ..input()
                },
                &t,
            )
            .unwrap();
            assert!((r.height - height).abs() < 1e-6);
            assert_eq!(r.velocity, Vec3::X * 8.);
        }
    }
    #[test]
    fn full_vector_cap_and_separate_height_preserve_vector_y() {
        let r = wall_launch(
            WallJumpInput {
                input_direction: Vec3::new(0., 1., 1.),
                ..input()
            },
            &WallJumpTuning {
                max_speed: 2.,
                ..tuning()
            },
        )
        .unwrap();
        assert!((r.velocity.length() - 2.).abs() < 1e-6);
        assert!((r.velocity.y - 2f32.sqrt()).abs() < 1e-6);
        assert_eq!(r.height, 1.);
    }
    #[test]
    fn fallback_threshold_and_forced_mode_are_separate() {
        let r = wall_launch(
            WallJumpInput {
                input_direction: Vec3::splat(0.1),
                fallback_direction: Vec3::Z * 1e-20,
                ..input()
            },
            &tuning(),
        )
        .unwrap();
        assert_eq!(r.velocity, Vec3::Z * 8.);
        assert_eq!(r.subtype, 3);
        let t = WallJumpTuning {
            old_school_wall_run: true,
            use_input_direction: false,
            ..tuning()
        };
        let r = wall_launch(
            WallJumpInput {
                input_direction: -Vec3::X,
                wall_tangent: Vec3::new(0., 99., 1.),
                in_wall_state: true,
                ..input()
            },
            &t,
        )
        .unwrap();
        assert_eq!(r.subtype, 1);
        assert_eq!(r.height, 1.);
        assert!((r.velocity - Vec3::new(1., 0., 1.).normalize() * 8.).length() < 1e-6);
    }
    #[test]
    fn zero_old_vector_and_zero_output_are_finite() {
        let r = wall_launch(
            WallJumpInput {
                input_direction: Vec3::ZERO,
                fallback_direction: Vec3::ZERO,
                ..input()
            },
            &tuning(),
        )
        .unwrap();
        assert_eq!(r.velocity, Vec3::ZERO);
        assert_eq!(r.height, 1.);
    }
    #[test]
    fn serde_defaults_and_robustness_validation() {
        let t: WallJumpTuning = serde_json::from_str("{}").unwrap();
        assert_eq!(t.speed_multiplier, 1.);
        assert_eq!(t.height_multiplier, 2.5);
        assert_eq!(t.modified_speed_multiplier, None);
        assert!(t.is_valid());
        assert!(
            WallJumpTuning {
                facing_gravity_duration: -1.,
                gravity_duration: -1.,
                ..t
            }
            .is_valid()
        );
        for bad in [0., -1., f32::NAN, f32::INFINITY] {
            assert!(
                !WallJumpTuning {
                    timing_window: bad,
                    ..t
                }
                .is_valid()
            );
        }
        assert!(
            wall_launch(
                WallJumpInput {
                    old_timer: f32::INFINITY,
                    ..input()
                },
                &t
            )
            .is_none()
        );
        assert!(
            wall_launch(
                WallJumpInput {
                    old_velocity: Vec3::splat(f32::MAX),
                    ..input()
                },
                &t
            )
            .is_none()
        );
    }
}
