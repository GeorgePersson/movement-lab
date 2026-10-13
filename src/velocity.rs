//! Original implementation of the recovered velocity equations.
//!
//! The research evidence establishes arithmetic, not baseline tuning or world
//! units. Gravity, collision, input scaling and fixed stepping belong to callers.
//! No-input braking returns a time-weighted mean of post-substep speeds.

use glam::Vec3;

#[derive(Clone, Copy, Debug)]
pub struct VelocityTuning {
    pub acceleration: f32,
    pub turn_rate: f32,
    pub deceleration: f32,
    pub max_speed: f32,
    /// Zero disables the elevation cutoff. The angle-helper identity is inferred
    /// to be sine; using Rust's sine is an explicit implementation approximation.
    pub angle_cutoff_degrees: f32,
}

const EPSILON: f32 = 1e-6;
const BRAKE_STEP: f32 = 1.0 / 30.0;

fn meaningfully_above(value: f32, limit: f32) -> bool {
    value > limit && (value - limit).abs() > EPSILON * value.abs().max(limit.abs()).max(1.0)
}

/// Integrate the movement velocity for one caller-provided timestep.
///
/// Deliberate laboratory safeguards: invalid vectors become zero; invalid or
/// negative tuning, nonpositive/nonfinite dt, and dt above one second preserve
/// the finite incoming velocity. The timestep bound prevents an accidental
/// unbounded braking loop. Nonfinite arithmetic results become zero. These are
/// local safety choices, not claims about the original engine's error handling.
pub fn accelerate_velocity(velocity: Vec3, target: Vec3, dt: f32, tuning: VelocityTuning) -> Vec3 {
    if !velocity.is_finite() || !velocity.length_squared().is_finite() {
        return Vec3::ZERO;
    }
    let coefficients = [
        tuning.acceleration,
        tuning.turn_rate,
        tuning.deceleration,
        tuning.max_speed,
        tuning.angle_cutoff_degrees,
    ];
    if !dt.is_finite()
        || dt <= 0.0
        || dt > 1.0
        || coefficients.iter().any(|x| !x.is_finite() || *x < 0.0)
    {
        return velocity;
    }
    if !target.is_finite() || !target.length_squared().is_finite() {
        return Vec3::ZERO;
    }

    let mut velocity = velocity;
    let mut acceleration = Vec3::ZERO;
    let target_length = target.length();
    if target_length > EPSILON {
        let direction = target / target_length;
        acceleration = direction * tuning.acceleration;
        let speed = velocity.length();
        if speed > 0.5 {
            let aligned = direction * (target_length.min(1.0) * speed);
            velocity -= (velocity - aligned) * (tuning.turn_rate * dt);
        }
    } else if dt > EPSILON {
        let original_speed = velocity.length();
        if original_speed > 0.0 {
            let direction = velocity / original_speed;
            let mut speed = original_speed;
            let mut remaining = dt;
            let mut average_speed = 0.0;
            while remaining > 0.0 {
                let step = remaining.min(BRAKE_STEP);
                speed = (speed - (speed * tuning.deceleration).max(3.0) * step).max(0.0);
                average_speed += step * speed * (1.0 / dt);
                remaining -= step;
                // Further zero-speed substeps contribute nothing to the mean.
                if speed == 0.0 {
                    break;
                }
            }
            velocity = direction * average_speed;
            if tuning.angle_cutoff_degrees > EPSILON * tuning.angle_cutoff_degrees.max(1.0) {
                let unit = velocity.normalize_or_zero();
                if unit.y.abs() > tuning.angle_cutoff_degrees.to_radians().sin() {
                    velocity = Vec3::ZERO;
                }
            }
        }
    }

    let previous_speed = velocity.length();
    let candidate = velocity + acceleration * dt;
    let candidate_speed = candidate.length();
    let result = if meaningfully_above(candidate_speed, tuning.max_speed) {
        let permitted_speed = if meaningfully_above(previous_speed, tuning.max_speed) {
            ((1.0 - tuning.turn_rate * 0.4 * dt) * previous_speed).max(tuning.max_speed)
        } else {
            tuning.max_speed
        };
        candidate * (permitted_speed / candidate_speed)
    } else {
        candidate
    };
    if result.is_finite() {
        result
    } else {
        Vec3::ZERO
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn tuning() -> VelocityTuning {
        // Synthetic inputs exercise equations; these are not recovered defaults.
        VelocityTuning {
            acceleration: 10.0,
            turn_rate: 2.0,
            deceleration: 4.0,
            max_speed: 5.0,
            angle_cutoff_degrees: 0.0,
        }
    }

    #[test]
    fn acceleration_reaches_cap_without_creating_overspeed() {
        let mut v = Vec3::ZERO;
        for _ in 0..300 {
            v = accelerate_velocity(v, Vec3::X, 1.0 / 120.0, tuning());
            assert!(v.x >= 0.0 && v.length() <= 5.00001);
            assert_eq!(v.y, 0.0);
            assert_eq!(v.z, 0.0);
        }
        assert!((v.x - 5.0).abs() < 1e-5);
    }

    #[test]
    fn existing_overspeed_decays_instead_of_snapping_to_cap() {
        let v = accelerate_velocity(Vec3::X * 10.0, Vec3::X, 0.1, tuning());
        assert!(v.x > tuning().max_speed && v.x < 10.0);
        assert!((v.x - 9.2).abs() < 1e-5);
    }

    #[test]
    fn braking_preserves_direction_and_eventually_stops() {
        let direction = Vec3::new(3.0, 0.0, 4.0).normalize();
        let mut v = direction * 4.0;
        for _ in 0..1000 {
            let next = accelerate_velocity(v, Vec3::ZERO, 1.0 / 120.0, tuning());
            assert!(next.length() <= v.length() + 1e-6);
            assert!(next.dot(direction) >= 0.0);
            assert!(next.cross(direction).length() < 1e-5);
            v = next;
        }
        assert_eq!(v, Vec3::ZERO);
    }

    #[test]
    fn braking_uses_average_speed_for_a_longer_update() {
        let t = VelocityTuning {
            deceleration: 0.0,
            ..tuning()
        };
        // Constant minimum deceleration gives post-step speeds 2.9 and 2.8.
        let v = accelerate_velocity(Vec3::X * 3.0, Vec3::ZERO, 2.0 / 30.0, t);
        assert!((v.x - 2.85).abs() < 1e-5);
    }

    #[test]
    fn invalid_inputs_are_bounded_and_zero_dt_preserves_motion() {
        assert_eq!(
            accelerate_velocity(Vec3::X, Vec3::Z, 0.0, tuning()),
            Vec3::X
        );
        assert_eq!(
            accelerate_velocity(Vec3::X, Vec3::Z, f32::NAN, tuning()),
            Vec3::X
        );
        assert_eq!(
            accelerate_velocity(Vec3::NAN, Vec3::X, 0.1, tuning()),
            Vec3::ZERO
        );
        assert_eq!(
            accelerate_velocity(Vec3::X, Vec3::NAN, 0.1, tuning()),
            Vec3::ZERO
        );
    }
}
