//! Original implementation of the recovered direct additive-record subset.
//! A full owner list, relative records and pending accumulation are not modeled.
//! The lab uses one instance per modeled owner and exact-zero expiry; native
//! epsilon pruning can differ for very small residual contributions.
use glam::Vec3;
use serde::{Deserialize, Serialize};

/// A velocity contribution with insertion-time decay, independent of base speed.
#[derive(Clone, Copy, Debug, Default, Serialize, Deserialize)]
pub struct DecayingImpulse {
    velocity: Vec3,
    decay: f32,
    evaluated: bool,
}
impl DecayingImpulse {
    pub fn velocity(&self) -> Vec3 {
        self.velocity
    }

    /// Landing projects persistent contributions without restarting their age,
    /// replacing the owner, or changing insertion-time decay.
    pub fn remove_vertical_component(&mut self) {
        self.velocity.y = 0.;
    }

    /// Replace the previous record and restart the full first evaluation.
    pub fn replace(&mut self, velocity: Vec3, decay: f32) {
        *self = Self {
            velocity,
            decay,
            evaluated: false,
        };
    }

    /// Update one existing owner without restarting its evaluation grace or
    /// changing its captured decay. An absent/expired owner is a fresh insertion.
    /// A zero update removes the owner, as in the native finish-event path.
    pub fn upsert(&mut self, velocity: Vec3, decay: f32) {
        if velocity == Vec3::ZERO {
            *self = Self::default();
        } else if self.velocity == Vec3::ZERO {
            self.replace(velocity, decay);
        } else {
            self.velocity = velocity;
        }
    }

    /// Call once per simulation substep. The controller validates dt and tuning.
    pub fn evaluate(&mut self, dt: f32) {
        if self.evaluated {
            let length = self.velocity.length();
            if length > 0. {
                self.velocity *= (length - dt * self.decay).max(0.) / length;
            }
        }
        self.evaluated = true;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn full_first_evaluation_then_linear_magnitude_decay_and_expiry() {
        let mut impulse = DecayingImpulse::default();
        impulse.replace(Vec3::new(3., 0., 4.), 2.);
        impulse.evaluate(0.5);
        assert_eq!(impulse.velocity(), Vec3::new(3., 0., 4.));
        impulse.evaluate(0.5);
        assert!((impulse.velocity() - Vec3::new(2.4, 0., 3.2)).length() < 1e-6);
        impulse.evaluate(10.);
        assert_eq!(impulse.velocity(), Vec3::ZERO);
    }

    #[test]
    fn replacement_restarts_grace_and_captures_new_decay() {
        let mut impulse = DecayingImpulse::default();
        impulse.replace(Vec3::X * 10., 2.);
        impulse.evaluate(1.);
        impulse.evaluate(1.);
        impulse.replace(Vec3::Z * 3., 1.);
        impulse.evaluate(1.);
        assert_eq!(impulse.velocity(), Vec3::Z * 3.);
        impulse.evaluate(1.);
        assert_eq!(impulse.velocity(), Vec3::Z * 2.);
        impulse.replace(Vec3::ZERO, 0.);
        impulse.evaluate(1.);
        assert_eq!(impulse.velocity(), Vec3::ZERO);
    }
    #[test]
    fn owner_upsert_preserves_decay_and_evaluated_but_removal_resets_them() {
        let mut impulse = DecayingImpulse::default();
        impulse.upsert(Vec3::X * 8., 2.);
        // Updating before first evaluation preserves the unevaluated grace.
        impulse.upsert(Vec3::Z * 6., 99.);
        impulse.evaluate(0.5);
        assert_eq!(impulse.velocity(), Vec3::Z * 6.);
        impulse.upsert(-Vec3::X * 5., 99.);
        impulse.evaluate(0.5);
        assert_eq!(impulse.velocity(), -Vec3::X * 4.);
        impulse.upsert(Vec3::ZERO, 99.);
        impulse.upsert(Vec3::Z * 3., 1.);
        impulse.evaluate(0.5);
        assert_eq!(impulse.velocity(), Vec3::Z * 3.);
        impulse.evaluate(0.5);
        assert_eq!(impulse.velocity(), Vec3::Z * 2.5);
    }

    #[test]
    fn expired_owner_reinsertion_is_fresh_with_new_decay() {
        let mut impulse = DecayingImpulse::default();
        impulse.upsert(Vec3::X, 2.);
        impulse.evaluate(1.);
        impulse.evaluate(1.);
        assert_eq!(impulse.velocity(), Vec3::ZERO);
        impulse.upsert(Vec3::Z * 4., 1.);
        impulse.evaluate(1.);
        assert_eq!(impulse.velocity(), Vec3::Z * 4.);
        impulse.evaluate(1.);
        assert_eq!(impulse.velocity(), Vec3::Z * 3.);
    }
}
