//! Finite direction/admission subset of native wall preparation140792B00.
//! This is not the query backend, traversal lifecycle or aim-latch identity.
//! The LAB caller supplies planar base velocity/input, a box-face normal and
//! Y-up; special actor bits are clear and the optional corner query is disabled.
use glam::Vec3;
use serde::{Deserialize, Serialize};

#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
#[serde(default)]
pub struct WallEntryTuning {
    /// Climb8A constructor true; active resource overrides are unknown.
    pub enabled: bool,
    /// Climb7C constructor60. Native uses the cosine family; standard Rust
    /// cosine is a finite approximation, not a bit-identical native helper.
    pub parallel_angle_degrees: f32,
}
impl Default for WallEntryTuning {
    fn default() -> Self {
        Self {
            enabled: true,
            parallel_angle_degrees: 60.,
        }
    }
}
impl WallEntryTuning {
    pub fn is_valid(self) -> bool {
        self.parallel_angle_degrees.is_finite()
            && (0. ..=180.).contains(&self.parallel_angle_degrees)
    }
}

#[derive(Clone, Copy, Debug)]
pub struct WallEntryInput {
    /// PhysicsD0 analogue. The controller deliberately excludes its separate
    /// vertical scalar and additive owners: unavailable native vectorY→LAB0.
    pub base_velocity: Vec3,
    /// Supplied service8E0.v1A0 analogue; native producer remains unbound.
    pub movement: Vec3,
    /// Selected horizontal normalized normal; LAB uses an AABB side face.
    pub wall_normal: Vec3,
    /// Opaque service8D0.v2B0 result; LAB explicitly supplies false.
    pub service_rejects: bool,
    /// Motion.v1A8 result; LAB supplies true for its valid character geometry.
    pub geometry_available: bool,
    /// Native missing Physics supplies zeroV and performs no modifier removal.
    pub physics_available: bool,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct WallEntryOutcome {
    /// Removal precedes direction rejection and is not rolled back on failure.
    pub remove_bullet_gravity: bool,
    /// Accepted direction only; not native success return (native is void).
    /// Subsequent state/effect publication and opaque callbacks are excluded.
    pub direction: Option<Vec3>,
}

fn normalized(value: Vec3) -> Vec3 {
    let length = value.length();
    if length > 1e-6 {
        // Native takes this branch for infinite computed length too. Finite
        // huge components then multiply by reciprocal zero, rather than using
        // the tiny-vector fallback and becoming an admitted unit direction.
        return value * (1. / length);
    }
    let largest = value.abs().max_element();
    if largest > 0. {
        let scaled = value * (1. / largest);
        let length = scaled.length();
        if length > 1e-6 {
            return scaled * (1. / length);
        }
        return scaled;
    }
    value
}

/// Both special-actor-bit reads are supplied clear; global up is suppliedY.
/// Climb80 is fixed at its constructor zero, where the unresolved native angle
/// helper returns zero. A nonzero setting would require a separately supplied
/// native threshold rather than assuming a general sine formula.
/// Climb78's constructor-false optional backend check is outside this subset.
/// Invalid supplied vectors/tuning returnNone without requesting any effects.
/// Nonfinite intermediate normalization rejects admission while preserving the
/// early effect; native ordered direction tests reject that result as well.
pub fn prepare_wall_entry(
    input: WallEntryInput,
    tuning: WallEntryTuning,
) -> Option<WallEntryOutcome> {
    if !tuning.is_valid()
        || !input.base_velocity.is_finite()
        || !input.movement.is_finite()
        || !input.wall_normal.is_finite()
    {
        return None;
    }
    let mut result = WallEntryOutcome {
        remove_bullet_gravity: false,
        direction: None,
    };
    if input.service_rejects || !input.geometry_available || !tuning.enabled {
        return Some(result);
    }
    result.remove_bullet_gravity = input.physics_available;
    let velocity = normalized(if input.physics_available {
        input.base_velocity
    } else {
        Vec3::ZERO
    });
    if !velocity.is_finite() {
        return Some(result);
    }
    let tangent = input.wall_normal.cross(Vec3::Y);
    let side = tangent.dot(input.movement);
    if !side.is_finite() {
        return Some(result);
    }
    let direction = if side.abs() > 0.3 {
        // Preserve the native multiply/divide order; no extra normalization.
        let direction = (side * tangent) * (1. / side.abs());
        let alignment = tuning.parallel_angle_degrees.to_radians().cos();
        let tangent_alignment = direction.dot(velocity);
        let forward = input.movement.dot(velocity);
        let outward = input.movement.dot(input.wall_normal);
        if !direction.is_finite()
            || !tangent_alignment.is_finite()
            || !forward.is_finite()
            || !outward.is_finite()
        {
            return Some(result);
        }
        if tangent_alignment < alignment || forward <= 0. || outward > 0. {
            return Some(result);
        }
        direction
    } else {
        let inward = (-input.wall_normal).dot(velocity);
        if !inward.is_finite() {
            return Some(result);
        }
        if inward <= 0.5 {
            return Some(result);
        }
        Vec3::Y
    };
    result.direction = Some(direction);
    Some(result)
}
