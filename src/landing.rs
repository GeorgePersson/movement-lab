//! Original finite selector from the recovered landing helper 1414CC530.
//! Role, physics cache, effective stats, interruption, flags, input and graph
//! samples are supplied independently. No native callbacks or RNG seed are
//! reconstructed, and this helper does not execute landing or graph writes.

use glam::Vec3;
use serde::{Deserialize, Serialize};

const MOVEMENT_EPSILON: f32 = 1e-6;
const RANDOM_SCALE: f32 = 1. / 16_777_216.;

/// LAB defaults, not observed active native configuration. Runtime callers may
/// supply modified threshold/probability and their own independently held RNG.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct LandingTuning {
    /// Assumes the recovered constructor's raw threshold, without stat modifiers.
    pub heavy_threshold: f32,
    /// Assumes the modifier query's zero base, without active stat modifiers.
    pub resist_chance: f32,
    /// Deterministic LAB initialization only; not the native global RNG seed.
    /// The selector never reads this field or resets its caller's RNG state.
    pub rng_seed: u64,
}

impl Default for LandingTuning {
    fn default() -> Self {
        Self {
            heavy_threshold: 7.,
            resist_chance: 0.,
            rng_seed: 0,
        }
    }
}

impl LandingTuning {
    /// LAB robustness boundary; malformed native configurations are not emulated.
    pub fn is_valid(&self) -> bool {
        self.heavy_threshold.is_finite()
            && self.heavy_threshold >= 0.
            && self.resist_chance.is_finite()
            && (0. ..=1.).contains(&self.resist_chance)
    }
}

/// Samples at their respective native helper reads, not a common-time snapshot.
#[derive(Clone, Copy, Debug)]
pub struct LandingInputs {
    pub actor_allowed: bool,
    pub physics_available: bool,
    /// Supplied cached vertical scalar (native cache760), whose magnitude is
    /// returned by physics.v110. This is not necessarily current velocity.y.
    pub cached_vertical: f32,
    pub interrupted: bool,
    /// Actor650 bit4 sampled in the candidate branch.
    pub flag4: bool,
    /// Input.v1A0 XYZ sample; it may be later than the contact/cache samples.
    pub movement: Vec3,
    /// Fresh evaluated jump-property byte, not the earlier landing caller read.
    pub evaluated_jump: u8,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct InvalidLandingInput;

/// Return a style byte in {0,1,3,4}. This is not a jump ordinal or a pending graph
/// write. RNG advances only after both early guards, no interruption, and a
/// magnitude sample >= threshold; it also advances when probability rejects.
/// Rejecting invalid supplied values before any RNG change is a LAB policy.
pub fn select_landing_style(
    input: LandingInputs,
    tuning: &LandingTuning,
    rng: &mut u64,
) -> Result<u8, InvalidLandingInput> {
    if !tuning.is_valid() || !input.cached_vertical.is_finite() || !input.movement.is_finite() {
        return Err(InvalidLandingInput);
    }
    if !input.actor_allowed || !input.physics_available {
        return Ok(0);
    }

    let candidate = if !input.interrupted && input.cached_vertical.abs() >= tuning.heavy_threshold {
        *rng = rng
            .wrapping_mul(0x5851_f42d_4c95_7f2d)
            .wrapping_add(0x1405_7b7e_f767_814f);
        let random = ((*rng >> 38) & 0x00ff_ffff) as f32 * RANDOM_SCALE;
        random > tuning.resist_chance
    } else {
        false
    };
    if candidate && !input.flag4 {
        return Ok(1);
    }
    let moving = input.movement.abs().max_element() > MOVEMENT_EPSILON;
    Ok(if matches!(input.evaluated_jump, 1 | 2) {
        if moving {
            3
        } else if candidate {
            1
        } else {
            0
        }
    } else if moving {
        4
    } else {
        0
    })
}
