//! Original finite model of the recovered jump-budget reset decision.
//!
//! The decision kernel and phase adapter take explicit air/role inputs and keep
//! evaluated/staged graph values separate. The playable controller supplies a
//! documented LAB schedule/contact policy. Only represented reset fields and
//! requested effects are modeled here; native engine callbacks are not executed.

use serde::{Deserialize, Serialize};

/// Animation values are numeric graph properties, not jump-type ordinals.
/// `staged` is the pending value, never a previous-frame history sample.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub struct JumpProperties {
    pub evaluated: u8,
    pub staged: u8,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct ResetState {
    pub cached_air: bool,
    pub suppress_once: bool,
    pub tracked_seconds: f32,
}

/// A snapshot at the reset block, after any preceding native callbacks.
#[derive(Clone, Copy, Debug)]
pub struct ResetInputs {
    pub current_air: bool,
    /// Role-dependent repair predicate; physical support is not a substitute.
    pub repair_enabled: bool,
    pub animation_triggered_jump: bool,
    pub deferred_pending: bool,
    pub properties: JumpProperties,
    pub shared_remaining: u8,
    pub effective_shared_maximum: u8,
    pub dt: f32,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub enum ResetDecision {
    #[default]
    None,
    Suppressed,
    /// Invoke the landing path, including all three budget resets.
    Landing,
    /// Invoke explicit reset, clearing pending request/height and staging zero.
    ExplicitDeferredReset,
    /// Restore all three budgets and request staged jump property four.
    StationaryRepair,
}

/// A supplied role predicate and the lab's collision policy are deliberately
/// different sources. Neither selects or reconstructs the live native role.
#[derive(Clone, Copy, Debug)]
pub enum AirSample {
    SuppliedRolePredicate(bool),
    LabSupport {
        supported: bool,
        defer_while_cached_air: bool,
    },
}

impl AirSample {
    pub fn resolve(self, cached_air: bool) -> bool {
        match self {
            Self::SuppliedRolePredicate(air) => air,
            Self::LabSupport {
                supported,
                defer_while_cached_air,
            } => !supported || (cached_air && defer_while_cached_air),
        }
    }
}

/// Original phase adapter around the recovered decision kernel. Its stored
/// graph values and reset effects are bounded; engine callbacks are not run.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct JumpLifecycle {
    pub reset: ResetState,
    pub properties: JumpProperties,
    pub deferred_pending: bool,
    pub deferred_height: f32,
    pub property_four_seconds: f32,
    /// Pending IV_LAND_STYLE value (graph property 0x56), not IV_CROUCH.
    /// Evaluated landing style and native graph bindings are not modeled here.
    #[serde(default)]
    pub staged_land_style: f32,
    /// Pending IV_DODGE value; only type-two landing cleanup writes it here.
    #[serde(default)]
    pub staged_dodge: f32,
}

impl Default for JumpLifecycle {
    fn default() -> Self {
        Self {
            reset: ResetState::default(),
            properties: JumpProperties::default(),
            deferred_pending: false,
            deferred_height: -1.,
            property_four_seconds: 0.,
            staged_land_style: 0.,
            staged_dodge: 0.,
        }
    }
}

#[derive(Clone, Copy, Debug)]
pub struct LifecycleInputs {
    pub air: AirSample,
    pub repair_enabled: bool,
    pub animation_triggered_jump: bool,
    pub shared_remaining: u8,
    pub effective_shared_maximum: u8,
    pub dt: f32,
}

/// Requests to the surrounding controller, not proof that native callbacks ran.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct ResetEffects {
    pub decision: ResetDecision,
    pub refill_budgets: bool,
    pub clear_land_style: bool,
    /// The native repair route separately checks its posture/flag guard before
    /// requesting Input.v210. This flag requests that conditional check.
    pub check_repair_posture_cleanup: bool,
    pub landing_callback: bool,
    pub remove_vertical_additive_motion: bool,
    pub physics_reset_callback: bool,
}

impl JumpLifecycle {
    /// Called only after the outer landing guard and style selector. Pending is
    /// sampled here, after selector callbacks, not at initial landing entry.
    /// This selects a route; it does not run the event or refill budgets.
    pub fn landing_callback_action(&self, style: u8) -> LandingAction {
        if self.deferred_pending {
            LandingAction::JumpStartImpulse
        } else {
            LandingAction::Cleanup(LandingCleanup { style, subtype: 1 })
        }
    }

    /// Apply the represented kind-one packet graph writes after posture cleanup
    /// and earlier packet notifications. The supplied gate values are sampled at
    /// this later phase. No launch, budget refill or pending-state clear occurs.
    /// Type-two directional classification is external; subtype is supplied.
    pub fn apply_landing_cleanup(&mut self, packet: LandingCleanup, gate: LandingCleanupGate) {
        if !gate.actor_flag_four && !gate.inventory_blocks_styled_landing {
            self.staged_land_style = (packet.style as i8) as f32;
            if packet.style == 2 {
                self.staged_dodge = packet.subtype as f32;
            }
            self.properties.staged = 4;
        } else {
            self.staged_land_style = 0.;
            self.properties.staged = 0;
        }
    }

    /// Evaluate and apply only the represented reset fields. Landing callbacks,
    /// physics reset and actual graph bindings remain the caller's responsibility.
    /// This phase does not commit air, advance property-four time, or promote graph
    /// values; those operations have separate, explicit entry points below.
    pub fn evaluate_reset(&mut self, input: LifecycleInputs) -> Result<ResetEffects, InvalidTime> {
        let current_air = input.air.resolve(self.reset.cached_air);
        let decision = self.reset.evaluate(ResetInputs {
            current_air,
            repair_enabled: input.repair_enabled,
            animation_triggered_jump: input.animation_triggered_jump,
            deferred_pending: self.deferred_pending,
            properties: self.properties,
            shared_remaining: input.shared_remaining,
            effective_shared_maximum: input.effective_shared_maximum,
            dt: input.dt,
        })?;
        let mut effects = ResetEffects {
            decision,
            refill_budgets: decision.refills_budgets(),
            ..Default::default()
        };
        match decision {
            ResetDecision::Landing => {
                // Native landing clears this before its conditional callbacks.
                self.reset.cached_air = false;
                effects.landing_callback = true;
                effects.remove_vertical_additive_motion =
                    matches!(self.properties.evaluated, 1..=3);
            }
            ResetDecision::ExplicitDeferredReset => {
                if self.deferred_pending {
                    self.deferred_pending = false;
                    self.deferred_height = -1.;
                    self.properties.staged = 0;
                }
                effects.physics_reset_callback = true;
            }
            ResetDecision::StationaryRepair => {
                // This path preserves deferred request/height, unlike explicit reset.
                self.properties.staged = 4;
                self.staged_land_style = 0.;
                effects.clear_land_style = true;
                effects.check_repair_posture_cleanup = true;
            }
            ResetDecision::None | ResetDecision::Suppressed => {}
        }
        Ok(effects)
    }

    /// Run after reset callbacks and before cached-air commit. Reads evaluated,
    /// not staged, state. Native grace timing is a separate, unmodeled phase.
    pub fn advance_property_four_time(&mut self, dt: f32) -> Result<(), InvalidTime> {
        if !dt.is_finite()
            || dt < 0.
            || !self.property_four_seconds.is_finite()
            || self.property_four_seconds < 0.
        {
            return Err(InvalidTime);
        }
        let next = if self.properties.evaluated == 4 {
            self.property_four_seconds + dt
        } else {
            0.
        };
        if !next.is_finite() {
            return Err(InvalidTime);
        }
        if next > 2. {
            self.property_four_seconds = 0.;
            self.properties.staged = 0;
        } else {
            self.property_four_seconds = next;
        }
        Ok(())
    }

    /// LAB graph evaluation: no native binding overrides or animation clips.
    /// Promotion must be scheduled explicitly; staging never changes evaluated.
    pub fn promote_staged_for_lab(&mut self) {
        self.properties.evaluated = self.properties.staged;
    }

    /// Represent the normal animation-enabled type-zero request's field writes.
    /// A negative finite height preserves the stored request height. AIR_JUMP
    /// notification and D98(1)'s prewrite callback are omitted; this method models
    /// their non-reentrant field result, not callback scheduling or physics.
    /// Rejecting nonfinite input is LAB robustness, not a native packet branch.
    pub fn request_deferred_height(&mut self, height: f32) -> Result<(), InvalidHeight> {
        if !height.is_finite() {
            return Err(InvalidHeight);
        }
        self.deferred_pending = true;
        if height >= 0. {
            self.deferred_height = height;
        }
        self.properties.staged = 1;
        Ok(())
    }

    /// Sample event guards after preceding service callbacks, but before the
    /// physics/animation prelaunch callbacks. AIR_JUMP fails the event guard.
    /// This phase does not consume pending state or capture the stored height.
    pub fn check_jump_start_impulse(
        &self,
        context: JumpStartImpulseContext,
    ) -> Option<EligibleJumpStart> {
        if !context.own_sender
            || !context.is_jump_start_impulse
            || !context.animation_triggered_jump
            || !context.physics_available
            || !self.deferred_pending
        {
            return None;
        }
        Some(EligibleJumpStart { _private: () })
    }

    /// Invoke after the eligible event's physics/animation prelaunch callbacks.
    /// Height is read now; eligibility is not checked again after those callbacks.
    /// Pending clears before the returned command's physics attempt. The event
    /// never stages a graph property, including when the attempt later fails.
    pub fn begin_jump_start_impulse(&mut self, _eligible: EligibleJumpStart) -> HeightImpulse {
        self.deferred_pending = false;
        HeightImpulse {
            height: self.deferred_height,
        }
    }

    /// Complete an actually begun physics attempt. Failure also consumes the
    /// height and overwrites suppression with false. Do not clear pending again:
    /// a callback during the attempt may have queued a newer request.
    /// Movement-flag updates and resource debit are requested on both outcomes;
    /// their native callbacks and resource accounting are outside this module.
    pub fn complete_jump_start_impulse(
        &mut self,
        _attempt: HeightImpulse,
        succeeded: bool,
    ) -> HeightImpulseEffects {
        self.deferred_height = -1.;
        self.reset.suppress_once = succeeded;
        HeightImpulseEffects {
            request_movement_flag_updates: true,
            request_resource_debit: true,
        }
    }

    /// LAB supplies a successful immediate height launch at input time. With the
    /// chosen animation-enabled policy, each accepted launch stages
    /// property one. A successful height launch overwrites suppression; a direct
    /// scalar launch sets it only when it writes a stronger scalar. These are not
    /// graph jump-type ordinals and do not reproduce native clip/event latency.
    /// This does not consume deferred requests. The viewer uses the distinct
    /// request/JumpStartImpulse phases for ordinary launches with zero LAB latency.
    pub fn immediate_lab_launch(&mut self, height_launch: bool, scalar_written: bool) {
        self.properties.staged = 1;
        if height_launch {
            self.reset.suppress_once = true;
        } else if scalar_written {
            self.reset.suppress_once = true;
        }
    }

    /// The lab emits DodgeImpulse at accepted roll input. The bool is its supplied
    /// movement-flag policy, not the role-selected native air predicate.
    pub fn immediate_lab_dodge(&mut self, airborne_flag: bool) {
        if airborne_flag {
            self.properties.staged = 3;
        }
    }
}

/// Captured evaluated value precedes additive projection; staged value follows
/// projection but precedes Anim.v150. Do not reread staged after that callback.
pub fn landing_callback_eligible(
    evaluated_before_projection: u8,
    staged_after_projection: u8,
    animation_v150: bool,
) -> bool {
    matches!(evaluated_before_projection, 1..=3)
        && !matches!(staged_after_projection, 1 | 4)
        && !animation_v150
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum LandingAction {
    JumpStartImpulse,
    Cleanup(LandingCleanup),
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct LandingCleanup {
    pub style: u8,
    pub subtype: u8,
}

#[derive(Clone, Copy, Debug)]
pub struct LandingCleanupGate {
    pub actor_flag_four: bool,
    pub inventory_blocks_styled_landing: bool,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct InvalidHeight;

/// Explicit event guards evaluated after earlier callbacks, not frame history.
#[derive(Clone, Copy, Debug)]
pub struct JumpStartImpulseContext {
    pub own_sender: bool,
    pub is_jump_start_impulse: bool,
    pub animation_triggered_jump: bool,
    pub physics_available: bool,
}

/// Move-only proof that the event's guard phase was reached. Prelaunch callbacks
/// can change state after this phase; begin deliberately does not resample guards.
#[derive(Debug)]
pub struct EligibleJumpStart {
    _private: (),
}

/// A move-only command returned by a reached, eligible impulse event. Consuming
/// it at completion prevents a skipped guard from producing a completion token.
#[derive(Debug)]
pub struct HeightImpulse {
    height: f32,
}

impl HeightImpulse {
    pub fn height(&self) -> f32 {
        self.height
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct HeightImpulseEffects {
    pub request_movement_flag_updates: bool,
    pub request_resource_debit: bool,
}

impl ResetDecision {
    pub fn refills_budgets(self) -> bool {
        matches!(
            self,
            Self::Landing | Self::ExplicitDeferredReset | Self::StationaryRepair
        )
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct InvalidTime;

impl ResetState {
    /// Evaluate only the finite reset block. Invalid time leaves state untouched;
    /// rejecting it is a lab robustness policy, not a recovered native branch.
    ///
    /// The native cached-air commit occurs later, after grace/property-four
    /// timing. Call `commit_air_sample` at that separate phase, even following
    /// suppression. This method deliberately does not perform that commit.
    pub fn evaluate(&mut self, input: ResetInputs) -> Result<ResetDecision, InvalidTime> {
        if !input.dt.is_finite()
            || input.dt < 0.
            || !self.tracked_seconds.is_finite()
            || self.tracked_seconds < 0.
        {
            return Err(InvalidTime);
        }
        if self.suppress_once {
            self.suppress_once = false;
            return Ok(ResetDecision::Suppressed);
        }
        if input.current_air {
            return Ok(ResetDecision::None);
        }
        if self.cached_air {
            return Ok(ResetDecision::Landing);
        }
        if !input.repair_enabled {
            return Ok(ResetDecision::None);
        }
        let properties = input.properties;
        let tracking = input.animation_triggered_jump
            && (matches!(properties.evaluated, 1 | 2) || matches!(properties.staged, 1 | 2));
        let next_time = if tracking {
            self.tracked_seconds + input.dt
        } else {
            0.
        };
        if !next_time.is_finite() {
            return Err(InvalidTime);
        }
        self.tracked_seconds = next_time;
        if input.deferred_pending && !tracking {
            return Ok(ResetDecision::ExplicitDeferredReset);
        }
        let idle_empty = properties.evaluated == 0
            && properties.staged == 0
            && input.shared_remaining == 0
            && input.effective_shared_maximum != 0;
        if properties.evaluated == 3 || (tracking && next_time >= 2.) || idle_empty {
            self.tracked_seconds = 0.;
            Ok(ResetDecision::StationaryRepair)
        } else {
            Ok(ResetDecision::None)
        }
    }

    /// Commit the sampled native air state at the later lifecycle phase.
    pub fn commit_air_sample(&mut self, current_air: bool) {
        self.cached_air = current_air;
    }
}
