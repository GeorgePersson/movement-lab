use crate::{
    impulse::DecayingImpulse,
    jump_animation::{GraphInputs, JumpEvent, JumpStyle, LabJumpAnimation},
    jump_graph::JumpBinding,
    jump_lifecycle::{
        AirSample, JumpLifecycle, JumpStartImpulseContext, LandingAction, LandingCleanupGate,
        LifecycleInputs, ResetDecision, landing_callback_eligible,
    },
    landing::{LandingInputs, LandingTuning, select_landing_style},
    velocity::{VelocityTuning, accelerate_velocity},
    wall_entry::{WallEntryInput, WallEntryTuning, prepare_wall_entry},
    wall_jump::{WallJumpInput, WallJumpTuning, wall_launch},
    world::World,
};
use glam::Vec3;
use serde::{Deserialize, Serialize};

// Original lab resolution, not a recovered native tick frequency. Finer slices
// bound differences in one-preparation launch flags across caller frame rates.
const MAX_SUBSTEP: f32 = 1. / 240.;

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub enum MovementState {
    #[default]
    Idle,
    Walk,
    Sprint,
    Crouch,
    Slide,
    Air,
    BulletJump,
    Roll,
    AimGlide,
    WallLatch,
    WallJump,
}

/// Jump and roll are press edges. Movement is a world-space analog direction;
/// camera_forward includes pitch for the bullet-jump launch direction.
#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
pub struct Input {
    pub movement: Vec3,
    pub camera_forward: Vec3,
    /// Optional world target from the aim query. Origin/filtering remain approximate.
    #[serde(default)]
    pub aim_target: Option<Vec3>,
    pub jump: bool,
    #[serde(default)]
    pub jump_held: bool,
    #[serde(default)]
    pub jump_actions: crate::wall_traversal::JumpActions,
    #[serde(default)]
    pub jump_edge_id: Option<u64>,
    pub crouch: bool,
    pub sprint: bool,
    pub aim: bool,
    pub roll: bool,
}
impl Default for Input {
    fn default() -> Self {
        Self {
            movement: Vec3::ZERO,
            camera_forward: -Vec3::Z,
            aim_target: None,
            jump: false,
            jump_held: false,
            jump_actions: Default::default(),
            jump_edge_id: None,
            crouch: false,
            sprint: false,
            aim: false,
            roll: false,
        }
    }
}

/// Original approximate baseline. Slide boost/cooldown constructor defaults
/// and the unmodified AimGravity literal are statically anchored; runtime
/// configuration may override them. No defaults here are gameplay measurements.
#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
pub struct Tuning {
    pub radius: f32,
    pub standing_height: f32,
    pub crouch_height: f32,
    #[serde(default)]
    pub collision: crate::world::CollisionTuning,
    pub walk_speed: f32,
    pub sprint_speed: f32,
    pub crouch_speed: f32,
    pub ground_acceleration: f32,
    pub air_acceleration: f32,
    /// Legacy fallback for velocity alignment drag, not character yaw speed.
    pub turn_rate: f32,
    /// Ground DecelDrag used by the recovered no-input braking equation.
    pub ground_drag: f32,
    /// Ground AccelDrag; absent/null preserves the legacy `turn_rate` value.
    #[serde(default)]
    pub ground_accel_drag: Option<f32>,
    /// AirAccelDrag controls air alignment and damping of excess speed.
    /// Absent/null preserves the legacy `turn_rate` value.
    #[serde(default)]
    pub air_accel_drag: Option<f32>,
    /// AirDecelDrag, separate from steering. Legacy files used zero; the
    /// recovered braking equation still has its minimum deceleration of 3.
    #[serde(default)]
    pub air_decel_drag: f32,
    /// Downward acceleration during motion, before named gravity modifiers.
    pub gravity: f32,
    /// Positive gravity magnitude used only for ordinary/double launch targets.
    /// Some(9.81) represents the recovered native helper's zero-argument fallback.
    /// None (including absent legacy JSON) preserves the old use of `gravity`.
    #[serde(default)]
    pub launch_gravity: Option<f32>,
    /// Shared launch budget; default 2 is approximate, not a recovered active config.
    #[serde(default = "default_max_jumps")]
    pub max_jumps: u8,
    /// Effective bullet allowance used at lab initialization/landing/wall reset.
    /// Native mNumBulletJumps reset source; default1 is a constructor assumption.
    #[serde(default = "default_special_allowance")]
    pub max_bullet_jumps: u8,
    /// Effective air-dodge maximum after any external stats. Native own Tenno
    /// NumAirDodges=1; active loadout and lab roll admission remain unmeasured.
    #[serde(default = "default_special_allowance")]
    pub max_air_rolls: u8,
    /// Native mAirDodgeConsumesJump; false is a constructor assumption. The lab
    /// maps native flag/admission gates to its airborne state and accepted roll.
    #[serde(default)]
    pub air_roll_consumes_jump: bool,
    pub jump_height: f32,
    pub double_jump_height: f32,
    pub double_jump_horizontal_gain: f32,
    /// Base scalar used by the recovered double-jump gain, independently of
    /// bullet launch speed. The default 18 preserves the old lab default impulse;
    /// it is a compatibility choice, not a recovered active physics setting.
    /// Legacy JSON without this field loads 18 even if bullet speed was customized.
    /// Set this to the old bullet speed to preserve that file's former double gain.
    #[serde(default = "default_physics_base_speed")]
    pub physics_base_speed: f32,
    pub bullet_jump_speed: f32,
    pub bullet_jump_vertical_scale: f32,
    pub bullet_jump_gravity_scale: f32,
    pub bullet_jump_duration: f32,
    /// Native file initializer; active runtime value remains unmeasured.
    #[serde(default = "default_gravity_fade_window")]
    pub gravity_modifier_fade_window: f32,
    pub slide_boost: f32,
    pub slide_boost_cooldown: f32,
    /// Insertion-time linear decay for the separate slide velocity record.
    /// Default 10 is a native constructor assumption, not an active-value proof.
    #[serde(default = "default_slide_impulse_decay")]
    pub slide_impulse_decay: f32,
    /// Flat-ground base-velocity DecelDrag coefficient, not fixed units/s².
    /// The separate slide impulse has its own insertion-time decay.
    pub slide_drag: f32,
    /// Static Tenno SlideAccel candidate; active configuration is unproved.
    /// Default/absent JSON adopts6 as LAB policy, not a generic native default.
    #[serde(default = "default_slide_slope_acceleration")]
    pub slide_slope_acceleration: f32,
    pub slide_end_speed: f32,
    /// Legacy JSON name retained: additive roll impulse magnitude, not a base
    /// speed target. Event-at-input timing and magnitude remain lab assumptions.
    pub roll_speed: f32,
    /// Captured roll-record decay; default10 is a constructor assumption.
    #[serde(default = "default_roll_impulse_decay")]
    pub roll_impulse_decay: f32,
    pub roll_duration: f32,
    pub aim_duration: f32,
    pub aim_recharge: f32,
    pub aim_exit_penalty: f32,
    /// Entry target for the separate vertical scalar; file initializer 0.25.
    /// Runtime stat modifiers and eligibility are not modeled in this lab.
    #[serde(default = "default_aim_entry_vertical_scale")]
    pub aim_entry_vertical_scale: f32,
    /// Target at vertical scalar <=0; gravity blends toward 1 when rising.
    pub aim_gravity_scale: f32,
    pub wall_latch_duration: f32,
    /// Finite native preparation subset, supplied planar signals and AABB face.
    /// Entry requests and frozen-latch continuation remain explicit LAB policies.
    #[serde(default)]
    pub wall_entry: WallEntryTuning,
    /// Recovered finite wall-launch coefficients; constructor defaults are
    /// assumptions. Native admission/contact, stats and service hook are partial.
    #[serde(default)]
    pub wall_jump: WallJumpTuning,
    #[serde(default)]
    pub wall_traversal: crate::wall_traversal::Policy,
    /// Constructor/base assumptions and an original deterministic RNG seed;
    /// active native modifiers, global RNG and contact roles are not observed.
    #[serde(default)]
    pub landing: LandingTuning,
}
fn default_slide_slope_acceleration() -> f32 { 6. }

impl Default for Tuning {
    fn default() -> Self {
        Self {
            radius: 0.3,
            standing_height: 1.8,
            crouch_height: 0.9,
            collision: crate::world::CollisionTuning::default(),
            walk_speed: 6.,
            sprint_speed: 9.,
            crouch_speed: 3.,
            ground_acceleration: 50.,
            air_acceleration: 14.,
            turn_rate: 16.,
            ground_drag: 14.,
            ground_accel_drag: None,
            air_accel_drag: None,
            air_decel_drag: 0.,
            gravity: 28.,
            launch_gravity: None,
            max_jumps: default_max_jumps(),
            max_bullet_jumps: default_special_allowance(),
            max_air_rolls: default_special_allowance(),
            air_roll_consumes_jump: false,
            jump_height: 1.5,
            double_jump_height: 1.4,
            double_jump_horizontal_gain: 0.35,
            physics_base_speed: default_physics_base_speed(),
            bullet_jump_speed: 18.,
            bullet_jump_vertical_scale: 1.,
            bullet_jump_gravity_scale: 0.65,
            bullet_jump_duration: 0.55,
            gravity_modifier_fade_window: default_gravity_fade_window(),
            slide_boost: 8.,
            slide_boost_cooldown: 1.,
            slide_impulse_decay: default_slide_impulse_decay(),
            slide_drag: 3.,
            slide_slope_acceleration: default_slide_slope_acceleration(),
            slide_end_speed: 2.,
            roll_speed: 16.,
            roll_impulse_decay: default_roll_impulse_decay(),
            roll_duration: 0.42,
            aim_duration: 2.5,
            aim_recharge: 0.7,
            aim_exit_penalty: 0.05,
            aim_entry_vertical_scale: default_aim_entry_vertical_scale(),
            aim_gravity_scale: 0.075,
            wall_latch_duration: 3.,
            wall_entry: WallEntryTuning::default(),
            wall_jump: WallJumpTuning::default(),
            wall_traversal: Default::default(),
            landing: LandingTuning::default(),
        }
    }
}
impl Tuning {
    pub fn is_valid(&self) -> bool {
        let values = [
            self.radius,
            self.standing_height,
            self.crouch_height,
            self.collision.contact_offset,
            self.collision.step_offset,
            self.walk_speed,
            self.sprint_speed,
            self.crouch_speed,
            self.ground_acceleration,
            self.air_acceleration,
            self.turn_rate,
            self.ground_drag,
            self.air_decel_drag,
            self.gravity,
            self.jump_height,
            self.double_jump_height,
            self.double_jump_horizontal_gain,
            self.physics_base_speed,
            self.bullet_jump_speed,
            self.bullet_jump_vertical_scale,
            self.bullet_jump_gravity_scale,
            self.bullet_jump_duration,
            self.gravity_modifier_fade_window,
            self.slide_boost,
            self.slide_boost_cooldown,
            self.slide_impulse_decay,
            self.slide_drag,
            self.slide_slope_acceleration,
            self.slide_end_speed,
            self.roll_speed,
            self.roll_impulse_decay,
            self.roll_duration,
            self.aim_duration,
            self.aim_recharge,
            self.aim_exit_penalty,
            self.aim_entry_vertical_scale,
            self.aim_gravity_scale,
            self.wall_latch_duration,
        ];
        values.iter().all(|v| v.is_finite() && *v >= 0.)
            && [self.ground_accel_drag, self.air_accel_drag]
                .iter()
                .all(|drag| drag.is_none_or(|d| d.is_finite() && d >= 0.))
            && self.launch_gravity.is_none_or(|g| g.is_finite() && g > 0.)
            && self.radius > 0.
            && self.crouch_height >= 2. * self.radius
            && self.standing_height >= self.crouch_height
            && self.aim_duration > 0.
            && self.wall_latch_duration > 0.
            && self.max_jumps > 0
            && self.collision.engine.is_valid()
            && self.wall_jump.is_valid()
            && self.wall_entry.is_valid()
            && self.landing.is_valid()
            && self.gravity_modifier_fade_window > 0.
    }
}

fn default_wall_timer() -> f32 {
    f32::MAX
}
fn default_wall_gravity_scale() -> f32 {
    1.
}
fn default_roll_impulse_decay() -> f32 {
    10.
}

fn default_slide_impulse_decay() -> f32 {
    10.
}

fn default_aim_entry_vertical_scale() -> f32 {
    0.25
}

fn default_special_allowance() -> u8 {
    1
}
fn default_max_jumps() -> u8 {
    2
}

fn default_physics_base_speed() -> f32 {
    18.
}

fn default_gravity_fade_window() -> f32 {
    0.5
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Player {
    /// Feet, not center; renderer center is position + Y * height/2.
    pub position: Vec3,
    /// Ordinary base velocity, excluding the separate slide and roll channels.
    pub velocity: Vec3,
    #[serde(default)]
    slide_impulse: DecayingImpulse,
    #[serde(default)]
    roll_impulse: DecayingImpulse,
    pub grounded: bool,
    pub state: MovementState,
    pub crouched: bool,
    pub aim_resource: f32,
    /// Shared allowance for ordinary, bullet and double launches.
    #[serde(default = "default_max_jumps")]
    pub jumps_remaining: u8,
    /// HUD projection: shared allowance minus ground launch when grounded.
    pub air_jumps_left: u8,
    pub bullet_jumps_left: u8,
    pub air_rolls_left: u8,
    pub wall_normal: Option<Vec3>,
    time: f32,
    last_slide_boost: f32,
    roll_remaining: f32,
    roll_direction: Vec3,
    bullet_remaining: f32,
    /// Named gravity lifetime is independent of the visible movement state.
    #[serde(default)]
    bullet_gravity_remaining: f32,
    wall_latch_remaining: f32,
    wall_latch_exhausted: bool,
    wall_jump_remaining: f32,
    /// Native-like timing history keeps negative elapsed deviation after cooldown.
    #[serde(default = "default_wall_timer")]
    wall_jump_timer: f32,
    #[serde(default)]
    wall_traversal: crate::wall_traversal::State,
    #[serde(default)]
    wall_gravity_remaining: f32,
    #[serde(default = "default_wall_gravity_scale")]
    wall_gravity_scale: f32,
    #[serde(default)]
    pub wall_jump_subtype: u8,
    previously_crouch: bool,
    slide_requested: bool,
    aim_was_active: bool,
    #[serde(default)]
    jump_budget_initialized: bool,
    #[serde(default)]
    jump_lifecycle: JumpLifecycle,
    #[serde(default)]
    engine_contact: crate::world::EngineContactRuntime,
    /// LAB diagnostic outside the atomic World collision transaction.
    #[serde(default)]
    last_world_move_failed: bool,
    #[serde(default)]
    jump_binding: JumpBinding,
    #[serde(default)]
    jump_animation: LabJumpAnimation,
    #[serde(default)]
    jump_style: JumpStyle,
    #[serde(default)]
    last_jump_reset: ResetDecision,
    /// Last scalar sampled under the LAB controller predicate, before collision.
    /// Separate from clipped current velocity and from a peak/impact-speed metric.
    #[serde(default)]
    landing_cached_vertical: f32,
    #[serde(default)]
    landing_rng: u64,
    /// Supplied counterpart of actor C2B; the lab does not synthesize native writers.
    #[serde(default)]
    landing_style_flag_c2b: bool,
}
impl Player {
    /// Read-only LAB collision runtime diagnostics and resume validation.
    pub fn engine_contact_runtime(&self) -> &crate::world::EngineContactRuntime {
        &self.engine_contact
    }
    pub fn last_world_move_failed(&self) -> bool { self.last_world_move_failed }
    pub fn new(spawn: Vec3) -> Self {
        Self {
            position: spawn,
            velocity: Vec3::ZERO,
            slide_impulse: DecayingImpulse::default(),
            roll_impulse: DecayingImpulse::default(),
            grounded: false,
            state: MovementState::Air,
            crouched: false,
            aim_resource: 1.,
            jumps_remaining: default_max_jumps(),
            air_jumps_left: default_max_jumps(),
            bullet_jumps_left: default_special_allowance(),
            air_rolls_left: default_special_allowance(),
            wall_normal: None,
            time: 0.,
            last_slide_boost: -1000.,
            roll_remaining: 0.,
            roll_direction: -Vec3::Z,
            bullet_remaining: 0.,
            bullet_gravity_remaining: 0.,
            wall_latch_remaining: 0.,
            wall_latch_exhausted: false,
            wall_jump_remaining: 0.,
            wall_jump_timer: default_wall_timer(),
            wall_traversal: Default::default(),
            wall_gravity_remaining: 0.,
            wall_gravity_scale: 1.,
            wall_jump_subtype: 0,
            previously_crouch: false,
            slide_requested: false,
            aim_was_active: false,
            jump_budget_initialized: false,
            jump_lifecycle: JumpLifecycle::default(),
            engine_contact: crate::world::EngineContactRuntime::default(),
            last_world_move_failed: false,
            jump_binding: JumpBinding::default(),
            jump_animation: LabJumpAnimation::default(),
            jump_style: JumpStyle::default(),
            last_jump_reset: ResetDecision::None,
            landing_cached_vertical: 0.,
            landing_rng: 0,
            landing_style_flag_c2b: false,
        }
    }
    pub fn respawn(&mut self, spawn: Vec3) {
        *self = Self::new(spawn);
    }
    /// Diagnostic phase state; these graph/contact inputs are generated by the
    /// lab and are not observations of the running game.
    pub fn wall_traversal(&self) -> &crate::wall_traversal::State { &self.wall_traversal }
    /// Explicit LAB helper/reset/focus transport boundary; ordinary release differs.
    pub fn cancel_wall_input(&mut self) { self.wall_traversal.hard_cancel(); }
    /// Helper/manual adapter handoff preserves existing tickets and dispositions.
    pub fn wall_transport_boundary(&mut self) {
        self.wall_traversal.authoritative=None;
        self.wall_traversal.legacy_held=false;
    }
    pub fn jump_lifecycle(&self) -> &JumpLifecycle {
        &self.jump_lifecycle
    }

    pub fn last_jump_reset(&self) -> ResetDecision {
        self.last_jump_reset
    }

    fn apply_lab_jump_start_impulse(&mut self, t: &Tuning) -> bool {
        let Some(eligible) =
            self.jump_lifecycle
                .check_jump_start_impulse(JumpStartImpulseContext {
                    own_sender: true,
                    is_jump_start_impulse: true,
                    animation_triggered_jump: true,
                    physics_available: true,
                })
        else {
            return false;
        };
        // LAB has no intervening prelaunch engine callbacks. Native height -1
        // delegates to the configured-height getter; t.jump_height is its LAB input.
        let attempt = self.jump_lifecycle.begin_jump_start_impulse(eligible);
        let height = if attempt.height() < 0. {
            t.jump_height
        } else {
            attempt.height()
        };
        let succeeded = height.is_finite() && height >= 0.;
        if succeeded {
            self.velocity.y = self
                .velocity
                .y
                .max((2. * t.launch_gravity.unwrap_or(t.gravity) * height).sqrt());
            self.grounded = false;
            self.slide_requested = false;
        }
        let _effects = self
            .jump_lifecycle
            .complete_jump_start_impulse(attempt, succeeded);
        // Native flag notifications/resource debit remain separate unmodeled effects.
        succeeded
    }

    /// Chosen LAB phase order: promote requests, evaluate the selected graph,
    /// then drain its events before preparing physics. The runtime supplies no
    /// external bindings and keeps a clip active while unsupported. Neither
    /// that predicate nor this cadence is a recovered animation duration.
    fn evaluate_lab_jump_graph(&mut self, input: &Input, t: &Tuning) -> bool {
        if self
            .jump_binding
            .promote(&mut self.jump_lifecycle.properties)
            .is_err()
        {
            return false;
        }
        let events = self.jump_animation.evaluate(
            &mut self.jump_lifecycle.properties,
            &mut self.jump_binding,
            GraphInputs {
                style: self.jump_style,
                aiming: input.aim,
                active_clip: !self.grounded,
                sliding: self.slide_requested,
            },
        );
        let mut height_launched = false;
        for event in events {
            match event {
                JumpEvent::Impulse => height_launched |= self.apply_lab_jump_start_impulse(t),
                // Their native fanout is not yet recovered. Omitting it is an
                // explicit LAB policy, not proof that these events are no-ops.
                JumpEvent::Start | JumpEvent::Wait => {}
            }
        }
        height_launched
    }

    fn advance_lab_jump_lifecycle(
        &mut self,
        input: &Input,
        dt: f32,
        world: &World,
        t: &Tuning,
    ) -> bool {
        // LAB schedule: consume previous published collision support before
        // graph promotion, input events and this substep's collision publication.
        // This is not a recovered native update rate or universal contact lag.
        let air = if self.engine_contact.initialized {
            AirSample::SuppliedRolePredicate(
                self.engine_contact
                    .air(self.jump_lifecycle.reset.cached_air, &t.collision.engine),
            )
        } else {
            // Legacy/new player has not published an engine state yet.
            AirSample::LabSupport {
                supported: self.grounded,
                defer_while_cached_air: false,
            }
        };
        let sampled_air = air.resolve(self.jump_lifecycle.reset.cached_air);
        let Ok(effects) = self.jump_lifecycle.evaluate_reset(LifecycleInputs {
            air,
            // Chosen LAB role and animation policies, not detected live values.
            repair_enabled: true,
            animation_triggered_jump: true,
            shared_remaining: self.jumps_remaining,
            effective_shared_maximum: t.max_jumps,
            dt,
        }) else {
            return false;
        };
        self.last_jump_reset = effects.decision;
        let evaluated_before_projection = self.jump_lifecycle.properties.evaluated;
        if effects.remove_vertical_additive_motion {
            self.slide_impulse.remove_vertical_component();
            self.roll_impulse.remove_vertical_component();
        }
        let mut height_launched = false;
        if effects.landing_callback {
            let staged_after_projection = self.jump_lifecycle.properties.staged;
            // LAB Anim.v150=false, actor.v290=true, controller available. Keep
            // caller admission separate from style selection and later packet gates.
            if landing_callback_eligible(
                evaluated_before_projection,
                staged_after_projection,
                false,
            ) {
                let style = select_landing_style(
                    LandingInputs {
                        actor_allowed: true,
                        physics_available: true,
                        cached_vertical: self.landing_cached_vertical,
                        interrupted: self.landing_style_flag_c2b,
                        flag4: false,
                        // Current LAB input, not the earlier published contact sample.
                        movement: input.movement,
                        evaluated_jump: self.jump_lifecycle.properties.evaluated,
                    },
                    &t.landing,
                    &mut self.landing_rng,
                )
                .expect("finite validated lab landing inputs");
                // Selector precedes this current pending check, even when the
                // selected callback then dispatches an impulse instead of cleanup.
                match self.jump_lifecycle.landing_callback_action(style) {
                    LandingAction::JumpStartImpulse => {
                        height_launched = self.apply_lab_jump_start_impulse(t);
                    }
                    LandingAction::Cleanup(packet) => {
                        // LAB byte[AAD+style]=false and posture4=effective crouch.
                        if self.crouched {
                            self.slide_requested = false;
                            if world.can_occupy(self.position, t.radius, t.standing_height) {
                                self.crouched = false;
                            }
                        }
                        self.jump_lifecycle.apply_landing_cleanup(
                            packet,
                            LandingCleanupGate {
                                actor_flag_four: false,
                                inventory_blocks_styled_landing: false,
                            },
                        );
                    }
                }
                // Only the reached callback path clears this actor flag.
                self.landing_style_flag_c2b = false;
            }
        }
        // Native landing refills after callback/event effects, including a
        // synchronous pending launch. A skipped inner callback still refills.
        if effects.refill_budgets {
            self.jumps_remaining = t.max_jumps;
            self.bullet_jumps_left = t.max_bullet_jumps;
            self.air_rolls_left = t.max_air_rolls;
        }
        if effects.check_repair_posture_cleanup && self.crouched {
            // LAB maps actor state4 to effective crouch and chooses the native
            // AAD guard false. This is separate from clearing IV_LAND_STYLE.
            self.slide_requested = false;
            // Collision hull clearance is a LAB constraint. Input later in this
            // substep may request crouch again; native graph bindings are absent.
            if world.can_occupy(self.position, t.radius, t.standing_height) {
                self.crouched = false;
            }
        }
        if effects.physics_reset_callback {
            // Recovered vD8 clears working height-gravity suppression and calls
            // v90(0). LAB chooses mIgnoreVerticalVelocity=false (cfgCE); only
            // the working scalar maps to velocity.y. The LAB cache760 is retained
            // on this supported path; native published snapshots and vF8 callback
            // execution remain unmodeled. Additives survive.
            // This decision cannot also take the landing impulse route above.
            self.velocity.y = 0.;
        }
        // Native landing notification/flag callbacks remain partial. Deferred
        // ordinary events drain after the selected graph evaluation below.
        // Grace is unmodeled (no invented positive window). Property-four timing
        // remains a separate phase and observes evaluated, not pending, state.
        let _ = self.jump_lifecycle.advance_property_four_time(dt);
        self.jump_lifecycle.reset.commit_air_sample(sampled_air);
        let _ = self
            .jump_binding
            .promote(&mut self.jump_lifecycle.properties);
        height_launched
    }
    /// Stored slide contribution before collision clipping. It keeps decaying
    /// during wall latch even though the lab suppresses its motion there.
    pub fn slide_velocity(&self) -> Vec3 {
        self.slide_impulse.velocity()
    }
    /// Stored Motion-owner roll contribution, before lab collision clipping.
    /// Fixed world direction and the timer-driven finish event are lab policies.
    pub fn roll_velocity(&self) -> Vec3 {
        self.roll_impulse.velocity()
    }
    /// Composed requested velocity, not displacement-derived collision speed.
    /// Wall-latch suppression is an explicit policy of this original lab.
    pub fn combined_velocity(&self) -> Vec3 {
        if self.state == MovementState::WallLatch {
            Vec3::ZERO
        } else {
            self.velocity + self.slide_velocity() + self.roll_velocity()
        }
    }
    pub fn height(&self, tuning: &Tuning) -> f32 {
        if self.crouched {
            tuning.crouch_height
        } else {
            tuning.standing_height
        }
    }
    /// Bounded substeps reduce sensitivity to viewer frame rate. Jump and roll
    /// press edges are consumed once per call; collision remains swept.
    pub fn step(&mut self, input: &Input, dt: f32, world: &World, tuning: &Tuning) {
        if !dt.is_finite()
            || dt <= 0.
            || dt > 2.
            || !input.movement.is_finite()
            || !input.camera_forward.is_finite()
            || !tuning.is_valid()
        {
            return;
        }
        self.last_world_move_failed = false;
        self.wall_traversal.outcomes.clear();
        self.wall_traversal.error=None;
        if !input.jump_actions.valid() || (input.jump_edge_id.is_some() && (!input.jump || !input.jump_actions.authoritative)) || !self.wall_traversal.valid_for(world) {
            self.wall_traversal.error=Some("invalid wall input or persisted wall/world state".into());return;
        }
        if !self.jump_budget_initialized {
            self.jumps_remaining = tuning.max_jumps;
            self.bullet_jumps_left = tuning.max_bullet_jumps;
            self.air_rolls_left = tuning.max_air_rolls;
            self.jump_budget_initialized = true;
            self.landing_rng = tuning.landing.rng_seed;
        }
        let steps = (dt / MAX_SUBSTEP).ceil().max(1.) as u32;
        let h = dt / steps as f32;
        let mut sub_input = *input;
        for _ in 0..steps {
            self.tick(&sub_input, h, world, tuning);
            sub_input.jump = false;
            sub_input.jump_edge_id=None;
            sub_input.jump_actions.consumed();
            sub_input.roll = false;
        }
    }
    fn ordinary_jump_request(&mut self,input:&Input,crouch:bool,world:&World,t:&Tuning)
        -> (crate::wall_traversal::ResultKind,bool,bool) {
        use crate::wall_traversal::ResultKind;
        if !t.wall_traversal.input_enabled {return (ResultKind::InputDisabled,false,false);}
        if self.roll_remaining>0. {return (ResultKind::Roll,false,false);}
        // Actor core14032A2D0 reads evaluated D90(false), not staged1/pending.
        if self.jump_lifecycle.properties.evaluated==1 {return (ResultKind::EvaluatedProperty,false,false);}
        let crouched=crouch || !world.can_occupy(self.position,t.radius,t.standing_height);
        let movement=horizontal(input.movement).clamp_length_max(1.);
        let moving=movement.length_squared()>0.0001;
        let launch_gravity=t.launch_gravity.unwrap_or(t.gravity);
        let mut accepted_jump=false;let mut ordinary_requested=false;
        let mut height_launch=false;let mut bullet_scalar_written=false;
            if crouched && self.bullet_jumps_left > 0 && self.jumps_remaining > 0 {
                accepted_jump = true;
                self.jump_style = JumpStyle::Bullet;
                // Native normal requests select actor posture 4/5. Effective
                // crouch, including blocked standing clearance, is the lab mapping.
                self.jumps_remaining -= 1;
                // Visual action ends; native-derived signed timing history remains.
                self.wall_jump_remaining = 0.;
                self.bullet_jumps_left -= 1;
                // Native accepted bullet removes Input and Motion owners.
                // These are the slide and roll channels in this bounded lab.
                self.slide_impulse = DecayingImpulse::default();
                self.roll_impulse = DecayingImpulse::default();
                let direction = crate::bullet::bullet_direction(
                    input.camera_forward,
                    self.position,
                    input.aim_target,
                    !self.grounded,
                );
                let launch = direction * t.bullet_jump_speed;
                let target_vertical = launch.y * t.bullet_jump_vertical_scale;
                set_horizontal(&mut self.velocity, horizontal(launch));
                if (target_vertical < 0. && target_vertical < self.velocity.y)
                    || (target_vertical >= 0. && self.velocity.y < target_vertical)
                {
                    self.velocity.y = target_vertical;
                    bullet_scalar_written = true;
                }
                self.bullet_remaining = t.bullet_jump_duration;
                self.bullet_gravity_remaining = t.bullet_jump_duration;
                self.grounded = false;
                self.slide_requested = false;
            } else if self.grounded && self.jumps_remaining > 0 {
                accepted_jump = true;
                self.jumps_remaining -= 1;
                // Visual action ends; native-derived signed timing history remains.
                self.wall_jump_remaining = 0.;
                // Stage the request now; the graph queues the impulse and its
                // later completion phase applies physics. AIR_JUMP is distinct.
                // Tuning validation guarantees this request height is finite.
                self.jump_lifecycle
                    .request_deferred_height(t.jump_height)
                    .expect("validated finite lab jump height");
                ordinary_requested = true;
                self.jump_style = JumpStyle::Normal;
                self.slide_requested = false;
            } else if !self.grounded && self.jumps_remaining > 0 {
                accepted_jump = true;
                self.jump_style = JumpStyle::Double;
                self.jumps_remaining -= 1;
                // Visual action ends; native-derived signed timing history remains.
                self.wall_jump_remaining = 0.;
                // The native double-height path also removes the Input-owned
                // contribution before applying its launch packet.
                self.slide_impulse = DecayingImpulse::default();
                if moving {
                    let old = horizontal(self.velocity);
                    let direction = movement.normalize();
                    let alignment = if old.length_squared() < 1e-6 {
                        1.
                    } else {
                        old.normalize().dot(direction).max(0.)
                    };
                    set_horizontal(
                        &mut self.velocity,
                        old * alignment
                            + movement * t.physics_base_speed * t.double_jump_horizontal_gain,
                    );
                }
                self.velocity.y = self
                    .velocity
                    .y
                    .max((2. * launch_gravity * t.double_jump_height).sqrt());
                height_launch = true;
                self.bullet_remaining = 0.;
            }

        if accepted_jump {
            if !ordinary_requested {self.jump_lifecycle.immediate_lab_launch(height_launch,bullet_scalar_written);}
            self.jump_animation.restart_for_lab_request();
        }
        (if accepted_jump {ResultKind::Accepted}else{ResultKind::Budget},height_launch,bullet_scalar_written)
    }
    fn submit_wall_query(&mut self,input:&Input,world:&World,t:&Tuning,
        kind:crate::wall_traversal::Kind,id:Option<u64>,crouch:bool) {
        use crate::wall_traversal::{Provider,Ticket,ProviderStatus};
        if input.movement.y!=0. || t.wall_jump.old_school_wall_run {
            self.wall_traversal.last_provider=Some(ProviderStatus::Unsupported);
            self.wall_traversal.error=Some("unsupported wall caller/configuration".into());return;
        }
        if t.wall_traversal.provider==Provider::Refused {self.wall_traversal.last_provider=Some(ProviderStatus::Refused);return;}
        if t.wall_traversal.provider==Provider::Unsupported {
            self.wall_traversal.last_provider=Some(ProviderStatus::Unsupported);
            self.wall_traversal.error=Some("unsupported wall provider".into());return;
        }
        match crate::wall_query::submit(world,self.position,self.height(t),t.radius,input.camera_forward) {
            Ok(query)=> {self.wall_traversal.submitted+=1;
                self.wall_traversal.last_provider=Some(if t.wall_traversal.provider==Provider::Pending {ProviderStatus::Pending}else{ProviderStatus::Submitted});
                self.wall_traversal.ticket=Some(Ticket{kind,press_id:id,crouch,query,
                    pending:t.wall_traversal.provider==Provider::Pending});},
            Err(e)=>{self.wall_traversal.last_provider=Some(ProviderStatus::Unsupported);self.wall_traversal.error=Some(e.into());},
        }
    }
    fn deliver_wall_actions(&mut self,input:&Input,world:&World,t:&Tuning)->bool {
        use crate::wall_traversal::{Kind,OutcomeKind,ResultKind};
        let packet=input.jump_actions;
        if self.wall_traversal.authoritative.is_some_and(|v|v!=packet.authoritative) {
            self.wall_traversal.error=Some("transport mode changed without explicit boundary".into());return false;
        }
        self.wall_traversal.authoritative=Some(packet.authoritative);
        let mut actions=Vec::new();
        if packet.authoritative {
            if self.wall_traversal.next_id.is_some_and(|n|n!=packet.first_id) {
                self.wall_traversal.error=Some("unordered or missing wall action ID".into());return false;
            }
            self.wall_traversal.next_id=Some(packet.first_id+packet.len as u64);
            for i in 0..packet.len {
                actions.push((Some(packet.first_id+i as u64),packet.active_bits&(1u64<<i)!=0,
                    packet.crouch_bits&(1u64<<i)!=0));
            }
        } else {
            if input.jump {actions.push((None,true,input.crouch));}
            if !input.jump_held && (input.jump || self.wall_traversal.legacy_held) {
                actions.push((None,false,false));
            }
            self.wall_traversal.legacy_held=input.jump_held;
        }
        let mut legacy_handled=false;
        for (id,active,crouch) in actions {
            let enabled=t.wall_traversal.input_enabled && self.roll_remaining==0.;
            if !active || !enabled {self.wall_traversal.held=false;}
            else {self.wall_traversal.held=true;}
            if !active {continue;}
            let effective_crouch=crouch || !world.can_occupy(self.position,t.radius,t.standing_height);
            let handled=enabled && t.wall_traversal.wall_enabled && !self.grounded && !effective_crouch;
            if handled {
                if self.wall_traversal.ticket.is_none() {
                    self.submit_wall_query(input,world,t,Kind::Initial,id,crouch);
                }
                self.wall_traversal.outcome(OutcomeKind::WallHandled,id,crouch,ResultKind::WallHandled);
            } else {self.wall_traversal.retry_timer=t.wall_jump.normal_cooldown;}
            if let Some(id)=id {self.wall_traversal.dispositions.insert(id,crate::wall_traversal::Disposition{handled,crouch});}
            else {legacy_handled=handled;}
        }
        if !input.jump {return false;}
        if packet.authoritative {
            let Some(id)=input.jump_edge_id else {self.wall_traversal.error=Some("missing ordinary press identity".into());return false;};
            match self.wall_traversal.dispositions.remove(&id) {
                Some(d) if d.crouch==input.crouch=>!d.handled,
                Some(_)=>{self.wall_traversal.error=Some("ordinary posture differs from associated press".into());false},
                None=>{self.wall_traversal.error=Some("ordinary edge has no delivered press association".into());false},
            }
        } else {!legacy_handled}
    }
    fn perform_wall_launch(&mut self,input:&Input,normal:Vec3,t:&Tuning)
        ->(crate::wall_traversal::ResultKind,bool) {
        use crate::wall_traversal::ResultKind;
        if input.movement.y!=0. || t.wall_jump.old_school_wall_run {return (ResultKind::Unsupported,false);}
        let Some(launch)=wall_launch(WallJumpInput {
            old_velocity:horizontal(self.velocity),input_direction:input.movement,
            fallback_direction:forward_horizontal(input.camera_forward),wall_normal:normal,
            wall_tangent:horizontal(self.velocity).normalize_or_zero(),in_wall_state:self.state==MovementState::WallLatch,
            old_timer:self.wall_jump_timer,base_speed:t.physics_base_speed,base_jump_height:t.jump_height,
        },&t.wall_jump) else {return (ResultKind::LaunchTimer,false);};
        // Original ordering: clocks/vector/gravity publish before supplied actor admission.
        self.wall_jump_timer=launch.cooldown;self.wall_traversal.retry_timer=launch.cooldown;
        self.wall_traversal.published_vector=Some(launch.velocity);
        set_horizontal(&mut self.velocity,horizontal(launch.velocity));
        self.wall_gravity_scale=launch.gravity_scale;self.wall_gravity_remaining=launch.gravity_duration;
        // Same actor-core early gate precedes mode2 resource reset, after caller publications.
        if self.jump_lifecycle.properties.evaluated==1 {return (ResultKind::EvaluatedProperty,false);}
        if !t.wall_traversal.actor_admits_wall || self.roll_remaining>0. {return (ResultKind::Actor,false);}
        self.velocity.y=self.velocity.y.max((2.*t.launch_gravity.unwrap_or(t.gravity)*launch.height).sqrt());
        self.jump_style=JumpStyle::Normal;self.wall_jump_remaining=launch.cooldown;
        self.wall_jump_subtype=launch.subtype;self.wall_latch_remaining=0.;self.wall_latch_exhausted=false;
        self.bullet_remaining=0.;self.grounded=false;self.slide_requested=false;
        self.jumps_remaining=t.max_jumps.saturating_sub(1);self.bullet_jumps_left=t.max_bullet_jumps;
        self.air_rolls_left=t.max_air_rolls;
        self.jump_lifecycle.immediate_lab_launch(true,false);self.jump_animation.restart_for_lab_request();
        self.wall_traversal.launches+=1;
        (ResultKind::Accepted,true)
    }
    fn tick(&mut self, input: &Input, dt: f32, world: &World, t: &Tuning) {
        let published_support = self.grounded;
        let lifecycle_height_launch = self.advance_lab_jump_lifecycle(input, dt, world, t);
        // Native working launch flags are consumed by one preparation/snapshot.
        // These locals model that lifetime under the lab's input-before-physics
        // schedule; full native mode/contact gates remain outside this subset.
        // A synchronous landing callback can launch before input. Do not erase
        // that working gravity-suppression marker when starting preparation.
        let mut height_launch = lifecycle_height_launch;
        let mut bullet_scalar_written = false;
        self.time += dt;
        let roll_was_active = self.roll_remaining > 0.;
        self.roll_remaining = (self.roll_remaining - dt).max(0.);
        if roll_was_active && self.roll_remaining == 0. {
            // Native DodgeRollFinished removes the Motion owner. Scheduling
            // that event at this timer boundary is an explicit lab assumption.
            self.roll_impulse = DecayingImpulse::default();
        }
        self.bullet_remaining = (self.bullet_remaining - dt).max(0.);
        self.wall_jump_remaining = (self.wall_jump_remaining - dt).max(0.);
        if !t.wall_traversal.competing_move && self.wall_jump_timer < f32::MAX {
            self.wall_jump_timer -= dt;
        }
        let was_grounded = self.grounded;
        let movement = horizontal(input.movement).clamp_length_max(1.);
        let moving = movement.length_squared() > 0.0001;
        let crouch_pressed = input.crouch && !self.previously_crouch;
        self.previously_crouch = input.crouch;
        if input.crouch {
            self.crouched = true;
        } else if world.can_occupy(self.position, t.radius, t.standing_height) {
            self.crouched = false;
        }
        if !input.crouch {
            self.slide_requested = false;
        }
        // A fresh crouch while moving fast requests slide; release under a
        // ceiling keeps the short collision hull until there is clearance.
        if crouch_pressed
            && self.roll_remaining == 0.
            && ((self.grounded
                && horizontal(self.velocity + self.slide_velocity() + self.roll_velocity())
                    .length()
                    > t.crouch_speed + 0.5)
                || (!self.grounded && input.sprint && moving))
        {
            self.slide_requested = true;
            if self.time - self.last_slide_boost >= t.slide_boost_cooldown {
                let direction = if moving {
                    movement.normalize()
                } else {
                    horizontal(self.velocity + self.slide_velocity() + self.roll_velocity())
                        .normalize_or_zero()
                };
                // Native slide replaceAll clears other owners only when the
                // boost inserts; a cooldown-rejected request does not clear roll.
                self.roll_impulse = DecayingImpulse::default();
                self.slide_impulse
                    .replace(direction * t.slide_boost, t.slide_impulse_decay);
                self.last_slide_boost = self.time;
            }
        }
        // Fresh records contribute fully on their first evaluation. Later
        // evaluations decay independently of movement state and base velocity.
        self.slide_impulse.evaluate(dt);
        let contact = world.wall_contact(self.position, t.radius, self.height(t), 0.18);
        self.wall_normal = contact;
        if input.roll
            && self.roll_remaining == 0.
            && (self.grounded
                || (self.air_rolls_left > 0
                    && (!t.air_roll_consumes_jump || self.jumps_remaining > 0)))
        {
            if !self.grounded {
                self.air_rolls_left -= 1;
                if t.air_roll_consumes_jump {
                    // Lab maps the native PM4 activation/flag8000 consumption
                    // to accepted airborne roll. Native bypasses are unmodeled.
                    self.jumps_remaining -= 1;
                }
            }
            self.jump_lifecycle.immediate_lab_dodge(!self.grounded);
            self.roll_direction = if moving {
                movement.normalize()
            } else {
                forward_horizontal(input.camera_forward)
            };
            self.roll_remaining = t.roll_duration;
            self.wall_latch_remaining = 0.;
            self.slide_requested = false;
            // Original event-at-input scheduling with fixed world direction.
            // Native existing-owner updates preserve evaluated/captured decay.
            self.roll_impulse
                .upsert(self.roll_direction * t.roll_speed, t.roll_impulse_decay);
            if self.roll_remaining == 0. {
                // Zero-duration tuning emits an immediate modeled finish event.
                self.roll_impulse = DecayingImpulse::default();
            }
        }
        self.roll_impulse.evaluate(dt);
        use crate::wall_traversal::{Kind,OutcomeKind,ResultKind};
        let prior_ticket=self.wall_traversal.ticket.is_some();
        let ordinary_edge=self.deliver_wall_actions(input,world,t);
        let mut wall_route=false;
        if prior_ticket && !t.wall_traversal.competing_move {
            if let Some(ticket)=self.wall_traversal.ticket.clone() {
                let selection=if ticket.pending {crate::wall_query::Selection::Pending}
                    else {crate::wall_query::select(&ticket.query.records)};
                match selection {
                    crate::wall_query::Selection::Pending=>{},
                    crate::wall_query::Selection::NoHit=> {
                        self.wall_traversal.ticket=None;
                        if ticket.kind==Kind::Initial {
                            let (result,h,b)=self.ordinary_jump_request(input,ticket.crouch,world,t);
                            height_launch|=h;bullet_scalar_written|=b;
                            if result==ResultKind::Accepted {self.wall_traversal.retry_timer=t.wall_jump.normal_cooldown;}
                            self.wall_traversal.outcome(OutcomeKind::InitialFallback,ticket.press_id,ticket.crouch,result);
                        }
                    },
                    crate::wall_query::Selection::Hit(contact)=> {
                        self.wall_traversal.ticket=None;self.wall_traversal.selected=Some(contact);
                        wall_route=true;self.wall_normal=Some(contact.normal);
                        let (result,h)=self.perform_wall_launch(input,contact.normal,t);
                        height_launch|=h;
                        self.wall_traversal.outcome(OutcomeKind::WallLaunch,ticket.press_id,ticket.crouch,result);
                    }
                }
            }
        }
        if ordinary_edge {
            let (result,h,b)=self.ordinary_jump_request(input,input.crouch,world,t);
            height_launch|=h;bullet_scalar_written|=b;
            self.wall_traversal.outcome(OutcomeKind::Ordinary,input.jump_edge_id,input.crouch,result);
        }
        // A ready wall route returns before the native retry tail, even if launch rejects.
        if !wall_route && !t.wall_traversal.competing_move {
            let primary=self.wall_traversal.held && (!self.grounded || self.jump_lifecycle.deferred_pending)
                && self.wall_traversal.retry_timer<f32::MAX;
            if !primary {self.wall_traversal.retry_timer=f32::MAX;}
            else {
                if self.wall_traversal.retry_timer>0. {self.wall_traversal.retry_timer-=dt;}
                if t.wall_traversal.wall_enabled && !t.wall_jump.old_school_wall_run
                    && self.wall_traversal.ticket.is_none() && self.wall_traversal.retry_timer<=0. {
                    self.submit_wall_query(input,world,t,Kind::Retry,None,input.crouch);
                }
            }
        }
        height_launch |= self.evaluate_lab_jump_graph(input, t);
        if !input.aim || self.grounded || contact.is_none() {
            self.wall_latch_exhausted = false;
        }
        let mut latch = !self.grounded
            && input.aim
            && self.roll_remaining == 0.
            && self.wall_jump_remaining == 0.
            && contact.is_some()
            && self.aim_resource > 0.
            && !self.wall_latch_exhausted;
        if latch && self.state != MovementState::WallLatch {
            // LAB aim/contact requests preparation at a new entry only. Native
            // traversalBBC, cached posture20..22 and frozen latch are distinct.
            // No opaque service predicate, backend or native cadence is inferred.
            let entry = prepare_wall_entry(
                WallEntryInput {
                    base_velocity: horizontal(self.velocity),
                    movement,
                    wall_normal: contact.expect("latch request requires side contact"),
                    service_rejects: false,
                    geometry_available: true,
                    physics_available: true,
                },
                t.wall_entry,
            );
            if entry.is_some_and(|entry| entry.remove_bullet_gravity) {
                // Native removal occurs after early guards but BEFORE later
                // direction rejection. Failed preparation keeps this effect.
                self.bullet_gravity_remaining = 0.;
            }
            latch = entry.is_some_and(|entry| entry.direction.is_some());
        }
        if latch {
            if self.state != MovementState::WallLatch {
                self.wall_latch_remaining = t.wall_latch_duration;
                // Frozen movement/resource/timeout after this finite direction
                // admission remains LAB, not the native genericBBC traversal.
            }
            self.wall_latch_remaining = (self.wall_latch_remaining - dt).max(0.);
            if self.wall_latch_remaining == 0. {
                self.wall_latch_exhausted = true;
            }
        } else {
            self.wall_latch_remaining = 0.;
        }
        let mut latch_active = latch && self.wall_latch_remaining > 0.;
        let mut aim_active = !self.grounded
            && input.aim
            && self.roll_remaining == 0.
            && !latch_active
            && self.aim_resource > 0.;
        if latch_active || aim_active {
            let duration = if latch_active {
                t.wall_latch_duration
            } else {
                t.aim_duration
            };
            let consumption = if latch_active { dt * 0.5 } else { dt };
            // Native first entry does not drain. Already-active updates drain
            // before checking exhaustion, including the exit update.
            if self.aim_was_active {
                self.aim_resource = (self.aim_resource - consumption / duration).max(0.);
            }
            if self.aim_resource <= 0. {
                latch_active = false;
                aim_active = false;
            } else if !self.aim_was_active {
                // Separate scalar only: do not scale the additive slide vector.
                // Native setter enabled/stats unmodified are lab assumptions.
                self.velocity.y = crate::gravity::aim_entry_vertical(
                    self.velocity.y,
                    self.aim_resource,
                    t.aim_entry_vertical_scale,
                );
            }
        } else if self.aim_was_active {
            // An active native update consumes even when eligibility is lost.
            let duration = if self.state == MovementState::WallLatch {
                t.wall_latch_duration
            } else {
                t.aim_duration
            };
            let consumption = if self.state == MovementState::WallLatch {
                dt * 0.5
            } else {
                dt
            };
            self.aim_resource = (self.aim_resource - consumption / duration).max(0.);
        }
        if !latch_active && !aim_active {
            if self.aim_was_active {
                self.aim_resource = (self.aim_resource - t.aim_exit_penalty).max(0.);
            } else {
                // Holding aim at empty airborne resource inhibits recharge to avoid
                // charge-sliver reactivation. This guard is an original approximation.
                if self.grounded || !input.aim {
                    self.aim_resource = (self.aim_resource + dt * t.aim_recharge).min(1.);
                }
            }
        }
        self.aim_was_active = latch_active || aim_active;
        if latch_active {
            self.velocity = Vec3::ZERO;
        } else if self.grounded && self.slide_requested && input.crouch {
            let old = horizontal(self.velocity);
            let accel_drag = t.ground_accel_drag.unwrap_or(t.turn_rate);
            // Native A0 selects slide braking and forces analog=1. On flat
            // support the slope target is zero; the ordinary kernel brakes
            // base velocity before the additive record is composed. Posture 5
            // does not imply input style 5: retain a selected jog/sprint cap.
            // Mapping that selection to this lab's sprint input, grounding and
            // unit friction stat remains approximate. Support mask and published
            // normal below remain separate from actor-grounded admission.
            let selected_cap = if input.sprint {
                t.sprint_speed
            } else {
                t.walk_speed
            };
            let value = if accel_drag > 0. {
                // Native v60 is signed downward gravity, sampled before the
                // later preparation ages modifiers. These named channels and
                // prior-publication timing remain explicit LAB bindings.
                let signed_gravity = -t.gravity
                    * crate::gravity::modifier_factor(t.bullet_jump_gravity_scale,
                        self.bullet_gravity_remaining, t.gravity_modifier_fade_window)
                    * crate::gravity::modifier_factor(self.wall_gravity_scale,
                        self.wall_gravity_remaining, t.gravity_modifier_fade_window);
                let normal = self.engine_contact.working_normal;
                let selected = crate::slide_slope::select(
                    self.engine_contact.mask, true,
                    Vec3::new(normal[0], normal[1], normal[2]), signed_gravity,
                    t.slide_slope_acceleration, movement,
                    t.ground_acceleration, accel_drag,
                );
                accelerate_velocity(
                    old,
                    selected.target,
                    dt,
                    VelocityTuning {
                        acceleration: selected.acceleration,
                        turn_rate: selected.turn_rate,
                        deceleration: t.slide_drag,
                        max_speed: selected_cap,
                        angle_cutoff_degrees: 110.,
                    },
                )
            } else {
                // The native zero-AccelDrag branch bypasses slope/braking and
                // scales the input target directly. World-space input and the
                // selected cap here retain the lab's ordinary-ground mapping.
                // No input keeps native working150; lab zero assumes the
                // ordinary snapshot/reset path left that working vector zero.
                movement * selected_cap
            };
            set_horizontal(&mut self.velocity, value);
            // State admission/end remain lab approximations. Their speed
            // observation includes the separate additive contribution.
            if horizontal(self.velocity + self.slide_velocity() + self.roll_velocity()).length()
                < t.slide_end_speed
            {
                self.slide_requested = false;
            }
        } else {
            let chosen_speed = if self.crouched && self.grounded {
                t.crouch_speed
            } else if input.sprint {
                t.sprint_speed
            } else {
                t.walk_speed
            };
            let old = horizontal(self.velocity);
            // The native ordinary path supplies its input-scaled target cap in
            // air too; the numeric helper gradually damps existing excess speed.
            let max_speed = chosen_speed * movement.length();
            let accel_drag = if self.grounded {
                t.ground_accel_drag
            } else {
                t.air_accel_drag
            }
            .unwrap_or(t.turn_rate);
            // Continuing ordinary base updates during a roll is a lab policy;
            // the recovered roll event adds velocity without overwriting base.
            // Lab grounded selects this ordinary coefficient pair. Native mode
            // gates, its direct-target bypass for zero coefficients, and contact
            // factor/slope composition remain outside this approximation.
            let value = accelerate_velocity(
                old,
                movement,
                dt,
                VelocityTuning {
                    acceleration: if self.grounded {
                        t.ground_acceleration
                    } else {
                        t.air_acceleration
                    },
                    turn_rate: accel_drag,
                    deceleration: if self.grounded {
                        t.ground_drag
                    } else {
                        t.air_decel_drag
                    },
                    max_speed,
                    angle_cutoff_degrees: 110.,
                },
            );
            set_horizontal(&mut self.velocity, value);
        }
        // Native preparation ages timed modifiers before scalar integration.
        // Age a newly inserted/replaced bullet modifier on this step too. The
        // lab still approximates the ordering of input versus native scheduling.
        self.bullet_gravity_remaining = (self.bullet_gravity_remaining - dt).max(0.);
        if self.wall_gravity_remaining >= 0. {
            self.wall_gravity_remaining = (self.wall_gravity_remaining - dt).max(0.);
        }
        if !self.grounded && !latch_active {
            // Recovered timed fade/product, with approximate aim eligibility and
            // modifier insertion/removal policies supplied by this lab controller.
            let bullet_factor = crate::gravity::modifier_factor(
                t.bullet_jump_gravity_scale,
                self.bullet_gravity_remaining,
                t.gravity_modifier_fade_window,
            );
            let wall_factor = crate::gravity::modifier_factor(
                self.wall_gravity_scale,
                self.wall_gravity_remaining,
                t.gravity_modifier_fade_window,
            );
            let aim_factor = if aim_active {
                crate::gravity::aim_gravity_factor(self.velocity.y, t.aim_gravity_scale)
            } else {
                1.
            };
            // Height launch always sets the skip flag, even when preserving a
            // stronger scalar or launching to zero height. Direct bullet writes
            // mark it only when the target replaces the scalar and preparation
            // sees positive Y. Modeled bullet additive owners are already clear.
            let skip_launch_gravity =
                height_launch || (bullet_scalar_written && self.velocity.y > 0.);
            if !skip_launch_gravity {
                self.velocity.y -= t.gravity * bullet_factor * wall_factor * aim_factor * dt;
            }
        } else if self.grounded {
            self.velocity.y = -2.;
        }
        // LAB models the last enabled-setter sample while its predicate is air,
        // using support published before this tick's events. Grounded writes
        // retain the prior cache. This is not the final clipped impact velocity,
        // a peak downward speed, or proof of native vF8/contact equivalence.
        if !published_support {
            self.landing_cached_vertical = self.velocity.y;
        }
        let requested_velocity = if latch_active {
            Vec3::ZERO
        } else {
            Vec3::new(self.velocity.x, 0., self.velocity.z)
                + self.slide_velocity()
                + self.roll_velocity()
        };
        let height = self.height(t);
        let motion = world.move_engine_capsule(
            &mut self.engine_contact,
            self.position,
            requested_velocity,
            if latch_active { 0. } else { self.velocity.y },
            height_launch || (bullet_scalar_written && self.velocity.y > 0.),
            self.jump_lifecycle.reset.cached_air,
            dt,
            t.radius,
            height,
            &t.collision,
        );
        self.last_world_move_failed |= motion.failed;
        self.position = motion.position;
        // The reviewed native chain solves position separately from prepared
        // motion; it does not project persistent velocity through every normal.
        // Stopping on an unsupported/invalid solve remains explicit LAB policy.
        if motion.failed {
            self.velocity = Vec3::ZERO;
        }
        self.grounded = motion.grounded;
        self.wall_normal = motion.wall_normal.or(contact);
        if self.grounded {
            // Physical contact cleanup is separate from lifecycle budget reset.
            self.bullet_remaining = 0.;
            self.wall_latch_remaining = 0.;
            self.wall_jump_remaining = 0.;
            self.wall_latch_exhausted = false;
            if !was_grounded
                && input.crouch
                && horizontal(self.velocity + self.slide_velocity() + self.roll_velocity()).length()
                    > t.slide_end_speed
            {
                self.slide_requested = true;
            }
        }
        // Native BEC is shared by every launch; walking off an edge preserves
        // it. The separate native BF4 grace window is not modeled here.
        self.air_jumps_left = if self.grounded {
            self.jumps_remaining.saturating_sub(1)
        } else {
            self.jumps_remaining
        };
        self.state = if self.roll_remaining > 0. {
            MovementState::Roll
        } else if self.grounded && self.slide_requested {
            MovementState::Slide
        } else if self.grounded && self.crouched {
            MovementState::Crouch
        } else if self.grounded && moving && input.sprint {
            MovementState::Sprint
        } else if self.grounded && moving {
            MovementState::Walk
        } else if self.grounded {
            MovementState::Idle
        } else if latch_active {
            MovementState::WallLatch
        } else if self.wall_jump_remaining > 0. {
            MovementState::WallJump
        } else if aim_active {
            MovementState::AimGlide
        } else if self.bullet_remaining > 0. {
            MovementState::BulletJump
        } else {
            MovementState::Air
        };
    }
}

fn horizontal(v: Vec3) -> Vec3 {
    Vec3::new(v.x, 0., v.z)
}
fn set_horizontal(v: &mut Vec3, value: Vec3) {
    v.x = value.x;
    v.z = value.z;
}
fn forward_horizontal(v: Vec3) -> Vec3 {
    let flat = horizontal(v).normalize_or_zero();
    if flat.length_squared() < 0.1 {
        -Vec3::Z
    } else {
        flat
    }
}

#[cfg(test)]
mod tests {
    // Explicit coefficient-test configuration; no production or legacy fast path.
    fn ordinary_only(mut t:Tuning)->Tuning {t.wall_traversal.wall_enabled=false;t}
    fn finish_query(p:&mut Player,input:&Input,w:&World,t:&Tuning) {
        assert!(p.wall_traversal.ticket.is_some(),"actual initial provider ticket required");
        p.step(&Input{jump:false,jump_held:false,..*input},MAX_SUBSTEP,w,t);
        assert!(p.wall_traversal.ticket.is_none(),"static request consumed on following substep");
    }

    use super::*;
    use crate::Solid;
    fn floor() -> World {
        World {
            terrain: Default::default(),
            solids: vec![Solid::new(
                Vec3::new(-100., -2., -100.),
                Vec3::new(100., 0., 100.),
            )],
        }
    }
    fn settled() -> Player {
        let mut p = Player::new(Vec3::new(0., 0.01, 0.));
        p.step(&Input::default(), 0.1, &floor(), &Tuning::default());
        assert!(p.grounded);
        p
    }

    #[test]
    fn landing_staged_one_skips_callback_but_refills_outer_budgets() {
        let mut p = settled();
        p.jump_lifecycle.reset.cached_air = true;
        p.jump_lifecycle.properties = crate::jump_lifecycle::JumpProperties {
            evaluated: 1,
            staged: 1,
        };
        p.jump_lifecycle.staged_land_style = 3.;
        p.landing_cached_vertical = -20.;
        p.landing_style_flag_c2b = true;
        p.jumps_remaining = 0;
        p.advance_lab_jump_lifecycle(&Input::default(), MAX_SUBSTEP, &floor(), &Tuning::default());
        assert_eq!(p.last_jump_reset(), ResetDecision::Landing);
        assert_eq!(p.jump_lifecycle.properties.staged, 1);
        assert_eq!(p.jump_lifecycle.staged_land_style, 3.);
        assert_eq!(p.landing_rng, 0);
        assert!(p.landing_style_flag_c2b);
        assert_eq!(p.jumps_remaining, 2);
    }

    #[test]
    fn landing_cleanup_selects_from_cached_scalar_and_promotes_four() {
        let mut p = settled();
        p.jump_lifecycle.reset.cached_air = true;
        p.jump_lifecycle.properties = crate::jump_lifecycle::JumpProperties {
            evaluated: 3,
            staged: 3,
        };
        p.landing_cached_vertical = -20.;
        p.velocity.y = 0.;
        p.advance_lab_jump_lifecycle(&Input::default(), MAX_SUBSTEP, &floor(), &Tuning::default());
        assert_eq!(p.jump_lifecycle.staged_land_style, 1.);
        assert_eq!(p.jump_lifecycle.properties.evaluated, 4);
        assert_eq!(p.landing_rng, 0x1405_7b7e_f767_814f);
        assert!(!p.jump_lifecycle.reset.suppress_once);
    }

    #[test]
    fn pending_landing_launch_keeps_gravity_skip_and_refills_after_event() {
        let t = Tuning::default();
        let mut p = settled();
        p.jump_lifecycle.reset.cached_air = true;
        p.jump_lifecycle.properties = crate::jump_lifecycle::JumpProperties {
            evaluated: 3,
            staged: 3,
        };
        p.jump_lifecycle.deferred_pending = true;
        p.jump_lifecycle.deferred_height = 2.;
        p.jump_lifecycle.staged_land_style = 3.;
        p.landing_cached_vertical = -20.;
        p.jumps_remaining = 0;
        p.bullet_jumps_left = 0;
        p.air_rolls_left = 0;
        p.step(&Input::default(), MAX_SUBSTEP, &floor(), &t);
        assert_eq!(p.velocity.y, (2. * t.gravity * 2.).sqrt());
        assert!(!p.grounded);
        assert_eq!(
            p.jump_lifecycle.staged_land_style, 3.,
            "pending event skips cleanup packet"
        );
        assert_eq!(p.jump_lifecycle.properties.staged, 3);
        assert!(!p.jump_lifecycle.deferred_pending);
        assert!(p.jump_lifecycle.reset.suppress_once);
        assert_eq!(
            p.landing_rng, 0x1405_7b7e_f767_814f,
            "selector runs before pending check"
        );
        assert_eq!(
            (p.jumps_remaining, p.bullet_jumps_left, p.air_rolls_left),
            (t.max_jumps, t.max_bullet_jumps, t.max_air_rolls)
        );
    }

    #[test]
    fn landing_cache_uses_last_air_scalar_before_collision_and_retains_on_support() {
        let t = Tuning::default();
        let mut p = Player::new(Vec3::Y * 10.);
        p.velocity.y = 3.;
        p.step(&Input::default(), MAX_SUBSTEP, &floor(), &t);
        assert!(p.landing_cached_vertical > 0.);
        assert_eq!(p.landing_cached_vertical, p.velocity.y);
        p.velocity.y = 1.;
        p.step(&Input::default(), MAX_SUBSTEP, &floor(), &t);
        assert!(p.landing_cached_vertical < 1., "cache is not a peak");
        p.position.y = 0.001;
        p.velocity.y = -20.;
        p.step(&Input::default(), MAX_SUBSTEP, &floor(), &t);
        assert!(p.grounded);
        let cached = p.landing_cached_vertical;
        assert_eq!(
            p.velocity.y, cached,
            "position solving preserves the prepared scalar"
        );
        assert!(cached < -20., "cache samples the prepared downward scalar");
        p.step(&Input::default(), MAX_SUBSTEP, &floor(), &t);
        assert_eq!(p.landing_cached_vertical, cached);
    }

    #[test]
    fn landing_legacy_state_defaults_and_tuning_reload_preserves_rng() {
        let mut p = settled();
        let mut value = serde_json::to_value(&p).unwrap();
        for key in [
            "landing_cached_vertical",
            "landing_rng",
            "landing_style_flag_c2b",
        ] {
            value.as_object_mut().unwrap().remove(key);
        }
        value["jump_lifecycle"]
            .as_object_mut()
            .unwrap()
            .remove("staged_dodge");
        let restored: Player = serde_json::from_value(value).unwrap();
        assert_eq!(restored.landing_cached_vertical, 0.);
        assert_eq!(restored.landing_rng, 0);
        assert!(!restored.landing_style_flag_c2b);
        assert_eq!(restored.jump_lifecycle.staged_dodge, 0.);
        let mut tuning_value = serde_json::to_value(Tuning::default()).unwrap();
        tuning_value.as_object_mut().unwrap().remove("landing");
        let mut t: Tuning = serde_json::from_value(tuning_value).unwrap();
        assert_eq!(t.landing, LandingTuning::default());
        p.landing_rng = 123;
        t.landing.rng_seed = 999;
        p.step(&Input::default(), MAX_SUBSTEP, &floor(), &t);
        assert_eq!(p.landing_rng, 123);
        p.respawn(Vec3::Y);
        p.step(&Input::default(), MAX_SUBSTEP, &floor(), &t);
        assert_eq!(p.landing_rng, 999);
    }

    #[test]
    fn stable_support_preserves_partial_budgets_and_defers_tuning_reload() {
        let mut p = settled();
        p.jumps_remaining = 1;
        p.bullet_jumps_left = 0;
        p.air_rolls_left = 0;
        let edited = Tuning {
            max_jumps: 4,
            max_bullet_jumps: 3,
            max_air_rolls: 2,
            ..Default::default()
        };
        p.step(&Input::default(), 0.5, &floor(), &edited);
        assert!(p.grounded);
        assert_eq!(p.last_jump_reset(), ResetDecision::None);
        assert_eq!(
            (p.jumps_remaining, p.bullet_jumps_left, p.air_rolls_left),
            (1, 0, 0)
        );
    }

    #[test]
    fn published_landing_refills_at_next_lifecycle_and_only_once() {
        let t = Tuning::default();
        let w = floor();
        let mut p = Player::new(Vec3::Y);
        p.step(&Input::default(), MAX_SUBSTEP, &w, &t);
        p.jumps_remaining = 0;
        p.bullet_jumps_left = 0;
        p.air_rolls_left = 0;
        p.position = Vec3::Y * 0.001;
        p.velocity = Vec3::NEG_Y;
        p.step(&Input::default(), MAX_SUBSTEP, &w, &t);
        assert!(p.grounded);
        assert_eq!(p.last_jump_reset(), ResetDecision::None);
        assert_eq!(
            (p.jumps_remaining, p.bullet_jumps_left, p.air_rolls_left),
            (0, 0, 0)
        );
        p.step(&Input::default(), MAX_SUBSTEP, &w, &t);
        assert_eq!(p.last_jump_reset(), ResetDecision::Landing);
        assert_eq!(
            (p.jumps_remaining, p.bullet_jumps_left, p.air_rolls_left),
            (2, 1, 1)
        );
        p.jumps_remaining = 1;
        p.bullet_jumps_left = 0;
        p.air_rolls_left = 0;
        p.step(&Input::default(), MAX_SUBSTEP, &w, &t);
        assert_eq!(p.last_jump_reset(), ResetDecision::None);
        assert_eq!(
            (p.jumps_remaining, p.bullet_jumps_left, p.air_rolls_left),
            (1, 0, 0)
        );
    }

    #[test]
    fn immediate_dive_contact_loses_suppressed_landing_then_graph_fall_allows_repair() {
        let t = Tuning::default();
        let w = floor();
        let mut p = Player::new(Vec3::Y * 0.05);
        p.step(
            &Input {
                jump: true,
                crouch: true,
                camera_forward: Vec3::NEG_Y,
                ..Default::default()
            },
            MAX_SUBSTEP,
            &w,
            &t,
        );
        assert!(p.grounded);
        assert_eq!((p.jumps_remaining, p.bullet_jumps_left), (1, 0));
        assert_eq!(p.jump_lifecycle.properties.evaluated, 2);
        assert_eq!(p.jump_lifecycle.properties.staged, 2);
        p.step(&Input::default(), MAX_SUBSTEP, &w, &t);
        assert_eq!(p.last_jump_reset(), ResetDecision::Suppressed);
        assert!(!p.jump_lifecycle.reset.cached_air);
        // LAB supported clip becomes inactive. Its queued impulse has no
        // pending request, so only graph Falling is published, with no relaunch.
        assert_eq!(p.jump_lifecycle.properties.evaluated, 3);
        assert_eq!((p.jumps_remaining, p.bullet_jumps_left), (1, 0));
        assert!(p.grounded);
        p.step(&Input::default(), MAX_SUBSTEP, &w, &t);
        assert_eq!(p.last_jump_reset(), ResetDecision::StationaryRepair);
        assert_eq!((p.jumps_remaining, p.bullet_jumps_left), (2, 1));
        assert_eq!(p.jump_lifecycle.properties.evaluated, 4);
    }

    #[test]
    fn zero_height_single_charge_does_not_refill_on_supported_launch() {
        let mut p = settled();
        p.jumps_remaining = 1;
        let t = Tuning {
            max_jumps: 1,
            jump_height: 0.,
            ..Default::default()
        };
        p.step(
            &Input {
                jump: true,
                ..Default::default()
            },
            MAX_SUBSTEP,
            &floor(),
            &t,
        );
        assert_eq!(p.jumps_remaining, 0);
        assert_eq!(p.jump_lifecycle.properties.staged, 3);
        assert!(!p.jump_lifecycle.deferred_pending);
        assert_eq!(p.jump_lifecycle.deferred_height, -1.);
        assert!(p.jump_lifecycle.reset.suppress_once);
        p.step(&Input::default(), MAX_SUBSTEP, &floor(), &t);
        assert_eq!(p.last_jump_reset(), ResetDecision::Suppressed);
        assert_eq!(p.jumps_remaining, 0);
    }

    #[test]
    fn supported_evaluated_dodge_repairs_without_an_air_edge() {
        let mut p = settled();
        p.jumps_remaining = 1;
        p.air_rolls_left = 0;
        p.jump_lifecycle.properties.staged = 3;
        p.step(&Input::default(), MAX_SUBSTEP, &floor(), &Tuning::default());
        assert_eq!(p.last_jump_reset(), ResetDecision::None);
        assert_eq!(p.air_rolls_left, 0);
        p.step(&Input::default(), MAX_SUBSTEP, &floor(), &Tuning::default());
        assert_eq!(p.last_jump_reset(), ResetDecision::StationaryRepair);
        assert_eq!(p.jump_lifecycle.properties.evaluated, 4);
        assert_eq!((p.jumps_remaining, p.air_rolls_left), (2, 1));
    }

    #[test]
    fn landing_projects_both_additive_channels_only_for_evaluated_jump_property() {
        for property in [0, 1, 2, 3, 4] {
            let mut p = settled();
            p.jump_lifecycle.reset.cached_air = true;
            p.jump_lifecycle.properties.evaluated = property;
            p.slide_impulse.replace(Vec3::new(3., 4., 0.), 2.);
            p.roll_impulse.replace(Vec3::new(0., 4., 3.), 2.);
            p.slide_impulse.evaluate(0.25);
            p.roll_impulse.evaluate(0.25);
            p.advance_lab_jump_lifecycle(&Input::default(), 0.25, &floor(), &Tuning::default());
            let expected_y = if matches!(property, 1..=3) { 0. } else { 4. };
            assert_eq!(p.slide_velocity(), Vec3::new(3., expected_y, 0.));
            assert_eq!(p.roll_velocity(), Vec3::new(0., expected_y, 3.));
            p.slide_impulse.evaluate(0.25);
            p.roll_impulse.evaluate(0.25);
            let expected_length = if expected_y == 0. { 2.5 } else { 4.5 };
            assert!((p.slide_velocity().length() - expected_length).abs() < 1e-5);
            assert!((p.roll_velocity().length() - expected_length).abs() < 1e-5);
        }
    }

    #[test]
    fn lifecycle_roundtrip_legacy_defaults_and_respawn_are_explicit() {
        let mut p = settled();
        p.jump_lifecycle.reset.suppress_once = true;
        p.jump_lifecycle.reset.tracked_seconds = 0.75;
        p.jump_lifecycle.properties.staged = 3;
        p.jump_lifecycle.properties.evaluated = 1;
        p.jump_lifecycle.staged_land_style = 3.;
        let encoded = serde_json::to_value(&p).unwrap();
        let restored: Player = serde_json::from_value(encoded.clone()).unwrap();
        assert_eq!(restored.jump_lifecycle, p.jump_lifecycle);
        let mut old_lifecycle = encoded.clone();
        old_lifecycle["jump_lifecycle"]
            .as_object_mut()
            .unwrap()
            .remove("staged_land_style");
        let migrated: Player = serde_json::from_value(old_lifecycle).unwrap();
        assert_eq!(migrated.jump_lifecycle.staged_land_style, 0.);
        assert_eq!(
            migrated.jump_lifecycle.properties,
            p.jump_lifecycle.properties
        );
        let mut legacy = encoded;
        legacy.as_object_mut().unwrap().remove("jump_lifecycle");
        legacy.as_object_mut().unwrap().remove("last_jump_reset");
        let restored: Player = serde_json::from_value(legacy).unwrap();
        assert_eq!(restored.jump_lifecycle, JumpLifecycle::default());
        assert_eq!(restored.last_jump_reset(), ResetDecision::None);
        p.respawn(Vec3::Y);
        assert_eq!(p.jump_lifecycle, JumpLifecycle::default());
        assert_eq!(p.last_jump_reset(), ResetDecision::None);
    }

    #[test]
    fn invalid_collision_tuning_cannot_consume_input_or_mutate_player() {
        for field in 0..2 {
            for value in [f32::NAN, f32::INFINITY, -0.01] {
                let mut tuning = Tuning::default();
                if field == 0 {
                    tuning.collision.contact_offset = value;
                } else {
                    tuning.collision.step_offset = value;
                }
                assert!(!tuning.is_valid());
                let mut player = settled();
                let before = serde_json::to_value(&player).unwrap();
                player.step(
                    &Input {
                        jump: true,
                        crouch: true,
                        ..Default::default()
                    },
                    MAX_SUBSTEP,
                    &floor(),
                    &tuning,
                );
                assert_eq!(serde_json::to_value(&player).unwrap(), before);
            }
        }
    }

    #[test]
    fn explicit_deferred_reset_zeros_only_working_vertical_scalar() {
        let mut p = settled();
        p.velocity = Vec3::new(3., 17., -5.);
        p.slide_impulse.replace(Vec3::new(2., 4., 0.), 2.);
        p.roll_impulse.replace(Vec3::new(0., 3., 6.), 3.);
        p.bullet_gravity_remaining = 0.4;
        p.wall_gravity_remaining = 0.7;
        p.jump_lifecycle.deferred_pending = true;
        p.jump_lifecycle.deferred_height = 7.;
        p.jumps_remaining = 1;
        p.bullet_jumps_left = 0;
        p.air_rolls_left = 0;
        // Both graph values are idle: pending request takes explicit reset.
        p.advance_lab_jump_lifecycle(&Input::default(), MAX_SUBSTEP, &floor(), &Tuning::default());
        assert_eq!(p.last_jump_reset(), ResetDecision::ExplicitDeferredReset);
        assert_eq!(p.velocity, Vec3::new(3., 0., -5.));
        assert_eq!(p.slide_velocity(), Vec3::new(2., 4., 0.));
        assert_eq!(p.roll_velocity(), Vec3::new(0., 3., 6.));
        assert_eq!(
            (p.bullet_gravity_remaining, p.wall_gravity_remaining),
            (0.4, 0.7)
        );
        assert_eq!(
            (p.jumps_remaining, p.bullet_jumps_left, p.air_rolls_left),
            (2, 1, 1)
        );
        assert!(!p.jump_lifecycle.deferred_pending);
        assert_eq!(p.jump_lifecycle.deferred_height, -1.);
        assert_eq!(p.jump_lifecycle.properties.staged, 0);
    }

    #[test]
    fn repair_land_style_clear_is_separate_from_conditional_posture_cleanup() {
        for crouched in [false, true] {
            let mut p = settled();
            p.crouched = crouched;
            p.slide_requested = true;
            p.jumps_remaining = 0;
            p.jump_lifecycle.staged_land_style = 3.;
            p.advance_lab_jump_lifecycle(
                &Input::default(),
                MAX_SUBSTEP,
                &floor(),
                &Tuning::default(),
            );
            assert_eq!(p.last_jump_reset(), ResetDecision::StationaryRepair);
            assert_eq!(p.jump_lifecycle.staged_land_style, 0.);
            assert_eq!(p.slide_requested, !crouched);
            assert!(!p.crouched, "floor provides standing clearance");
        }
    }
    #[test]
    fn jump_double_jump_exhaustion_and_landing_reset() {
        let t = Tuning::default();
        let w = floor();
        let mut p = settled();
        let jump = Input {
            jump: true,
            ..Default::default()
        };
        p.step(&jump, MAX_SUBSTEP, &w, &t);
        assert!(!p.grounded);
        p.step(&Input::default(), 0.1, &w, &t);
        p.step(&jump, MAX_SUBSTEP, &w, &t);
        finish_query(&mut p, &jump, &w, &t);
        assert_eq!(p.air_jumps_left, 0);
        let before = p.velocity.y;
        p.step(&jump, MAX_SUBSTEP, &w, &t);
        assert!(p.velocity.y < before);
        for _ in 0..360 {
            p.step(&Input::default(), MAX_SUBSTEP, &w, &t);
        }
        assert!(p.grounded);
        assert_eq!(p.air_jumps_left, 1);
        assert_eq!(p.bullet_jumps_left, 1);
    }
    #[test]
    fn bullet_jump_uses_pitch_and_one_edge_per_outer_frame() {
        let mut p = settled();
        p.step(
            &Input {
                jump: true,
                crouch: true,
                camera_forward: Vec3::new(0., 1., -1.),
                ..Default::default()
            },
            0.05,
            &floor(),
            &Tuning::default(),
        );
        assert_eq!(p.bullet_jumps_left, 0);
        assert_eq!(p.air_jumps_left, 1);
        assert!(p.velocity.y > 8.);
        assert!(p.velocity.z < -8.);
    }
    #[test]
    fn ground_bullet_then_double_consumes_the_shared_budget() {
        let t = Tuning::default();
        let w = floor();
        let mut p = settled();
        p.step(
            &Input {
                jump: true,
                crouch: true,
                camera_forward: Vec3::Y,
                ..Default::default()
            },
            MAX_SUBSTEP,
            &w,
            &t,
        );
        assert_eq!(p.jumps_remaining, 1);
        assert_eq!(p.bullet_jumps_left, 0);
        p.step(
            &Input {
                jump: true,
                ..Default::default()
            },
            MAX_SUBSTEP,
            &w,
            &t,
        );
        finish_query(&mut p, &Input::default(), &w, &t);
        assert_eq!(p.jumps_remaining, 0);
        assert_eq!(p.air_jumps_left, 0);
        assert!(p.velocity.y > 8.);
        let before = p.velocity.y;
        p.step(
            &Input {
                jump: true,
                ..Default::default()
            },
            MAX_SUBSTEP,
            &w,
            &t,
        );
        assert!(
            (p.velocity.y - (before - t.gravity * t.bullet_jump_gravity_scale * MAX_SUBSTEP)).abs()
                < 1e-4
        );
    }
    #[test]
    fn ordinary_jump_then_air_bullet_denies_a_third_launch() {
        let t = Tuning::default();
        let w = floor();
        let mut p = settled();
        p.step(
            &Input {
                jump: true,
                ..Default::default()
            },
            MAX_SUBSTEP,
            &w,
            &t,
        );
        p.step(
            &Input {
                jump: true,
                crouch: true,
                camera_forward: Vec3::Y,
                ..Default::default()
            },
            MAX_SUBSTEP,
            &w,
            &t,
        );
        assert_eq!(p.jumps_remaining, 0);
        assert_eq!(p.bullet_jumps_left, 0);
        let before = p.velocity.y;
        p.step(
            &Input {
                jump: true,
                ..Default::default()
            },
            MAX_SUBSTEP,
            &w,
            &t,
        );
        assert!(
            (p.velocity.y - (before - t.gravity * t.bullet_jump_gravity_scale * MAX_SUBSTEP)).abs()
                < 1e-4
        );
    }
    #[test]
    fn ordinary_jump_then_double_cannot_use_an_unused_bullet_budget() {
        let t = Tuning::default();
        let w = floor();
        let mut p = settled();
        let jump = Input {
            jump: true,
            ..Default::default()
        };
        p.step(&jump, MAX_SUBSTEP, &w, &t);
        p.step(&jump, MAX_SUBSTEP, &w, &t);
        finish_query(&mut p, &jump, &w, &t);
        assert_eq!(p.jumps_remaining, 0);
        assert_eq!(p.bullet_jumps_left, 1);
        p.step(
            &Input {
                jump: true,
                crouch: true,
                camera_forward: Vec3::X,
                ..Default::default()
            },
            MAX_SUBSTEP,
            &w,
            &t,
        );
        assert_eq!(p.bullet_jumps_left, 1);
        assert_eq!(horizontal(p.velocity), Vec3::ZERO);
    }
    #[test]
    fn walking_off_a_platform_preserves_two_airborne_launches() {
        let t = Tuning::default();
        let w = World {
            terrain: Default::default(),
            solids: vec![Solid::new(Vec3::new(-3., 5., -3.), Vec3::new(1., 6., 3.))],
        };
        let mut p = Player::new(Vec3::new(0., 6.01, 0.));
        p.step(&Input::default(), 0.1, &w, &t);
        assert!(p.grounded);
        assert_eq!(p.air_jumps_left, 1);
        p.step(
            &Input {
                movement: Vec3::X,
                ..Default::default()
            },
            0.5,
            &w,
            &t,
        );
        assert!(!p.grounded);
        assert_eq!(p.jumps_remaining, 2);
        assert_eq!(p.air_jumps_left, 2);
        let jump = Input {
            jump: true,
            ..Default::default()
        };
        p.step(&jump, MAX_SUBSTEP, &w, &t);
        finish_query(&mut p, &jump, &w, &t);
        assert_eq!(p.jumps_remaining, 1);
        p.step(&jump, MAX_SUBSTEP, &w, &t);
        finish_query(&mut p, &jump, &w, &t);
        assert_eq!(p.jumps_remaining, 0);
    }
    #[test]
    fn custom_jump_maximum_initializes_respawn_and_landing_and_wall_budget() {
        let t = Tuning {
            max_jumps: 3,
            ..Default::default()
        };
        let mut w = floor();
        w.solids
            .push(Solid::new(Vec3::new(2., 0., -10.), Vec3::new(3., 8., 10.)));
        let mut p = Player::new(Vec3::Y * 20.);
        p.step(&Input::default(), MAX_SUBSTEP, &w, &t);
        assert_eq!(p.jumps_remaining, 3);
        p.jumps_remaining = 0;
        p.position = Vec3::new(1.699, 3., 0.);
        p.step(
            &Input {
                jump: true,
                ..Default::default()
            },
            MAX_SUBSTEP,
            &w,
            &t,
        );
        finish_query(&mut p, &Input::default(), &w, &t);
        assert_eq!(p.state, MovementState::WallJump);
        assert_eq!(p.jumps_remaining, 2);
        p.respawn(Vec3::new(0., 0.01, 0.));
        p.step(&Input::default(), 0.1, &w, &t);
        assert!(p.grounded);
        assert_eq!(p.jumps_remaining, 3);
        assert_eq!(p.air_jumps_left, 2);
    }
    #[test]
    fn legacy_tuning_json_defaults_the_new_shared_jump_maximum() {
        let mut value = serde_json::to_value(Tuning::default()).unwrap();
        value.as_object_mut().unwrap().remove("max_jumps");
        let tuning: Tuning = serde_json::from_value(value).unwrap();
        assert_eq!(tuning.max_jumps, 2);
        assert!(tuning.is_valid());
        assert!(
            !Tuning {
                max_jumps: 0,
                ..Default::default()
            }
            .is_valid()
        );
    }
    #[test]
    fn stronger_vertical_speed_is_preserved_by_bullet_jump() {
        let mut p = Player::new(Vec3::Y * 20.);
        p.velocity.y = 25.;
        p.step(
            &Input {
                jump: true,
                crouch: true,
                camera_forward: Vec3::new(0., 1., -1.),
                ..Default::default()
            },
            MAX_SUBSTEP,
            &floor(),
            &Tuning::default(),
        );
        assert!(p.velocity.y > 24.);
    }
    #[test]
    fn ordinary_double_and_wall_launches_preserve_stronger_upward_velocity() {
        let t = Tuning::default();
        let mut w = floor();
        w.solids
            .push(Solid::new(Vec3::new(2., 0., -10.), Vec3::new(3., 8., 10.)));
        let jump = Input {
            jump: true,
            ..Default::default()
        };
        for mut p in [
            settled(),
            Player::new(Vec3::Y * 20.),
            Player::new(Vec3::new(1.699, 3., 0.)),
        ]
        .into_iter()
        {
            p.velocity.y = 25.;
            let was_grounded=p.grounded;
            p.step(&jump, MAX_SUBSTEP, &w, &t);
            if !was_grounded {finish_query(&mut p,&jump,&w,&t);}
            // Airborne query preparation precedes the launch preparation. Only
            // that earlier substep ages gravity; the admitted launch preserves it.
            let expected = if was_grounded {25.}else{25.-t.gravity*MAX_SUBSTEP};
            assert!((p.velocity.y - expected).abs() < 1e-4);
        }
    }
    #[test]
    fn explicit_launch_gravity_keeps_ground_and_double_targets_independent_of_fall_gravity() {
        // Isolate ordinary launch arithmetic; default staged wall service is tested in wall90_tests.
        let w = floor();
        let jump = Input {
            jump: true,
            ..Default::default()
        };
        let light = Tuning {
            gravity: 10.,
            launch_gravity: Some(9.81),
            jump_height: 5.,
            double_jump_height: 3.75,
            ..Default::default()
        };
        let heavy = Tuning {
            gravity: 40.,
            ..light
        };
        for initial in [settled(), Player::new(Vec3::Y * 20.)] {
            let mut a = initial.clone();
            let mut b = initial;
            a.step(&jump, MAX_SUBSTEP, &w, &ordinary_only(light));
            b.step(&jump, MAX_SUBSTEP, &w, &ordinary_only(heavy));
            assert!(!a.grounded && !b.grounded);
            // Launch preparation skips scalar gravity, so the first result is
            // independent of ongoing gravity. The following update is not.
            assert!((a.velocity.y - b.velocity.y).abs() < 1e-5);
            assert!(a.velocity.y > 0.);
            assert_eq!(a.jumps_remaining, b.jumps_remaining);
            a.step(&Input::default(), MAX_SUBSTEP, &w, &ordinary_only(light));
            b.step(&Input::default(), MAX_SUBSTEP, &w, &ordinary_only(heavy));
            assert!((a.velocity.y - b.velocity.y - 0.125).abs() < 1e-5);
        }
    }

    #[test]
    fn zero_height_launch_consumes_one_gravity_skip_even_without_scalar_increase() {
        // Isolate ordinary launch arithmetic; default staged wall service is tested in wall90_tests.
        let t = Tuning {
            gravity: 24.,
            jump_height: 0.,
            double_jump_height: 0.,
            ..Default::default()
        };
        for (mut p, incoming) in [(settled(), 25.), (Player::new(Vec3::Y * 20.), 0.)] {
            p.velocity.y = incoming;
            p.step(
                &Input {
                    jump: true,
                    ..Default::default()
                },
                MAX_SUBSTEP,
                &floor(),
                &ordinary_only(t),
            );
            assert_eq!(p.velocity.y, incoming);
            p.step(&Input::default(), MAX_SUBSTEP, &floor(), &ordinary_only(t));
            assert!((p.velocity.y - (incoming - 0.1)).abs() < 1e-5);
        }
    }

    #[test]
    fn bullet_gravity_skip_requires_an_actual_positive_scalar_write() {
        let t = Tuning {
            gravity: 24.,
            bullet_jump_speed: 12.,
            bullet_jump_gravity_scale: 1.,
            ..Default::default()
        };
        // Equal and already-stronger scalars do not set the direct-launch flag.
        // Downward replacements set the marker but do not promote it to a skip.
        for (incoming, direction, expected) in [
            (0., Vec3::Y, 12.),
            (-10., Vec3::Y, 12.),
            (12., Vec3::Y, 11.9),
            (15., Vec3::Y, 14.9),
            (0., Vec3::NEG_Y, -12.1),
            (-20., Vec3::NEG_Y, -20.1),
            (0., Vec3::X, -0.1),
        ] {
            let mut p = Player::new(Vec3::Y * 20.);
            p.velocity.y = incoming;
            p.step(
                &Input {
                    jump: true,
                    crouch: true,
                    camera_forward: direction,
                    ..Default::default()
                },
                MAX_SUBSTEP,
                &World::default(),
                &t,
            );
            assert!(
                (p.velocity.y - expected).abs() < 1e-5,
                "incoming={incoming}, direction={direction}"
            );
            assert!(
                (p.bullet_gravity_remaining - (t.bullet_jump_duration - MAX_SUBSTEP)).abs() < 1e-6
            );
            p.step(&Input::default(), MAX_SUBSTEP, &World::default(), &t);
            assert!((p.velocity.y - (expected - 0.1)).abs() < 1e-5);
        }
    }

    #[test]
    fn double_height_launch_clears_slide_owner_without_folding_it_into_base() {
        // Isolate ordinary launch arithmetic; default staged wall service is tested in wall90_tests.
        let t = Tuning::default();
        let w = World::default();
        let mut p = Player::new(Vec3::Y * 20.);
        p.velocity = Vec3::new(5., 25., 0.);
        let mut control = p.clone();
        p.slide_impulse.replace(Vec3::new(8., 3., 0.), 10.);
        let input = Input {
            jump: true,
            movement: Vec3::X,
            ..Default::default()
        };
        p.step(&input, MAX_SUBSTEP, &w, &ordinary_only(t));
        control.step(&input, MAX_SUBSTEP, &w, &ordinary_only(t));
        assert_eq!(p.slide_velocity(), Vec3::ZERO);
        assert_eq!(p.velocity, control.velocity);
        assert_eq!(p.position, control.position);
        assert_eq!(p.velocity.y, 25.);
    }
    #[test]
    fn legacy_launch_gravity_uses_ongoing_gravity_and_explicit_values_are_validated() {
        let mut value = serde_json::to_value(Tuning {
            gravity: 13.,
            ..Default::default()
        })
        .unwrap();
        value.as_object_mut().unwrap().remove("launch_gravity");
        let legacy: Tuning = serde_json::from_value(value).unwrap();
        assert_eq!(legacy.launch_gravity, None);
        assert!(legacy.is_valid());
        let explicit = Tuning {
            launch_gravity: Some(13.),
            ..legacy
        };
        let round_trip: Tuning =
            serde_json::from_str(&serde_json::to_string(&explicit).unwrap()).unwrap();
        assert_eq!(round_trip.launch_gravity, Some(13.));
        for initial in [settled(), Player::new(Vec3::Y * 20.)] {
            let mut a = initial.clone();
            let mut b = initial;
            let jump = Input {
                jump: true,
                ..Default::default()
            };
            a.step(&jump, MAX_SUBSTEP, &floor(), &legacy);
            b.step(&jump, MAX_SUBSTEP, &floor(), &explicit);
            assert_eq!(a.velocity, b.velocity);
            assert_eq!(a.position, b.position);
        }
        for invalid in [0., -1., f32::NAN, f32::INFINITY] {
            assert!(
                !Tuning {
                    launch_gravity: Some(invalid),
                    ..Default::default()
                }
                .is_valid()
            );
        }
    }
    #[test]
    fn double_jump_ends_visible_bullet_state_but_retains_fading_gravity() {
        // Isolate ordinary launch arithmetic; default staged wall service is tested in wall90_tests.
        let t = Tuning::default();
        let w = floor();
        let mut p = settled();
        p.step(
            &Input {
                jump: true,
                crouch: true,
                camera_forward: Vec3::Y,
                ..Default::default()
            },
            MAX_SUBSTEP,
            &w,
            &ordinary_only(t),
        );
        p.step(&Input::default(), 0.1, &w, &ordinary_only(t));
        let before = p.velocity.y;
        p.step(
            &Input {
                jump: true,
                ..Default::default()
            },
            MAX_SUBSTEP,
            &w,
            &ordinary_only(t),
        );
        assert_eq!(p.bullet_remaining, 0.);
        assert_ne!(p.state, MovementState::BulletJump);
        assert!(p.bullet_gravity_remaining > 0.);
        let factor = crate::gravity::modifier_factor(
            t.bullet_jump_gravity_scale,
            p.bullet_gravity_remaining,
            t.gravity_modifier_fade_window,
        );
        assert!(factor > t.bullet_jump_gravity_scale && factor < 1.);
        assert_eq!(p.velocity.y, before);
        p.step(&Input::default(), MAX_SUBSTEP, &w, &ordinary_only(t));
        assert!(p.velocity.y < before);
    }
    #[test]
    fn landing_preserves_gravity_until_expiry_and_next_bullet_refreshes_it() {
        let t = Tuning::default();
        let w = floor();
        let mut p = Player::new(Vec3::new(0., 0.2, 0.));
        let bullet = Input {
            jump: true,
            crouch: true,
            camera_forward: Vec3::NEG_Y,
            ..Default::default()
        };
        p.step(&bullet, MAX_SUBSTEP, &w, &t);
        assert!(!p.grounded);
        for _ in 0..20 {
            if p.grounded {
                break;
            }
            p.step(&Input::default(), MAX_SUBSTEP, &w, &t);
        }
        assert!(p.grounded);
        assert_eq!(p.bullet_remaining, 0.);
        assert!(p.bullet_gravity_remaining > t.bullet_jump_duration - 0.1);
        let before_reset = p.bullet_gravity_remaining;
        p.step(&Input::default(), MAX_SUBSTEP, &w, &t);
        assert_eq!(p.last_jump_reset(), ResetDecision::Landing);
        assert_eq!(p.bullet_jumps_left, 1);
        assert_eq!(p.bullet_gravity_remaining, before_reset - MAX_SUBSTEP);
        p.step(
            &Input {
                jump: true,
                ..Default::default()
            },
            MAX_SUBSTEP,
            &w,
            &t,
        );
        let desired = (2. * t.gravity * t.jump_height).sqrt();
        assert!((p.velocity.y - desired).abs() < 1e-4);
        p.step(&Input::default(), 0.2, &w, &t);
        assert!(p.bullet_gravity_remaining < t.bullet_jump_duration - 0.19);
        p.step(
            &Input {
                camera_forward: Vec3::Y,
                ..bullet
            },
            MAX_SUBSTEP,
            &w,
            &t,
        );
        assert_eq!(
            p.bullet_gravity_remaining,
            t.bullet_jump_duration - MAX_SUBSTEP
        );
        p.step(
            &Input::default(),
            t.bullet_jump_duration + MAX_SUBSTEP,
            &w,
            &t,
        );
        assert_eq!(p.bullet_gravity_remaining, 0.);
    }
    #[test]
    fn double_jump_gain_is_independent_of_bullet_speed_and_scales_with_physics_base() {
        // Isolate ordinary launch arithmetic; default staged wall service is tested in wall90_tests.
        let w = floor();
        let t = Tuning {
            physics_base_speed: 8.,
            air_acceleration: 0.,
            turn_rate: 0.,
            ..Default::default()
        };
        let launch = |tuning: &Tuning| {
            let mut p = Player::new(Vec3::Y * 20.);
            p.velocity = Vec3::X * 10.;
            p.step(
                &Input {
                    movement: Vec3::X,
                    jump: true,
                    ..Default::default()
                },
                MAX_SUBSTEP,
                &w,
                &ordinary_only(*tuning),
            );
            p.velocity
        };
        let slow_bullet = launch(&Tuning {
            bullet_jump_speed: 1.,
            ..t
        });
        let fast_bullet = launch(&Tuning {
            bullet_jump_speed: 50.,
            ..t
        });
        assert_eq!(slow_bullet, fast_bullet);
        assert!((slow_bullet.x - 12.8).abs() < 1e-5);
        let doubled_base = launch(&Tuning {
            physics_base_speed: 16.,
            ..t
        });
        assert!((doubled_base.x - 10. - 2. * (slow_bullet.x - 10.)).abs() < 1e-5);
        assert_eq!(doubled_base.y, slow_bullet.y);
        let no_base = launch(&Tuning {
            physics_base_speed: 0.,
            ..t
        });
        assert_eq!(no_base.x, 10.);
    }
    #[test]
    fn physics_base_speed_has_explicit_legacy_json_default_and_round_trips() {
        let mut value = serde_json::to_value(Tuning {
            bullet_jump_speed: 31.,
            ..Default::default()
        })
        .unwrap();
        value.as_object_mut().unwrap().remove("physics_base_speed");
        let legacy: Tuning = serde_json::from_value(value).unwrap();
        assert_eq!(legacy.physics_base_speed, 18.);
        assert_eq!(legacy.bullet_jump_speed, 31.);
        assert!(legacy.is_valid());

        // Explicit migration can preserve a customized old file's double gain.
        let migrated = Tuning {
            physics_base_speed: legacy.bullet_jump_speed,
            ..legacy
        };
        let reloaded: Tuning =
            serde_json::from_str(&serde_json::to_string(&migrated).unwrap()).unwrap();
        assert_eq!(reloaded.physics_base_speed, 31.);
        assert!(reloaded.is_valid());
        for invalid in [-1., f32::NAN, f32::INFINITY] {
            assert!(
                !Tuning {
                    physics_base_speed: invalid,
                    ..Default::default()
                }
                .is_valid()
            );
        }
    }
    #[test]
    fn air_target_cap_damps_excess_speed_without_snapping_to_the_cap() {
        let t = Tuning {
            air_acceleration: 10.,
            air_accel_drag: Some(0.05),
            ..Default::default()
        };
        let mut p = Player::new(Vec3::Y * 20.);
        p.velocity = Vec3::X * 20.;
        p.step(
            &Input {
                movement: Vec3::X,
                ..Default::default()
            },
            MAX_SUBSTEP,
            &floor(),
            &t,
        );
        // Raising the cap to the old speed used to leave this exactly at 20.
        assert!(p.velocity.x < 20.);
        assert!(p.velocity.x > t.walk_speed);
        assert_eq!(p.velocity.z, 0.);
    }
    #[test]
    fn ground_and_air_alignment_drags_affect_only_their_selected_branch() {
        let w = floor();
        let t = Tuning {
            ground_acceleration: 12.,
            air_acceleration: 10.,
            ground_accel_drag: Some(8.5),
            air_accel_drag: Some(0.05),
            ..Default::default()
        };
        let step = |grounded: bool, tuning: &Tuning| {
            let mut p = if grounded {
                settled()
            } else {
                Player::new(Vec3::Y * 20.)
            };
            p.velocity = Vec3::X * 4.;
            p.step(
                &Input {
                    movement: Vec3::Z,
                    ..Default::default()
                },
                MAX_SUBSTEP,
                &w,
                tuning,
            );
            horizontal(p.velocity)
        };
        let ground_changed = Tuning {
            ground_accel_drag: Some(20.),
            ..t
        };
        let air_changed = Tuning {
            air_accel_drag: Some(2.),
            ..t
        };
        assert_eq!(step(false, &t), step(false, &ground_changed));
        assert_eq!(step(true, &t), step(true, &air_changed));
        assert!(step(true, &ground_changed).x < step(true, &t).x);
        assert!(step(false, &air_changed).x < step(false, &t).x);
    }
    #[test]
    fn air_decel_drag_controls_no_input_braking_independently_of_steering() {
        let t = Tuning {
            air_accel_drag: Some(0.05),
            air_decel_drag: 0.,
            ..Default::default()
        };
        let mut baseline = Player::new(Vec3::Y * 20.);
        baseline.velocity = Vec3::X * 30.;
        let mut stronger = baseline.clone();
        baseline.step(&Input::default(), MAX_SUBSTEP, &floor(), &t);
        stronger.step(
            &Input::default(),
            MAX_SUBSTEP,
            &floor(),
            &Tuning {
                air_decel_drag: 0.2,
                ..t
            },
        );
        assert!(stronger.velocity.x < baseline.velocity.x);
        assert!(baseline.velocity.x < 30.); // Minimum braking remains at drag0.
        assert!(stronger.velocity.x > 0.);
        assert_eq!(stronger.velocity.y, baseline.velocity.y);
        assert_eq!(stronger.velocity.z, 0.);
    }
    #[test]
    fn ground_air_drag_fields_load_legacy_json_and_validate_independently() {
        let mut value = serde_json::to_value(Tuning::default()).unwrap();
        for name in ["ground_accel_drag", "air_accel_drag", "air_decel_drag"] {
            value.as_object_mut().unwrap().remove(name);
        }
        let legacy: Tuning = serde_json::from_value(value).unwrap();
        assert_eq!(legacy.ground_accel_drag, None);
        assert_eq!(legacy.air_accel_drag, None);
        assert_eq!(legacy.air_decel_drag, 0.);
        assert!(legacy.is_valid());
        let tuned = Tuning {
            ground_accel_drag: Some(8.5),
            air_accel_drag: Some(0.05),
            air_decel_drag: 0.2,
            ..legacy
        };
        let round_trip: Tuning =
            serde_json::from_str(&serde_json::to_string(&tuned).unwrap()).unwrap();
        assert_eq!(round_trip.ground_accel_drag, Some(8.5));
        assert_eq!(round_trip.air_accel_drag, Some(0.05));
        assert_eq!(round_trip.air_decel_drag, 0.2);
        for invalid in [-1., f32::NAN, f32::INFINITY] {
            for bad in [
                Tuning {
                    ground_accel_drag: Some(invalid),
                    ..tuned
                },
                Tuning {
                    air_accel_drag: Some(invalid),
                    ..tuned
                },
                Tuning {
                    air_decel_drag: invalid,
                    ..tuned
                },
            ] {
                assert!(!bad.is_valid());
            }
        }
    }
    #[test]
    fn double_jump_gain_preserves_analog_input_magnitude() {
        // Isolate ordinary launch arithmetic; default staged wall service is tested in wall90_tests.
        let t = Tuning {
            air_acceleration: 0.,
            turn_rate: 0.,
            ..Tuning::default()
        };
        let w = floor();
        let mut full = Player::new(Vec3::Y * 20.);
        full.velocity = Vec3::X * 10.;
        let mut half = full.clone();
        full.step(
            &Input {
                movement: Vec3::X,
                jump: true,
                ..Default::default()
            },
            MAX_SUBSTEP,
            &w,
            &ordinary_only(t),
        );
        half.step(
            &Input {
                movement: Vec3::X * 0.5,
                jump: true,
                ..Default::default()
            },
            MAX_SUBSTEP,
            &w,
            &ordinary_only(t),
        );
        let half_added = half.velocity.x - 10.;
        assert!((full.velocity.x - 10. - 2. * half_added).abs() < 1e-5);
        assert!(half_added > 0.);
    }
    #[test]
    fn crouch_release_under_ceiling_preserves_clearance() {
        let mut w = floor();
        w.solids
            .push(Solid::new(Vec3::new(-2., 1.1, -2.), Vec3::new(2., 2., 2.)));
        let mut p = Player::new(Vec3::new(0., 0.001, 0.));
        p.crouched = true;
        p.step(&Input::default(), 0.1, &w, &Tuning::default());
        assert!(p.crouched);
        assert!(w.can_occupy(p.position, 0.3, p.height(&Tuning::default())));
    }
    #[test]
    fn slide_boost_has_cooldown_and_preserves_momentum() {
        let t = Tuning::default();
        let w = floor();
        let mut p = settled();
        p.velocity = -Vec3::Z * 9.;
        let slide = Input {
            movement: -Vec3::Z,
            crouch: true,
            sprint: true,
            ..Default::default()
        };
        p.step(&slide, MAX_SUBSTEP, &w, &t);
        assert_eq!(p.state, MovementState::Slide);
        assert!(p.combined_velocity().length() > 16.);
        assert!(horizontal(p.velocity).length() < 9.);
        assert_eq!(p.slide_velocity(), -Vec3::Z * t.slide_boost);
        p.step(
            &Input {
                crouch: false,
                movement: -Vec3::Z,
                sprint: true,
                ..Default::default()
            },
            MAX_SUBSTEP,
            &w,
            &t,
        );
        let before = p.combined_velocity().length();
        p.step(&slide, MAX_SUBSTEP, &w, &t);
        assert!(p.combined_velocity().length() <= before + 0.01);
    }

    #[test]
    fn flat_slide_brakes_base_speed_proportionally_without_braking_the_impulse() {
        let t = Tuning {
            slide_drag: 0.7,
            ground_accel_drag: Some(2.),
            slide_end_speed: 0.,
            walk_speed: 6.,
            ..Default::default()
        };
        let mut p = settled();
        p.velocity = Vec3::X * 10.;
        p.slide_requested = true;
        p.previously_crouch = true;
        p.slide_impulse.replace(Vec3::X * 5., 10.);
        // Perpendicular input must not steer the flat slope target.
        p.step(
            &Input {
                crouch: true,
                movement: Vec3::Z,
                ..Default::default()
            },
            MAX_SUBSTEP,
            &floor(),
            &t,
        );
        // At 240 Hz: 10 -> 9.9708333 by braking, then excess-speed damping.
        assert!((p.velocity.x - 9.9375972).abs() < 2e-5);
        assert_eq!(p.velocity.z, 0.);
        assert_eq!(p.slide_velocity(), Vec3::X * 5.);
        assert_eq!(p.state, MovementState::Slide);
    }

    #[test]
    fn flat_slide_keeps_full_selected_cap_without_input_and_minimum_braking() {
        let t = Tuning {
            slide_drag: 0.7,
            ground_accel_drag: Some(8.5),
            slide_end_speed: 0.,
            walk_speed: 6.,
            sprint_speed: 7.5,
            ..Default::default()
        };
        let input = Input {
            crouch: true,
            ..Default::default()
        };
        let make = |speed| {
            let mut p = settled();
            p.velocity = Vec3::X * speed;
            p.slide_requested = true;
            p.previously_crouch = true;
            p
        };
        let mut jog = make(7.);
        let mut sprint = jog.clone();
        jog.step(&input, MAX_SUBSTEP, &floor(), &t);
        sprint.step(
            &Input {
                sprint: true,
                ..input
            },
            MAX_SUBSTEP,
            &floor(),
            &t,
        );
        assert!((sprint.velocity.x - 6.9795833).abs() < 2e-5);
        assert!((jog.velocity.x - 6.8807059).abs() < 2e-5);
        // Zero SlideDrag still retains the native minimum braking of 3.
        let mut slow = make(2.);
        slow.step(
            &input,
            MAX_SUBSTEP,
            &floor(),
            &Tuning {
                slide_drag: 0.,
                ..t
            },
        );
        assert!((slow.velocity.x - 1.9875).abs() < 2e-5);
    }

    #[test]
    fn zero_slide_alignment_drag_uses_the_direct_input_branch() {
        let t = Tuning {
            ground_accel_drag: Some(0.),
            slide_end_speed: 0.,
            walk_speed: 6.,
            ..Default::default()
        };
        let mut p = settled();
        p.velocity = Vec3::X * 10.;
        p.slide_requested = true;
        p.previously_crouch = true;
        p.step(
            &Input {
                crouch: true,
                movement: Vec3::Z * 0.5,
                ..Default::default()
            },
            MAX_SUBSTEP,
            &floor(),
            &t,
        );
        assert_eq!(horizontal(p.velocity), Vec3::Z * 3.);
        p.step(
            &Input {
                crouch: true,
                ..Default::default()
            },
            MAX_SUBSTEP,
            &floor(),
            &t,
        );
        assert_eq!(horizontal(p.velocity), Vec3::ZERO);
    }
    #[test]
    fn aim_glide_consumes_resource_and_recharges_after_release() {
        let t = Tuning::default();
        let w = floor();
        let mut p = Player::new(Vec3::Y * 20.);
        p.step(
            &Input {
                aim: true,
                ..Default::default()
            },
            0.5,
            &w,
            &t,
        );
        assert_eq!(p.state, MovementState::AimGlide);
        assert!(p.aim_resource < 0.9);
        assert!(p.velocity.y > -2.);
        let charge = p.aim_resource;
        p.step(&Input::default(), 0.5, &w, &t);
        assert!(p.aim_resource > charge);
    }

    #[test]
    fn aim_entry_scales_once_skips_first_drain_and_keeps_slide_vector() {
        let t = Tuning {
            gravity: 12.,
            ..Default::default()
        };
        let mut p = Player::new(Vec3::Y * 20.);
        p.velocity.y = 8.;
        p.slide_impulse.replace(Vec3::Y * 3., 0.);
        let input = Input {
            aim: true,
            ..Default::default()
        };
        p.step(&input, MAX_SUBSTEP, &floor(), &t);
        assert_eq!(p.state, MovementState::AimGlide);
        assert_eq!(p.aim_resource, 1.);
        assert!((p.velocity.y - 1.95).abs() < 1e-5);
        assert_eq!(p.slide_velocity(), Vec3::Y * 3.);
        p.step(&input, MAX_SUBSTEP, &floor(), &t);
        assert!((p.velocity.y - 1.9).abs() < 1e-5);
        assert!((p.aim_resource - (1. - MAX_SUBSTEP / t.aim_duration)).abs() < 1e-6);
    }

    #[test]
    fn aim_phase_uses_post_entry_vertical_and_partial_resource() {
        let t = Tuning {
            gravity: 12.,
            ..Default::default()
        };
        let input = Input {
            aim: true,
            ..Default::default()
        };
        for (incoming, resource, expected) in
            [(2., 1., 0.473125), (2., 0.5, 1.2), (-8., 1., -2.00375)]
        {
            let mut p = Player::new(Vec3::Y * 20.);
            p.velocity.y = incoming;
            p.aim_resource = resource;
            p.step(&input, MAX_SUBSTEP, &floor(), &t);
            assert!((p.velocity.y - expected).abs() < 1e-5);
            assert_eq!(p.aim_resource, resource);
        }
    }

    #[test]
    fn aim_exhaustion_removes_gravity_and_exit_does_not_recharge_in_same_update() {
        let t = Tuning {
            gravity: 12.,
            ..Default::default()
        };
        let mut p = Player::new(Vec3::Y * 20.);
        p.velocity.y = -2.;
        p.aim_was_active = true;
        p.state = MovementState::AimGlide;
        p.aim_resource = 0.001;
        p.step(
            &Input {
                aim: true,
                ..Default::default()
            },
            MAX_SUBSTEP,
            &floor(),
            &t,
        );
        assert_eq!(p.aim_resource, 0.);
        assert_eq!(p.state, MovementState::Air);
        assert!((p.velocity.y + 2.05).abs() < 1e-5);
        p.aim_was_active = true;
        p.state = MovementState::AimGlide;
        p.aim_resource = 0.5;
        p.step(&Input::default(), MAX_SUBSTEP, &floor(), &t);
        let after_exit = 0.5 - MAX_SUBSTEP / t.aim_duration - t.aim_exit_penalty;
        assert!((p.aim_resource - after_exit).abs() < 1e-6);
        p.step(&Input::default(), MAX_SUBSTEP, &floor(), &t);
        assert!((p.aim_resource - (after_exit + MAX_SUBSTEP * t.aim_recharge)).abs() < 1e-6);
    }

    #[test]
    fn legacy_aim_entry_parameter_defaults_and_rejects_invalid_values() {
        let mut value = serde_json::to_value(Tuning::default()).unwrap();
        value
            .as_object_mut()
            .unwrap()
            .remove("aim_entry_vertical_scale");
        let restored: Tuning = serde_json::from_value(value).unwrap();
        assert_eq!(restored.aim_entry_vertical_scale, 0.25);
        for invalid in [-1., f32::NAN, f32::INFINITY] {
            assert!(
                !Tuning {
                    aim_entry_vertical_scale: invalid,
                    ..restored
                }
                .is_valid()
            );
        }
    }

    #[test]
    fn aim_and_wall_modes_share_activity_without_repeating_entry_grace() {
        let t = Tuning {
            gravity: 12.,
            ..Default::default()
        };
        let mut w = floor();
        w.solids
            .push(Solid::new(Vec3::new(2., 0., -10.), Vec3::new(3., 30., 10.)));
        let mut p = Player::new(Vec3::Y * 20.);
        p.velocity.y = 8.;
        let input = Input {
            aim: true,
            ..Default::default()
        };
        p.step(&input, MAX_SUBSTEP, &w, &t);
        assert_eq!(p.state, MovementState::AimGlide);
        assert_eq!(p.aim_resource, 1.);
        p.position.x = 1.699;
        p.velocity.x = 4.; // Incoming planar vector admits new wall preparation.
        p.step(&input, MAX_SUBSTEP, &w, &t);
        assert_eq!(p.state, MovementState::WallLatch);
        let after_wall = 1. - MAX_SUBSTEP * 0.5 / t.wall_latch_duration;
        assert!((p.aim_resource - after_wall).abs() < 1e-6);
        p.position.x = 0.;
        p.velocity.y = 4.;
        p.step(&input, MAX_SUBSTEP, &w, &t);
        assert_eq!(p.state, MovementState::AimGlide);
        assert!((p.velocity.y - 3.95).abs() < 1e-5);
        assert!((p.aim_resource - (after_wall - MAX_SUBSTEP / t.aim_duration)).abs() < 1e-6);
    }
    #[test]
    fn wall_latch_and_jump_require_side_contact() {
        let t = Tuning::default();
        let mut w = floor();
        w.solids
            .push(Solid::new(Vec3::new(2., 0., -10.), Vec3::new(3., 8., 10.)));
        let mut p = Player::new(Vec3::new(1.699, 3., 0.));
        p.velocity.x = 4.;
        p.step(
            &Input {
                aim: true,
                ..Default::default()
            },
            MAX_SUBSTEP,
            &w,
            &t,
        );
        assert_eq!(p.state, MovementState::WallLatch);
        p.step(
            &Input {
                jump: true,
                aim: true,
                movement: Vec3::X,
                ..Default::default()
            },
            MAX_SUBSTEP,
            &w,
            &t,
        );
        finish_query(&mut p,&Input{aim:true,movement:Vec3::X,..Default::default()},&w,&t);
        assert!(p.velocity.x < 0.);
        assert!(p.velocity.y > 5.);
        assert_eq!(p.state, MovementState::WallJump);
        let mut q = settled();
        q.step(
            &Input {
                aim: true,
                ..Default::default()
            },
            MAX_SUBSTEP,
            &w,
            &t,
        );
        assert_ne!(q.state, MovementState::WallLatch);
    }
    #[test]
    fn equivalent_frame_partitions_produce_close_results() {
        let t = Tuning::default();
        let w = floor();
        let mut a = settled();
        let mut b = a.clone();
        let input = Input {
            movement: -Vec3::Z,
            sprint: true,
            ..Default::default()
        };
        for _ in 0..60 {
            a.step(&input, 1. / 60., &w, &t);
        }
        for _ in 0..120 {
            b.step(&input, 1. / 120., &w, &t);
        }
        assert!(a.position.distance(b.position) < 0.001);
        assert!(a.velocity.distance(b.velocity) < 0.001);
    }
    #[test]
    fn respawn_clears_movement_and_air_action_state() {
        let mut p = settled();
        p.velocity = Vec3::ONE * 20.;
        p.air_jumps_left = 0;
        p.aim_resource = 0.;
        p.respawn(Vec3::new(3., 5., 7.));
        assert_eq!(p.position, Vec3::new(3., 5., 7.));
        assert_eq!(p.velocity, Vec3::ZERO);
        assert_eq!(p.aim_resource, 1.);
        assert_eq!(p.air_jumps_left, default_max_jumps());
    }

    #[test]
    fn air_roll_expires_consumes_budget_and_resets_on_landing() {
        let t = Tuning::default();
        let w = floor();
        let mut p = Player::new(Vec3::Y * 20.);
        let roll = Input {
            movement: Vec3::X,
            roll: true,
            ..Default::default()
        };
        p.step(&roll, MAX_SUBSTEP, &w, &t);
        assert_eq!(p.state, MovementState::Roll);
        assert_eq!(p.air_rolls_left, 0);
        assert_eq!(p.roll_velocity(), Vec3::X * t.roll_speed);
        assert!(p.velocity.x < t.roll_speed);
        p.step(&Input::default(), t.roll_duration * 0.5, &w, &t);
        assert_eq!(p.state, MovementState::Roll);
        p.step(&Input::default(), t.roll_duration, &w, &t);
        assert_ne!(p.state, MovementState::Roll);
        p.step(&roll, MAX_SUBSTEP, &w, &t);
        assert_ne!(p.state, MovementState::Roll);
        assert_eq!(p.air_rolls_left, 0);
        for _ in 0..360 {
            p.step(&Input::default(), MAX_SUBSTEP, &w, &t);
        }
        assert!(p.grounded);
        assert_eq!(p.air_rolls_left, 1);
    }
    #[test]
    fn accepted_bullet_removes_slide_owner_before_launch_composition() {
        let t = Tuning::default();
        let w = World::default();
        let mut p = Player::new(Vec3::Y * 20.);
        p.velocity = Vec3::X * 5.;
        p.previously_crouch = true;
        let mut control = p.clone();
        p.slide_impulse.replace(Vec3::X * 8., 10.);
        let input = Input {
            jump: true,
            crouch: true,
            camera_forward: -Vec3::Z,
            ..Default::default()
        };
        p.step(&input, MAX_SUBSTEP, &w, &t);
        control.step(&input, MAX_SUBSTEP, &w, &t);
        assert_eq!(p.velocity, control.velocity);
        assert_eq!(p.slide_velocity(), Vec3::ZERO);
        assert_eq!(p.position, control.position);
        p.step(&Input::default(), MAX_SUBSTEP, &w, &t);
        assert_eq!(p.slide_velocity(), Vec3::ZERO);
    }

    #[test]
    fn bullet_clears_same_step_slide_insertion_but_rejected_bullet_keeps_record() {
        let t = Tuning::default();
        let w = floor();
        let mut p = settled();
        p.velocity = Vec3::X * 8.;
        p.step(
            &Input {
                movement: Vec3::X,
                camera_forward: Vec3::X,
                jump: true,
                crouch: true,
                sprint: true,
                ..Default::default()
            },
            MAX_SUBSTEP,
            &w,
            &t,
        );
        assert_eq!(p.state, MovementState::BulletJump);
        assert_eq!(p.slide_velocity(), Vec3::ZERO);

        let mut rejected = Player::new(Vec3::Y * 20.);
        rejected.jump_budget_initialized = true;
        rejected.jumps_remaining = 0;
        rejected.previously_crouch = true;
        rejected.slide_impulse.replace(Vec3::X * 8., 10.);
        rejected.step(
            &Input {
                jump: true,
                crouch: true,
                ..Default::default()
            },
            MAX_SUBSTEP,
            &World::default(),
            &t,
        );
        assert_eq!(rejected.bullet_jumps_left, 1);
        assert_eq!(rejected.slide_velocity(), Vec3::X * 8.);
    }

    #[test]
    fn new_bullet_gravity_ages_before_first_integration_and_can_expire_immediately() {
        // Independent first-step expectations at 240 Hz, gravity 24, zero
        // multiplier and fade window .5: .25s lifetime yields factor61/120;
        // lifetimes <= one step expire and yield normal gravity.
        for (duration, expected_drop) in [(0.25, 61. / 1200.), (MAX_SUBSTEP, 0.1), (0., 0.1)] {
            let t = Tuning {
                gravity: 24.,
                bullet_jump_duration: duration,
                bullet_jump_gravity_scale: 0.,
                gravity_modifier_fade_window: 0.5,
                ..Default::default()
            };
            let mut p = Player::new(Vec3::Y * 20.);
            p.step(
                &Input {
                    jump: true,
                    crouch: true,
                    camera_forward: Vec3::X,
                    ..Default::default()
                },
                MAX_SUBSTEP,
                &World::default(),
                &t,
            );
            assert!((p.velocity.y + expected_drop).abs() < 1e-6);
        }
    }

    #[test]
    fn slide_record_persists_through_landing_release_and_jump_but_respawn_clears_it() {
        let t = Tuning::default();
        let mut p = Player::new(Vec3::Y * 0.001);
        p.velocity.y = -3.;
        p.slide_impulse.replace(Vec3::X * 8., 10.);
        p.step(&Input::default(), MAX_SUBSTEP, &floor(), &t);
        assert!(p.grounded);
        assert_eq!(p.slide_velocity(), Vec3::X * 8.);
        assert_eq!(p.velocity.x, 0.);
        p.step(
            &Input {
                jump: true,
                ..Default::default()
            },
            MAX_SUBSTEP,
            &floor(),
            &t,
        );
        assert!(!p.grounded);
        assert!(p.slide_velocity().x > 7.9);
        assert_eq!(p.velocity.x, 0.);
        p.respawn(Vec3::Y);
        assert_eq!(p.slide_velocity(), Vec3::ZERO);
        assert_eq!(p.combined_velocity(), Vec3::ZERO);
    }

    #[test]
    fn blocked_composed_velocity_never_folds_impulse_into_base() {
        let w = World {
            terrain: Default::default(),
            solids: vec![Solid::new(Vec3::new(1., 0., -5.), Vec3::new(2., 10., 5.))],
        };
        let t = Tuning::default();
        let mut p = Player::new(Vec3::new(0.69, 2., 0.));
        p.velocity.x = 5.;
        p.slide_impulse.replace(Vec3::X * 8., 10.);
        let mut free = p.clone();
        free.step(&Input::default(), MAX_SUBSTEP, &World::default(), &t);
        p.step(&Input::default(), MAX_SUBSTEP, &w, &t);
        assert!(p.position.x <= 0.7);
        assert_eq!(p.velocity, free.velocity);
        assert_eq!(p.slide_velocity().x, 8.);
        assert_eq!(p.combined_velocity(), free.combined_velocity());
        free.step(&Input::default(), MAX_SUBSTEP, &World::default(), &t);
        p.step(&Input::default(), MAX_SUBSTEP, &w, &t);
        assert_eq!(p.velocity, free.velocity);
        assert!(p.slide_velocity().x > 7.9 && p.slide_velocity().x < 8.);
        // Removing the wall releases prepared motion with both channels intact.
        let before = p.position.x;
        free.step(&Input::default(), MAX_SUBSTEP, &World::default(), &t);
        p.step(&Input::default(), MAX_SUBSTEP, &World::default(), &t);
        assert_eq!(p.velocity, free.velocity);
        assert!((p.position.x - before - p.combined_velocity().x * MAX_SUBSTEP).abs() < 1e-6);
    }

    #[test]
    fn wall_latch_suppresses_slide_motion_while_record_decays() {
        let w = World {
            terrain: Default::default(),
            solids: vec![Solid::new(Vec3::new(1., 0., -5.), Vec3::new(2., 10., 5.))],
        };
        let mut p = Player::new(Vec3::new(0.69, 2., 0.));
        p.velocity.x = 4.;
        p.slide_impulse.replace(Vec3::Z * 8., 10.);
        let input = Input {
            aim: true,
            ..Default::default()
        };
        let before = p.position;
        p.step(&input, MAX_SUBSTEP, &w, &Tuning::default());
        assert_eq!(p.state, MovementState::WallLatch);
        assert_eq!(p.position, before);
        assert_eq!(p.combined_velocity(), Vec3::ZERO);
        p.step(&input, MAX_SUBSTEP, &w, &Tuning::default());
        assert_eq!(p.position, before);
        assert!((p.slide_velocity().z - (8. - 10. * MAX_SUBSTEP)).abs() < 1e-6);
    }

    #[test]
    fn slide_retrigger_replaces_record_after_cooldown_and_captures_decay() {
        let t = Tuning {
            slide_boost_cooldown: 0.,
            ..Default::default()
        };
        let w = floor();
        let mut p = settled();
        p.velocity = -Vec3::Z * 9.;
        let input = Input {
            movement: -Vec3::Z,
            crouch: true,
            ..Default::default()
        };
        p.step(&input, MAX_SUBSTEP, &w, &t);
        p.step(&Input::default(), MAX_SUBSTEP, &w, &t);
        let changed = Tuning {
            slide_boost: 3.,
            slide_impulse_decay: 2.,
            ..t
        };
        p.step(
            &Input {
                movement: Vec3::X,
                ..input
            },
            MAX_SUBSTEP,
            &w,
            &changed,
        );
        assert_eq!(p.slide_velocity(), Vec3::X * 3.);
        // Changing tuning after insertion does not rewrite this record's decay.
        p.step(&Input::default(), MAX_SUBSTEP, &w, &t);
        assert!((p.slide_velocity().x - (3. - 2. * MAX_SUBSTEP)).abs() < 1e-6);
    }

    #[test]
    fn legacy_slide_json_defaults_and_new_record_round_trip() {
        let mut value = serde_json::to_value(Tuning::default()).unwrap();
        value.as_object_mut().unwrap().remove("slide_impulse_decay");
        let t: Tuning = serde_json::from_value(value).unwrap();
        assert_eq!(t.slide_impulse_decay, 10.);
        assert!(
            Tuning {
                slide_impulse_decay: 0.,
                ..t
            }
            .is_valid()
        );
        for invalid in [-1., f32::NAN, f32::INFINITY] {
            assert!(
                !Tuning {
                    slide_impulse_decay: invalid,
                    ..t
                }
                .is_valid()
            );
        }
        let mut p = Player::new(Vec3::Y);
        p.slide_impulse.replace(Vec3::X * 3., 2.);
        p.slide_impulse.evaluate(MAX_SUBSTEP);
        let mut value = serde_json::to_value(&p).unwrap();
        let mut restored: Player = serde_json::from_value(value.clone()).unwrap();
        restored.slide_impulse.evaluate(0.5);
        assert_eq!(restored.slide_velocity(), Vec3::X * 2.);
        value.as_object_mut().unwrap().remove("slide_impulse");
        let legacy: Player = serde_json::from_value(value).unwrap();
        assert_eq!(legacy.slide_velocity(), Vec3::ZERO);
    }
    #[test]
    fn no_input_retrigger_uses_surviving_composed_slide_direction() {
        let t = Tuning {
            slide_boost_cooldown: 0.,
            ..Default::default()
        };
        let mut p = settled();
        p.velocity = Vec3::ZERO;
        p.slide_impulse.replace(Vec3::X * 5., 10.);
        // Base has stopped, but the surviving additive speed admits a slide.
        p.step(
            &Input {
                crouch: true,
                ..Default::default()
            },
            MAX_SUBSTEP,
            &floor(),
            &t,
        );
        assert_eq!(p.slide_velocity(), Vec3::X * t.slide_boost);
        assert_eq!(p.velocity.x, 0.);
    }
    #[test]
    fn roll_adds_orthogonal_or_opposed_velocity_without_redirecting_base() {
        let t = Tuning::default();
        let w = World::default();
        for direction in [Vec3::Z, -Vec3::X] {
            let mut p = Player::new(Vec3::Y * 20.);
            p.velocity = Vec3::X * 8.;
            let mut control = p.clone();
            let input = Input {
                movement: direction,
                roll: true,
                ..Default::default()
            };
            p.step(&input, MAX_SUBSTEP, &w, &t);
            control.step(
                &Input {
                    roll: false,
                    ..input
                },
                MAX_SUBSTEP,
                &w,
                &t,
            );
            assert_eq!(p.velocity, control.velocity);
            assert_eq!(p.roll_velocity(), direction * t.roll_speed);
            assert!(
                (p.position - control.position - direction * t.roll_speed * MAX_SUBSTEP).length()
                    < 1e-6
            );
            assert!((p.combined_velocity() - p.velocity - p.roll_velocity()).length() < 1e-6);
        }
    }

    #[test]
    fn roll_decays_independently_then_finish_cancels_without_changing_base() {
        let t = Tuning {
            roll_duration: 0.25,
            roll_impulse_decay: 2.,
            ..Default::default()
        };
        let w = World::default();
        let mut p = Player::new(Vec3::Y * 20.);
        p.velocity = Vec3::X * 8.;
        let mut control = p.clone();
        let input = Input {
            roll: true,
            camera_forward: Vec3::Z,
            ..Default::default()
        };
        p.step(&input, MAX_SUBSTEP, &w, &t);
        control.step(
            &Input {
                roll: false,
                ..input
            },
            MAX_SUBSTEP,
            &w,
            &t,
        );
        // A live tuning edit does not replace this owner's insertion-time decay.
        let edited = Tuning {
            roll_impulse_decay: 100.,
            ..t
        };
        p.step(&Input::default(), MAX_SUBSTEP, &w, &edited);
        control.step(&Input::default(), MAX_SUBSTEP, &w, &edited);
        assert!((p.roll_velocity().z - (t.roll_speed - 2. * MAX_SUBSTEP)).abs() < 1e-6);
        assert_eq!(p.velocity, control.velocity);
        p.step(&Input::default(), 0.3, &w, &edited);
        control.step(&Input::default(), 0.3, &w, &edited);
        assert_eq!(p.roll_velocity(), Vec3::ZERO);
        assert_eq!(p.velocity, control.velocity);
        assert_ne!(p.state, MovementState::Roll);
    }

    #[test]
    fn zero_duration_roll_finishes_immediately_without_orphan_impulse() {
        let t = Tuning {
            roll_duration: 0.,
            ..Default::default()
        };
        let w = World::default();
        let mut p = Player::new(Vec3::Y * 20.);
        p.velocity = Vec3::X * 8.;
        let mut control = p.clone();
        p.step(
            &Input {
                roll: true,
                ..Default::default()
            },
            MAX_SUBSTEP,
            &w,
            &t,
        );
        control.step(&Input::default(), MAX_SUBSTEP, &w, &t);
        assert_eq!(p.roll_velocity(), Vec3::ZERO);
        assert_eq!(p.velocity, control.velocity);
        assert_eq!(p.position, control.position);
        assert_ne!(p.state, MovementState::Roll);
        assert_eq!(p.air_rolls_left, 0);
    }

    #[test]
    fn jump_cleanup_distinguishes_slide_and_roll_owners() {
        // Isolate ordinary launch arithmetic; default staged wall service is tested in wall90_tests.
        // Seed a surviving owner outside the lab roll timer to exercise native
        // owner cleanup independently from the lab's action admission policy.
        for bullet in [false, true] {
            let mut p = Player::new(Vec3::Y * 20.);
            p.previously_crouch = true;
            p.slide_impulse.replace(Vec3::X * 5., 0.);
            p.roll_impulse.upsert(Vec3::Z * 7., 0.);
            p.step(
                &Input {
                    jump: true,
                    crouch: bullet,
                    ..Default::default()
                },
                MAX_SUBSTEP,
                &World::default(),
                &ordinary_only(Tuning::default()),
            );
            assert_eq!(p.slide_velocity(), Vec3::ZERO);
            assert_eq!(
                p.roll_velocity(),
                if bullet { Vec3::ZERO } else { Vec3::Z * 7. }
            );
        }
        let mut p = settled();
        p.slide_impulse.replace(Vec3::X * 5., 0.);
        p.roll_impulse.upsert(Vec3::Z * 7., 0.);
        p.step(
            &Input {
                jump: true,
                ..Default::default()
            },
            MAX_SUBSTEP,
            &floor(),
            &ordinary_only(Tuning::default()),
        );
        assert_eq!(p.slide_velocity(), Vec3::X * 5.);
        assert_eq!(p.roll_velocity(), Vec3::Z * 7.);
        p.respawn(Vec3::Y);
        assert_eq!(p.slide_velocity(), Vec3::ZERO);
        assert_eq!(p.roll_velocity(), Vec3::ZERO);
    }

    #[test]
    fn slide_replace_all_clears_roll_only_when_boost_inserts() {
        for permit_boost in [false, true] {
            let mut p = settled();
            p.velocity = Vec3::X * 8.;
            p.roll_impulse.upsert(Vec3::Z * 7., 0.);
            p.last_slide_boost = if permit_boost { -1000. } else { p.time };
            p.step(
                &Input {
                    crouch: true,
                    movement: Vec3::X,
                    ..Default::default()
                },
                MAX_SUBSTEP,
                &floor(),
                &Tuning::default(),
            );
            assert_eq!(
                p.roll_velocity(),
                if permit_boost {
                    Vec3::ZERO
                } else {
                    Vec3::Z * 7.
                }
            );
            assert_eq!(p.slide_velocity().length() > 0., permit_boost);
        }
    }

    #[test]
    fn roll_and_slide_compose_for_collision_without_folding_into_base() {
        let w = World {
            terrain: Default::default(),
            solids: vec![Solid::new(Vec3::new(1., 0., -5.), Vec3::new(2., 10., 5.))],
        };
        let mut p = Player::new(Vec3::new(0.69, 2., 0.));
        p.velocity.x = 5.;
        p.slide_impulse.replace(Vec3::X * 3., 0.);
        p.roll_impulse.upsert(Vec3::X * 8., 0.);
        let mut free = p.clone();
        free.step(
            &Input::default(),
            MAX_SUBSTEP,
            &World::default(),
            &Tuning::default(),
        );
        p.step(&Input::default(), MAX_SUBSTEP, &w, &Tuning::default());
        assert!(p.position.x <= 0.7);
        assert_eq!(p.velocity, free.velocity);
        assert_eq!(p.slide_velocity().x, 3.);
        assert_eq!(p.roll_velocity().x, 8.);
        assert_eq!(p.combined_velocity().x, p.velocity.x + 11.);
    }

    #[test]
    fn legacy_roll_json_defaults_and_captured_record_round_trip() {
        let mut value = serde_json::to_value(Tuning::default()).unwrap();
        value.as_object_mut().unwrap().remove("roll_impulse_decay");
        value["roll_speed"] = serde_json::json!(12.);
        let t: Tuning = serde_json::from_value(value).unwrap();
        assert_eq!(t.roll_speed, 12.);
        assert_eq!(t.roll_impulse_decay, 10.);
        assert!(
            Tuning {
                roll_impulse_decay: 0.,
                ..t
            }
            .is_valid()
        );
        for decay in [-1., f32::NAN, f32::INFINITY] {
            assert!(
                !Tuning {
                    roll_impulse_decay: decay,
                    ..t
                }
                .is_valid()
            );
        }
        let mut p = Player::new(Vec3::Y * 20.);
        p.roll_impulse.upsert(Vec3::X * 3., 2.);
        p.roll_impulse.evaluate(MAX_SUBSTEP);
        let mut value = serde_json::to_value(&p).unwrap();
        let mut restored: Player = serde_json::from_value(value.clone()).unwrap();
        restored.roll_impulse.upsert(Vec3::Z * 5., 99.);
        restored.roll_impulse.evaluate(0.5);
        assert_eq!(restored.roll_velocity(), Vec3::Z * 4.);
        value.as_object_mut().unwrap().remove("roll_impulse");
        let legacy: Player = serde_json::from_value(value).unwrap();
        assert_eq!(legacy.roll_velocity(), Vec3::ZERO);
    }
    #[test]
    fn zero_slide_boost_replace_all_still_removes_roll_owner() {
        let t = Tuning {
            slide_boost: 0.,
            ..Default::default()
        };
        let mut p = settled();
        p.velocity = Vec3::X * 8.;
        p.roll_impulse.upsert(Vec3::Z * 7., 0.);
        p.step(
            &Input {
                crouch: true,
                movement: Vec3::X,
                ..Default::default()
            },
            MAX_SUBSTEP,
            &floor(),
            &t,
        );
        assert_eq!(p.slide_velocity(), Vec3::ZERO);
        assert_eq!(p.roll_velocity(), Vec3::ZERO);
    }

    #[test]
    fn roll_owner_insertion_and_finish_preserve_slide_owner() {
        let t = Tuning {
            roll_duration: 0.125,
            ..Default::default()
        };
        let mut p = Player::new(Vec3::Y * 20.);
        p.slide_impulse.replace(Vec3::X * 5., 0.);
        p.step(
            &Input {
                roll: true,
                camera_forward: Vec3::Z,
                ..Default::default()
            },
            MAX_SUBSTEP,
            &World::default(),
            &t,
        );
        assert_eq!(p.slide_velocity(), Vec3::X * 5.);
        assert_eq!(p.roll_velocity(), Vec3::Z * t.roll_speed);
        p.step(&Input::default(), 0.25, &World::default(), &t);
        assert_eq!(p.slide_velocity(), Vec3::X * 5.);
        assert_eq!(p.roll_velocity(), Vec3::ZERO);
    }

    #[test]
    fn configured_special_allowances_initialize_consume_and_reset() {
        let t = Tuning {
            max_jumps: 4,
            max_bullet_jumps: 2,
            max_air_rolls: 2,
            roll_duration: 0.,
            ..Default::default()
        };
        let mut p = Player::new(Vec3::Y * 20.);
        let empty = World::default();
        let bullet = Input {
            jump: true,
            crouch: true,
            ..Default::default()
        };
        p.step(&bullet, MAX_SUBSTEP, &empty, &t);
        assert_eq!(
            (p.jumps_remaining, p.bullet_jumps_left, p.air_rolls_left),
            (3, 1, 2)
        );
        p.step(&bullet, MAX_SUBSTEP, &empty, &t);
        assert_eq!((p.jumps_remaining, p.bullet_jumps_left), (2, 0));
        let roll = Input {
            roll: true,
            ..Default::default()
        };
        p.step(&roll, MAX_SUBSTEP, &empty, &t);
        p.step(&roll, MAX_SUBSTEP, &empty, &t);
        assert_eq!(p.air_rolls_left, 0);
        let mut wall = floor();
        wall.solids
            .push(Solid::new(Vec3::new(2., 0., -10.), Vec3::new(3., 8., 10.)));
        p.position = Vec3::new(1.699, 3., 0.);
        p.step(
            &Input {
                jump: true,
                ..Default::default()
            },
            MAX_SUBSTEP,
            &wall,
            &t,
        );
        finish_query(&mut p, &Input::default(), &wall, &t);
        assert_eq!(p.state, MovementState::WallJump);
        assert_eq!(
            (p.jumps_remaining, p.bullet_jumps_left, p.air_rolls_left),
            (3, 2, 2)
        );
        p.bullet_jumps_left = 0;
        p.air_rolls_left = 0;
        p.position = Vec3::new(0., 0.001, 0.);
        p.velocity = Vec3::NEG_Y;
        p.step(&Input::default(), MAX_SUBSTEP, &wall, &t);
        assert!(p.grounded);
        assert_eq!(
            (p.jumps_remaining, p.bullet_jumps_left, p.air_rolls_left),
            (3, 0, 0),
            "collision publication does not itself refill budgets"
        );
        p.step(&Input::default(), MAX_SUBSTEP, &wall, &t);
        assert_eq!(p.last_jump_reset(), ResetDecision::Landing);
        assert_eq!(
            (p.jumps_remaining, p.bullet_jumps_left, p.air_rolls_left),
            (4, 2, 2)
        );
        p.respawn(Vec3::Y * 20.);
        p.step(&Input::default(), MAX_SUBSTEP, &empty, &t);
        assert_eq!(
            (p.jumps_remaining, p.bullet_jumps_left, p.air_rolls_left),
            (4, 2, 2)
        );
    }

    #[test]
    fn zero_special_allowances_disable_special_use_without_removing_general_jumps() {
        let t = Tuning {
            max_bullet_jumps: 0,
            max_air_rolls: 0,
            ..Default::default()
        };
        assert!(t.is_valid());
        let mut p = Player::new(Vec3::Y * 20.);
        p.step(
            &Input {
                roll: true,
                ..Default::default()
            },
            MAX_SUBSTEP,
            &World::default(),
            &t,
        );
        assert_eq!(p.roll_velocity(), Vec3::ZERO);
        assert_ne!(p.state, MovementState::Roll);
        p.step(
            &Input {
                jump: true,
                crouch: true,
                ..Default::default()
            },
            MAX_SUBSTEP,
            &World::default(),
            &t,
        );
        assert_ne!(p.state, MovementState::BulletJump);
        assert_eq!(p.jumps_remaining, t.max_jumps - 1);
        assert_eq!((p.bullet_jumps_left, p.air_rolls_left), (0, 0));
        // Grounded rolls do not consume the airborne allowance in this lab.
        let mut p = Player::new(Vec3::Y * 0.01);
        p.step(&Input::default(), 0.1, &floor(), &t);
        assert!(p.grounded);
        p.step(
            &Input {
                roll: true,
                ..Default::default()
            },
            MAX_SUBSTEP,
            &floor(),
            &t,
        );
        assert_eq!(p.state, MovementState::Roll);
        assert_eq!(p.air_rolls_left, 0);
    }

    #[test]
    fn special_allowance_legacy_defaults_and_reload_apply_at_reset() {
        let mut json = serde_json::to_value(Tuning::default()).unwrap();
        json.as_object_mut().unwrap().remove("max_bullet_jumps");
        json.as_object_mut().unwrap().remove("max_air_rolls");
        let legacy: Tuning = serde_json::from_value(json).unwrap();
        assert_eq!((legacy.max_bullet_jumps, legacy.max_air_rolls), (1, 1));
        let mut p = Player::new(Vec3::Y * 20.);
        p.step(&Input::default(), MAX_SUBSTEP, &World::default(), &legacy);
        p.bullet_jumps_left = 0;
        p.air_rolls_left = 0;
        let edited = Tuning {
            max_bullet_jumps: 3,
            max_air_rolls: 4,
            ..legacy
        };
        p.step(&Input::default(), MAX_SUBSTEP, &World::default(), &edited);
        assert_eq!((p.bullet_jumps_left, p.air_rolls_left), (0, 0));
        p.respawn(Vec3::Y * 20.);
        p.step(&Input::default(), MAX_SUBSTEP, &World::default(), &edited);
        assert_eq!((p.bullet_jumps_left, p.air_rolls_left), (3, 4));
    }

    #[test]
    fn optional_air_roll_jump_cost_gates_and_spends_shared_allowance() {
        let t = Tuning {
            max_air_rolls: 3,
            air_roll_consumes_jump: true,
            roll_duration: 0.,
            ..Default::default()
        };
        let mut p = Player::new(Vec3::Y * 20.);
        let roll = Input {
            roll: true,
            ..Default::default()
        };
        p.step(&roll, MAX_SUBSTEP, &World::default(), &t);
        assert_eq!((p.jumps_remaining, p.air_rolls_left), (1, 2));
        p.step(&roll, MAX_SUBSTEP, &World::default(), &t);
        assert_eq!((p.jumps_remaining, p.air_rolls_left), (0, 1));
        p.step(&roll, MAX_SUBSTEP, &World::default(), &t);
        assert_eq!((p.jumps_remaining, p.air_rolls_left), (0, 1));
        p.step(
            &Input {
                jump: true,
                ..Default::default()
            },
            MAX_SUBSTEP,
            &World::default(),
            &t,
        );
        assert_eq!(p.jumps_remaining, 0);
        assert_eq!(p.bullet_jumps_left, t.max_bullet_jumps);
        let mut json = serde_json::to_value(t).unwrap();
        json.as_object_mut()
            .unwrap()
            .remove("air_roll_consumes_jump");
        let legacy: Tuning = serde_json::from_value(json).unwrap();
        assert!(!legacy.air_roll_consumes_jump);
        p.step(&roll, MAX_SUBSTEP, &World::default(), &legacy);
        assert_eq!((p.jumps_remaining, p.air_rolls_left), (0, 0));
    }

    #[test]
    fn wall_height_launch_preserves_owners_and_composes_named_gravity() {
        let t = Tuning {
            gravity: 12.,
            launch_gravity: Some(9.81),
            bullet_jump_gravity_scale: 0.25,
            wall_jump: WallJumpTuning {
                facing_gravity_scale: 0.5,
                facing_gravity_duration: 2.,
                ..Default::default()
            },
            ..Default::default()
        };
        let w = World {
            terrain: Default::default(),
            solids: vec![Solid::new(Vec3::new(2., 0., -10.), Vec3::new(3., 30., 10.))],
        };
        let mut p = Player::new(Vec3::new(1.699, 3., 0.));
        p.bullet_gravity_remaining = 2.;
        p.slide_impulse.replace(Vec3::Z * 3., 0.);
        p.roll_impulse.upsert(Vec3::Z * 2., 0.);
        p.step(
            &Input {
                jump: true,
                movement: Vec3::X,
                ..Default::default()
            },
            MAX_SUBSTEP,
            &w,
            &t,
        );
        finish_query(&mut p,&Input{movement:Vec3::X,..Default::default()},&w,&t);
        let target =
            (2. * 9.81 * t.jump_height * t.wall_jump.perpendicular_height_multiplier).sqrt();
        assert!((p.velocity.y - target).abs() < 1e-5);
        assert_eq!(p.wall_jump_subtype, 2);
        assert_eq!(p.slide_velocity(), Vec3::Z * 3.);
        assert_eq!(p.roll_velocity(), Vec3::Z * 2.);
        assert!((p.bullet_gravity_remaining - (2. - 2.*MAX_SUBSTEP)).abs() < 1e-6);
        assert!((p.wall_gravity_remaining - (2. - MAX_SUBSTEP)).abs() < 1e-6);
        p.step(&Input::default(), MAX_SUBSTEP, &w, &t);
        assert!((p.velocity.y - (target - 12. * 0.25 * 0.5 * MAX_SUBSTEP)).abs() < 1e-5);
    }

    #[test]
    fn wall_zero_height_skips_gravity_and_signed_timing_scales_height() {
        let w = World {
            terrain: Default::default(),
            solids: vec![Solid::new(Vec3::new(2., 0., -10.), Vec3::new(3., 30., 10.))],
        };
        let input = Input {
            jump: true,
            movement: Vec3::X,
            ..Default::default()
        };
        let mut p = Player::new(Vec3::new(1.699, 3., 0.));
        let zero = Tuning {
            wall_jump: WallJumpTuning {
                perpendicular_height_multiplier: 0.,
                ..Default::default()
            },
            ..Default::default()
        };
        p.step(&input, MAX_SUBSTEP, &w, &zero);
        finish_query(&mut p,&input,&w,&zero);
        assert_eq!(p.velocity.y, 0.);
        p.step(&Input::default(), MAX_SUBSTEP, &w, &zero);
        assert!(p.velocity.y < 0.);
        let t = Tuning {
            launch_gravity: Some(9.81),
            wall_jump: WallJumpTuning {
                minimum_timing_multiplier: 0.2,
                ..Default::default()
            },
            ..Default::default()
        };
        let mut p = Player::new(Vec3::new(1.699, 3., 0.));
        p.wall_jump_timer = -0.25;
        p.step(&input, MAX_SUBSTEP, &w, &t);
        finish_query(&mut p,&input,&w,&t);
        let timing = 1. + (0.25 + 2.*MAX_SUBSTEP) / t.wall_jump.timing_window * (0.2 - 1.);
        let target =
            (2. * 9.81 * t.jump_height * t.wall_jump.perpendicular_height_multiplier * timing)
                .sqrt();
        assert!((p.velocity.y - target).abs() < 1e-5);
        p.step(&Input::default(), 1.1, &World::default(), &t);
        assert!(p.wall_jump_timer < 0.);
    }

    #[test]
    fn latch_entry_removal_is_distinct_from_direct_wall_launch() {
        let w = World {
            terrain: Default::default(),
            solids: vec![Solid::new(Vec3::new(2., 0., -10.), Vec3::new(3., 30., 10.))],
        };
        let mut p = Player::new(Vec3::new(1.699, 3., 0.));
        p.velocity.x = 4.;
        p.bullet_gravity_remaining = 2.;
        p.step(
            &Input {
                aim: true,
                ..Default::default()
            },
            MAX_SUBSTEP,
            &w,
            &Tuning::default(),
        );
        assert_eq!(p.state, MovementState::WallLatch);
        assert_eq!(p.bullet_gravity_remaining, 0.);
    }

    #[test]
    fn rejected_wall_entry_keeps_early_gravity_removal_without_budget_reset() {
        let w = World {
            terrain: Default::default(),
            solids: vec![Solid::new(Vec3::new(2., 0., -10.), Vec3::new(3., 30., 10.))],
        };
        let mut p = Player::new(Vec3::new(1.699, 3., 0.));
        // Complete initialization's separate lifecycle reset before depleting
        // budgets; the wall request must not be blamed for that first reset.
        p.step(
            &Input::default(),
            MAX_SUBSTEP,
            &World::default(),
            &Tuning::default(),
        );
        p.velocity.x = -4.;
        p.bullet_gravity_remaining = 2.;
        p.jumps_remaining = 0;
        p.bullet_jumps_left = 0;
        p.air_rolls_left = 0;
        p.step(
            &Input {
                aim: true,
                ..Default::default()
            },
            MAX_SUBSTEP,
            &w,
            &Tuning::default(),
        );
        assert_eq!(p.state, MovementState::AimGlide);
        assert_eq!(p.bullet_gravity_remaining, 0.);
        assert_eq!(
            (p.jumps_remaining, p.bullet_jumps_left, p.air_rolls_left),
            (0, 0, 0)
        );
    }

    #[test]
    fn disabled_wall_entry_preserves_bullet_gravity_and_continuation_skips_entry_gate() {
        let w = World {
            terrain: Default::default(),
            solids: vec![Solid::new(Vec3::new(2., 0., -10.), Vec3::new(3., 30., 10.))],
        };
        let mut p = Player::new(Vec3::new(1.699, 3., 0.));
        p.velocity.x = 4.;
        p.bullet_gravity_remaining = 2.;
        let mut t = Tuning::default();
        t.wall_entry.enabled = false;
        let input = Input {
            aim: true,
            ..Default::default()
        };
        p.step(&input, MAX_SUBSTEP, &w, &t);
        assert_eq!(p.state, MovementState::AimGlide);
        assert!((p.bullet_gravity_remaining - (2. - MAX_SUBSTEP)).abs() < 1e-6);
        t.wall_entry.enabled = true;
        p.velocity.x = 4.;
        p.step(&input, MAX_SUBSTEP, &w, &t);
        assert_eq!(p.state, MovementState::WallLatch);
        assert_eq!(p.velocity, Vec3::ZERO);
        let position = p.position;
        // Entry zeroes velocity. Continuation must not fail its own entry gate,
        // even if the next-entry setting is disabled in the meantime.
        t.wall_entry.enabled = false;
        p.step(&input, MAX_SUBSTEP, &w, &t);
        assert_eq!(p.state, MovementState::WallLatch);
        assert_eq!(p.position, position);
    }

    #[test]
    fn wall_entry_excludes_vertical_scalar_and_additive_owners() {
        let w = World {
            terrain: Default::default(),
            solids: vec![Solid::new(Vec3::new(2., 0., -10.), Vec3::new(3., 30., 10.))],
        };
        let mut p = Player::new(Vec3::new(1.699, 3., 0.));
        p.velocity.y = 20.;
        p.slide_impulse.replace(Vec3::X * 4., 0.);
        p.step(
            &Input {
                aim: true,
                ..Default::default()
            },
            MAX_SUBSTEP,
            &w,
            &Tuning::default(),
        );
        assert_eq!(p.state, MovementState::AimGlide);
        assert_eq!(p.slide_velocity(), Vec3::X * 4.);
    }

    #[test]
    fn wall_tuning_migration_and_state_serialization_are_explicit() {
        let mut value = serde_json::to_value(Tuning::default()).unwrap();
        value.as_object_mut().unwrap().remove("wall_jump");
        value["wall_jump_horizontal_speed"] = serde_json::json!(999.);
        value["wall_jump_vertical_speed"] = serde_json::json!(999.);
        value["wall_jump_cooldown"] = serde_json::json!(999.);
        let migrated: Tuning = serde_json::from_value(value).unwrap();
        assert!(migrated.is_valid());
        assert_eq!(migrated.wall_jump.height_multiplier, 2.5);
        assert_eq!(migrated.wall_jump.normal_cooldown, 1.);
        let mut p = Player::new(Vec3::Y * 20.);
        p.wall_jump_timer = -0.2;
        p.wall_gravity_remaining = -1.;
        p.wall_gravity_scale = 0.5;
        let encoded = serde_json::to_value(&p).unwrap();
        let restored: Player = serde_json::from_value(encoded.clone()).unwrap();
        assert_eq!(restored.wall_jump_timer, -0.2);
        assert_eq!(restored.wall_gravity_remaining, -1.);
        let mut legacy = encoded;
        for field in [
            "wall_jump_timer",
            "wall_gravity_remaining",
            "wall_gravity_scale",
            "wall_jump_subtype",
        ] {
            legacy.as_object_mut().unwrap().remove(field);
        }
        let restored: Player = serde_json::from_value(legacy).unwrap();
        assert_eq!(restored.wall_jump_timer, f32::MAX);
        assert_eq!(restored.wall_gravity_remaining, 0.);
        assert_eq!(restored.wall_gravity_scale, 1.);
    }
}

#[cfg(test)]
mod engine_publication_tests {
    use super::*;
    #[test]
    fn serialized_player_resumes_identical_center_contact_and_motion() {
        let world = World::graybox();
        let tuning = Tuning::default();
        let mut original = Player::new(Vec3::new(0., 0.01, 8.));
        original.step(&Input::default(), 0.05, &world, &tuning);
        assert!(original.engine_contact.initialized);
        original.engine_contact.center[0] += 1.0e-12;
        original.engine_contact.reports[32] = 0xa5;
        let mut restored: Player =
            serde_json::from_str(&serde_json::to_string(&original).unwrap()).unwrap();
        assert_eq!(
            restored.engine_contact.center[0].to_bits(),
            original.engine_contact.center[0].to_bits()
        );
        let input = Input {
            movement: Vec3::X,
            ..Input::default()
        };
        for _ in 0..4 {
            original.step(&input, 1. / 120., &world, &tuning);
            restored.step(&input, 1. / 120., &world, &tuning);
        }
        assert_eq!(
            serde_json::to_string(&restored).unwrap(),
            serde_json::to_string(&original).unwrap()
        );
        assert_eq!(restored.engine_contact.reports[32], 0xa5);
    }
    #[test]
    fn prior_published_defer_delays_landing_budget_reset() {
        let mut p = Player::new(Vec3::ZERO);
        let t = Tuning::default();
        p.engine_contact.initialized = true;
        p.engine_contact.mask = 4;
        p.engine_contact.defer = true;
        p.jump_lifecycle.reset.cached_air = true;
        p.jumps_remaining = 0;
        p.jump_lifecycle.properties = crate::jump_lifecycle::JumpProperties {
            evaluated: 3,
            staged: 3,
        };
        p.advance_lab_jump_lifecycle(&Input::default(), MAX_SUBSTEP, &World::default(), &t);
        assert_eq!(p.jumps_remaining, 0);
        assert_ne!(p.last_jump_reset(), ResetDecision::Landing);
        p.engine_contact.defer = false;
        p.advance_lab_jump_lifecycle(&Input::default(), MAX_SUBSTEP, &World::default(), &t);
        assert_eq!(p.last_jump_reset(), ResetDecision::Landing);
        assert_eq!(p.jumps_remaining, t.max_jumps);
    }
    #[test]
    fn invalid_contact_policy_rejects_before_player_mutation() {
        let mut p = Player::new(Vec3::ZERO);
        let mut t = Tuning::default();
        t.collision.engine.adjustment_lower = f32::NAN;
        let before = serde_json::to_string(&p).unwrap();
        p.step(
            &Input {
                jump: true,
                ..Input::default()
            },
            0.01,
            &World::default(),
            &t,
        );
        assert_eq!(serde_json::to_string(&p).unwrap(), before);
    }
}

#[cfg(test)]
#[path = "slope_composition88.rs"]
mod slope_composition88;

#[cfg(test)]
#[path="wall90_tests.rs"]
mod wall90_tests;
