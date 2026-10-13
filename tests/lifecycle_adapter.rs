//! Independent adapter checks. Role, graph promotion and collision scheduling
//! below are supplied LAB policies, not observations of native gameplay.

use glam::Vec3;
use movement_lab::impulse::DecayingImpulse;
use movement_lab::jump_lifecycle::{
    AirSample, JumpLifecycle, JumpProperties, LifecycleInputs, ResetDecision, ResetEffects,
    ResetState,
};
use movement_lab::{Input, Player, Solid, Tuning, World};

fn inputs(air: AirSample, dt: f32) -> LifecycleInputs {
    LifecycleInputs {
        air,
        repair_enabled: true,
        animation_triggered_jump: true,
        shared_remaining: 1,
        effective_shared_maximum: 2,
        dt,
    }
}

fn supplied_ground(dt: f32) -> LifecycleInputs {
    inputs(AirSample::SuppliedRolePredicate(false), dt)
}

fn pending_state() -> JumpLifecycle {
    JumpLifecycle {
        reset: ResetState {
            tracked_seconds: 0.75,
            ..Default::default()
        },
        properties: JumpProperties {
            evaluated: 2,
            staged: 1,
        },
        deferred_pending: true,
        deferred_height: 7.,
        property_four_seconds: 0.5,
        staged_land_style: 5.,
        staged_dodge: 0.,
    }
}

#[test]
fn supplied_role_can_land_while_lab_support_defers_cached_air() {
    let mut supplied = pending_state();
    supplied.reset.cached_air = true;
    let mut lab = supplied;
    assert_eq!(
        supplied
            .evaluate_reset(supplied_ground(0.125))
            .unwrap()
            .decision,
        ResetDecision::Landing
    );
    assert_eq!(
        lab.evaluate_reset(inputs(
            AirSample::LabSupport {
                supported: true,
                defer_while_cached_air: true,
            },
            0.125,
        ))
        .unwrap(),
        ResetEffects::default()
    );
    assert!(lab.reset.cached_air);
    assert_eq!(lab.properties, supplied.properties);
    assert_eq!(
        lab.evaluate_reset(inputs(
            AirSample::LabSupport {
                supported: true,
                defer_while_cached_air: false,
            },
            0.125,
        ))
        .unwrap()
        .decision,
        ResetDecision::Landing
    );
    // A supplied role also has authority to remain airborne despite support.
    assert!(AirSample::SuppliedRolePredicate(true).resolve(false));
}

#[test]
fn landing_requests_projection_without_consuming_deferred_or_graph_state() {
    let mut state = pending_state();
    state.reset.cached_air = true;
    let before = state;
    let mut input = supplied_ground(0.125);
    input.repair_enabled = false;
    input.effective_shared_maximum = 0;
    assert_eq!(
        state.evaluate_reset(input).unwrap(),
        ResetEffects {
            decision: ResetDecision::Landing,
            refill_budgets: true,
            landing_callback: true,
            remove_vertical_additive_motion: true,
            ..Default::default()
        }
    );
    assert!(!state.reset.cached_air);
    assert_eq!(state.properties, before.properties);
    assert_eq!(state.deferred_pending, before.deferred_pending);
    assert_eq!(state.deferred_height, before.deferred_height);
    assert_eq!(state.reset.tracked_seconds, before.reset.tracked_seconds);
    assert_eq!(state.property_four_seconds, before.property_four_seconds);
    assert_eq!(state.staged_land_style, before.staged_land_style);
    state.properties.evaluated = 4;
    state.reset.cached_air = true;
    assert!(
        !state
            .evaluate_reset(input)
            .unwrap()
            .remove_vertical_additive_motion
    );
}

#[test]
fn explicit_reset_consumes_pending_while_stationary_repair_preserves_it() {
    let mut explicit = pending_state();
    explicit.properties = JumpProperties {
        evaluated: 0,
        staged: 3,
    };
    assert_eq!(
        explicit.evaluate_reset(supplied_ground(0.25)).unwrap(),
        ResetEffects {
            decision: ResetDecision::ExplicitDeferredReset,
            refill_budgets: true,
            physics_reset_callback: true,
            ..Default::default()
        }
    );
    assert!(!explicit.deferred_pending);
    assert_eq!(explicit.deferred_height, -1.);
    assert_eq!(explicit.properties, JumpProperties::default());
    assert_eq!(explicit.property_four_seconds, 0.5);
    assert_eq!(explicit.staged_land_style, 5.);

    let mut repair = pending_state();
    repair.reset.tracked_seconds = 1.5;
    assert_eq!(
        repair.evaluate_reset(supplied_ground(0.5)).unwrap(),
        ResetEffects {
            decision: ResetDecision::StationaryRepair,
            refill_budgets: true,
            clear_land_style: true,
            check_repair_posture_cleanup: true,
            ..Default::default()
        }
    );
    assert!(repair.deferred_pending);
    assert_eq!(repair.deferred_height, 7.);
    assert_eq!(
        repair.properties,
        JumpProperties {
            evaluated: 2,
            staged: 4
        }
    );
    assert_eq!(repair.reset.tracked_seconds, 0.);
    assert_eq!(repair.property_four_seconds, 0.5);
    assert_eq!(repair.staged_land_style, 0.);
}

#[test]
fn suppressed_contact_loses_landing_only_after_explicit_sample_commit() {
    let mut state = pending_state();
    state.reset.cached_air = true;
    state.reset.suppress_once = true;
    let before = state;
    assert_eq!(
        state.evaluate_reset(supplied_ground(0.)).unwrap(),
        ResetEffects {
            decision: ResetDecision::Suppressed,
            ..Default::default()
        }
    );
    assert!(state.reset.cached_air);
    assert!(!state.reset.suppress_once);
    assert_eq!(state.properties, before.properties);
    assert_eq!(state.deferred_height, before.deferred_height);
    // Scheduler's later publication deliberately changes the next decision.
    state.reset.commit_air_sample(false);
    assert_eq!(
        state.evaluate_reset(supplied_ground(0.)).unwrap(),
        ResetEffects::default()
    );
}

#[test]
fn property_four_observes_evaluated_state_and_expires_strictly_after_two() {
    let mut state = JumpLifecycle {
        properties: JumpProperties {
            evaluated: 1,
            staged: 4,
        },
        property_four_seconds: 1.75,
        ..Default::default()
    };
    state.advance_property_four_time(0.25).unwrap();
    assert_eq!(state.property_four_seconds, 0.);
    assert_eq!(
        state.properties,
        JumpProperties {
            evaluated: 1,
            staged: 4
        }
    );
    state.promote_staged_for_lab();
    state.advance_property_four_time(2.).unwrap();
    assert_eq!(state.property_four_seconds, 2.);
    assert_eq!(
        state.properties,
        JumpProperties {
            evaluated: 4,
            staged: 4
        }
    );
    state.advance_property_four_time(f32::EPSILON * 2.).unwrap();
    assert_eq!(state.property_four_seconds, 0.);
    assert_eq!(
        state.properties,
        JumpProperties {
            evaluated: 4,
            staged: 0
        }
    );
    state.promote_staged_for_lab();
    assert_eq!(state.properties, JumpProperties::default());
}

#[test]
fn height_launch_suppresses_even_without_scalar_write_but_direct_launch_does_not() {
    for (height, written, suppressed) in [
        (true, false, true),
        (false, true, true),
        (false, false, false),
    ] {
        let mut state = pending_state();
        state.immediate_lab_launch(height, written);
        assert_eq!(state.reset.suppress_once, suppressed);
        assert_eq!(
            state.properties,
            JumpProperties {
                evaluated: 2,
                staged: 1
            }
        );
        assert!(state.deferred_pending);
        assert_eq!(state.deferred_height, 7.);
        assert_eq!(state.reset.tracked_seconds, 0.75);
        assert_eq!(state.property_four_seconds, 0.5);
    }
    let mut state = pending_state();
    state.reset.suppress_once = true;
    state.immediate_lab_launch(false, false);
    assert!(
        state.reset.suppress_once,
        "a no-write direct launch cannot clear earlier suppression"
    );
}

#[test]
fn airborne_dodge_stages_three_without_promoting_or_consuming_pending_request() {
    let mut state = pending_state();
    let before = state;
    state.immediate_lab_dodge(false);
    assert_eq!(state, before);
    state.immediate_lab_dodge(true);
    assert_eq!(
        state.properties,
        JumpProperties {
            evaluated: 2,
            staged: 3
        }
    );
    assert_eq!(state.deferred_height, before.deferred_height);
    assert!(state.deferred_pending);
    assert_eq!(state.reset, before.reset);
}

#[test]
fn invalid_phase_time_and_overflow_preserve_the_entire_adapter() {
    for dt in [-1., f32::NAN, f32::INFINITY] {
        let mut state = pending_state();
        state.reset.suppress_once = true;
        let before = state;
        assert!(state.evaluate_reset(supplied_ground(dt)).is_err());
        assert_eq!(state, before);
        assert!(state.advance_property_four_time(dt).is_err());
        assert_eq!(state, before);
    }
    let mut state = pending_state();
    state.reset.tracked_seconds = f32::MAX;
    let before = state;
    assert!(state.evaluate_reset(supplied_ground(f32::MAX)).is_err());
    assert_eq!(state, before);
    state.properties.evaluated = 4;
    state.property_four_seconds = f32::MAX;
    let before = state;
    assert!(state.advance_property_four_time(f32::MAX).is_err());
    assert_eq!(state, before);
}

#[test]
fn landing_projection_preserves_horizontal_owner_decay_and_evaluation_age() {
    let mut impulse = DecayingImpulse::default();
    impulse.replace(Vec3::new(6., 8., 0.), 2.);
    impulse.evaluate(0.5);
    impulse.remove_vertical_component();
    assert_eq!(impulse.velocity(), Vec3::new(6., 0., 0.));
    impulse.evaluate(0.5);
    assert_eq!(impulse.velocity(), Vec3::new(5., 0., 0.));
}

fn grounded_fixture() -> (Player, World, Tuning) {
    let world = World::boxes(vec![Solid::new(
        Vec3::new(-50., -2., -50.),
        Vec3::new(50., 0., 50.),
    )]);
    let tuning = Tuning::default();
    let mut player = Player::new(Vec3::new(0., 0.05, 0.));
    for _ in 0..120 {
        player.step(&Input::default(), 1. / 240., &world, &tuning);
    }
    assert!(player.grounded);
    (player, world, tuning)
}

#[test]
fn stable_support_preserves_partial_budgets_even_when_caps_change() {
    let (mut player, world, mut tuning) = grounded_fixture();
    player.jumps_remaining = 1;
    player.bullet_jumps_left = 0;
    player.air_rolls_left = 0;
    tuning.max_jumps = 4;
    tuning.max_bullet_jumps = 3;
    tuning.max_air_rolls = 2;
    player.step(&Input::default(), 1. / 240., &world, &tuning);
    assert!(player.grounded);
    assert_eq!(player.last_jump_reset(), ResetDecision::None);
    assert_eq!(
        (
            player.jumps_remaining,
            player.bullet_jumps_left,
            player.air_rolls_left
        ),
        (1, 0, 0)
    );
}

#[test]
fn physics_support_is_published_before_the_next_substep_landing_reset() {
    let (mut player, world, tuning) = grounded_fixture();
    player.step(
        &Input {
            jump: true,
            ..Default::default()
        },
        1. / 240.,
        &world,
        &tuning,
    );
    assert!(!player.grounded);
    assert_eq!(player.jumps_remaining, tuning.max_jumps - 1);
    assert_eq!(
        player.jump_lifecycle().properties,
        JumpProperties {
            evaluated: 3,
            staged: 3
        }
    );
    for _ in 0..1200 {
        player.step(&Input::default(), 1. / 240., &world, &tuning);
        if player.grounded {
            break;
        }
    }
    assert!(player.grounded);
    assert_eq!(player.last_jump_reset(), ResetDecision::None);
    assert_eq!(player.jumps_remaining, tuning.max_jumps - 1);
    player.step(&Input::default(), 1. / 240., &world, &tuning);
    assert_eq!(player.last_jump_reset(), ResetDecision::Landing);
    assert_eq!(player.jumps_remaining, tuning.max_jumps);
}

#[test]
fn invalid_public_step_cannot_advance_or_consume_launch_suppression() {
    let (mut player, world, tuning) = grounded_fixture();
    player.step(
        &Input {
            jump: true,
            ..Default::default()
        },
        1. / 240.,
        &world,
        &tuning,
    );
    assert!(player.jump_lifecycle().reset.suppress_once);
    let before = serde_json::to_value(&player).unwrap();
    for dt in [0., -1., 2.1, f32::NAN, f32::INFINITY] {
        player.step(&Input::default(), dt, &world, &tuning);
        assert_eq!(serde_json::to_value(&player).unwrap(), before);
    }
}
