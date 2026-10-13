//! Supplied deferred request/event traces, not a native clip or event scheduler.
//! Physics outcomes and callback mutations are explicit fixtures; actual Player
//! launch admission and physical integration are checked separately by the root.

use movement_lab::jump_lifecycle::{
    AirSample, HeightImpulse, HeightImpulseEffects, JumpLifecycle, JumpProperties,
    JumpStartImpulseContext, LifecycleInputs, ResetDecision,
};

fn eligible_event() -> JumpStartImpulseContext {
    JumpStartImpulseContext {
        own_sender: true,
        is_jump_start_impulse: true,
        animation_triggered_jump: true,
        physics_available: true,
    }
}

fn begin_eligible(state: &mut JumpLifecycle) -> HeightImpulse {
    let eligible = state.check_jump_start_impulse(eligible_event()).unwrap();
    state.begin_jump_start_impulse(eligible)
}

fn supported_phase(dt: f32) -> LifecycleInputs {
    LifecycleInputs {
        air: AirSample::SuppliedRolePredicate(false),
        repair_enabled: true,
        animation_triggered_jump: true,
        shared_remaining: 0,
        effective_shared_maximum: 2,
        dt,
    }
}

fn event_effects() -> HeightImpulseEffects {
    HeightImpulseEffects {
        request_movement_flag_updates: true,
        request_resource_debit: true,
    }
}

#[test]
fn repeated_requests_replace_height_but_negative_height_preserves_previous_value() {
    let mut state = JumpLifecycle::default();
    state.reset.suppress_once = true;
    state.properties.evaluated = 3;
    state.request_deferred_height(-2.).unwrap();
    assert!(state.deferred_pending);
    assert_eq!(state.deferred_height, -1.);
    for (requested, stored) in [(4., 4.), (-1., 4.), (7., 7.), (-8., 7.), (0., 0.)] {
        state.request_deferred_height(requested).unwrap();
        assert!(state.deferred_pending);
        assert_eq!(state.deferred_height, stored);
        assert_eq!(
            state.properties,
            JumpProperties {
                evaluated: 3,
                staged: 1
            }
        );
        assert!(state.reset.suppress_once, "request cannot write C01");
    }
    let attempt = begin_eligible(&mut state);
    assert_eq!(attempt.height(), 0.);
    assert!(!state.deferred_pending);
    assert!(
        state.check_jump_start_impulse(eligible_event()).is_none(),
        "requests do not accumulate in a queue"
    );
}

#[test]
fn wrong_sender_air_jump_disabled_animation_and_missing_physics_do_not_consume() {
    let mut state = JumpLifecycle::default();
    state.request_deferred_height(5.).unwrap();
    state.reset.suppress_once = true;
    let before = state;
    for context in [
        JumpStartImpulseContext {
            own_sender: false,
            ..eligible_event()
        },
        // AIR_JUMP notification is represented by a non-impulse event identity.
        JumpStartImpulseContext {
            is_jump_start_impulse: false,
            ..eligible_event()
        },
        JumpStartImpulseContext {
            animation_triggered_jump: false,
            ..eligible_event()
        },
        JumpStartImpulseContext {
            physics_available: false,
            ..eligible_event()
        },
    ] {
        assert!(state.check_jump_start_impulse(context).is_none());
        assert_eq!(state, before);
    }
    state.deferred_pending = false;
    let before = state;
    assert!(state.check_jump_start_impulse(eligible_event()).is_none());
    assert_eq!(state, before);
}

#[test]
fn event_captures_late_height_and_clears_pending_before_the_physics_attempt() {
    let mut state = JumpLifecycle::default();
    state.request_deferred_height(2.).unwrap();
    // Supplied prelaunch callback overwrites height before this compact phase.
    // This fixture does not simulate the handler's preceding native callbacks.
    state.deferred_height = 9.;
    state.properties = JumpProperties {
        evaluated: 2,
        staged: 4,
    };
    let graph_before = state.properties;
    let attempt = begin_eligible(&mut state);
    assert!(
        !state.deferred_pending,
        "physics must observe cleared pending"
    );
    assert_eq!(state.deferred_height, 9., "height clears after the attempt");
    assert_eq!(attempt.height(), 9.);
    assert_eq!(state.properties, graph_before);
    assert_eq!(
        state.complete_jump_start_impulse(attempt, true),
        event_effects()
    );
    assert_eq!(state.deferred_height, -1.);
    assert!(state.reset.suppress_once);
    assert_eq!(
        state.properties, graph_before,
        "event completion never stages IV_JUMP"
    );
}

#[test]
fn captured_eligibility_survives_prelaunch_guard_mutation_and_reads_new_height() {
    let mut state = JumpLifecycle::default();
    state.request_deferred_height(2.).unwrap();
    let before_check = state;
    let eligible = state.check_jump_start_impulse(eligible_event()).unwrap();
    assert_eq!(
        state, before_check,
        "guard phase neither consumes nor captures height"
    );
    // A supplied prelaunch callback clears pending and changes the saved height.
    // A later guard sample would fail, but the reached event is not readmitted.
    state.deferred_pending = false;
    state.deferred_height = 13.;
    assert!(
        state
            .check_jump_start_impulse(JumpStartImpulseContext {
                animation_triggered_jump: false,
                ..eligible_event()
            })
            .is_none()
    );
    let graph_before = state.properties;
    let attempt = state.begin_jump_start_impulse(eligible);
    assert_eq!(attempt.height(), 13.);
    assert!(!state.deferred_pending);
    assert_eq!(state.properties, graph_before);
    assert_eq!(
        state.complete_jump_start_impulse(attempt, false),
        event_effects()
    );
    assert_eq!(state.deferred_height, -1.);
}

#[test]
fn failed_attempt_clears_height_and_previous_suppression_but_still_requests_effects() {
    let mut state = JumpLifecycle::default();
    state.request_deferred_height(3.).unwrap();
    state.reset.suppress_once = true;
    state.staged_land_style = 6.;
    let graph_before = state.properties;
    let attempt = begin_eligible(&mut state);
    assert_eq!(
        state.complete_jump_start_impulse(attempt, false),
        event_effects()
    );
    assert!(!state.deferred_pending);
    assert_eq!(state.deferred_height, -1.);
    assert!(
        !state.reset.suppress_once,
        "false is an overwrite, not an OR"
    );
    assert_eq!(state.properties, graph_before);
    assert_eq!(state.staged_land_style, 6.);
}

#[test]
fn request_during_attempt_remains_pending_while_completion_clears_its_height() {
    for succeeded in [false, true] {
        let mut state = JumpLifecycle::default();
        state.request_deferred_height(2.).unwrap();
        let first = begin_eligible(&mut state);
        // Simulate a height-call callback requesting a new launch.
        state.request_deferred_height(11.).unwrap();
        assert_eq!(
            first.height(),
            2.,
            "begun command retains its supplied height"
        );
        assert!(state.deferred_pending);
        assert_eq!(state.deferred_height, 11.);
        assert_eq!(
            state.complete_jump_start_impulse(first, succeeded),
            event_effects()
        );
        assert!(
            state.deferred_pending,
            "completion cannot re-clear new pending"
        );
        assert_eq!(
            state.deferred_height, -1.,
            "native post-call write clears even replaced height"
        );
        assert_eq!(state.reset.suppress_once, succeeded);
        let second = begin_eligible(&mut state);
        assert_eq!(
            second.height(),
            -1.,
            "next attempt requests controller default height"
        );
    }
}

#[test]
fn invalid_request_height_preserves_all_phase_state() {
    let mut state = JumpLifecycle::default();
    state.request_deferred_height(8.).unwrap();
    state.reset.suppress_once = true;
    state.property_four_seconds = 1.25;
    state.staged_land_style = 2.;
    let before = state;
    for height in [f32::NAN, f32::INFINITY, f32::NEG_INFINITY] {
        assert!(state.request_deferred_height(height).is_err());
        assert_eq!(state, before);
    }
}

#[test]
fn delayed_request_tracks_staged_state_until_a_supplied_event_arrives() {
    let mut state = JumpLifecycle::default();
    state.request_deferred_height(4.).unwrap();
    // Pending one is a graph request, not evaluated history, but either one
    // blocks idle-empty repair and enters the tracking condition.
    let effects = state.evaluate_reset(supported_phase(0.5)).unwrap();
    assert_eq!(effects.decision, ResetDecision::None);
    assert!(!effects.refill_budgets);
    assert_eq!(state.reset.tracked_seconds, 0.5);
    assert_eq!(
        state.properties,
        JumpProperties {
            evaluated: 0,
            staged: 1
        }
    );
    assert!(state.deferred_pending);
    state.promote_staged_for_lab();
    state.evaluate_reset(supported_phase(0.5)).unwrap();
    assert_eq!(state.reset.tracked_seconds, 1.);
    let attempt = begin_eligible(&mut state);
    assert_eq!(attempt.height(), 4.);
    let graph_before = state.properties;
    state.complete_jump_start_impulse(attempt, true);
    assert_eq!(state.properties, graph_before);
    assert_eq!(
        state.evaluate_reset(supported_phase(0.5)).unwrap().decision,
        ResetDecision::Suppressed
    );
    assert_eq!(
        state.reset.tracked_seconds, 1.,
        "suppression skips tracking this phase"
    );
}

#[test]
fn tracking_timeout_preserves_request_until_explicit_reset_cancels_it() {
    let mut state = JumpLifecycle::default();
    state.request_deferred_height(6.).unwrap();
    state.staged_land_style = 7.;
    let effects = state.evaluate_reset(supported_phase(2.)).unwrap();
    assert_eq!(effects.decision, ResetDecision::StationaryRepair);
    assert!(effects.clear_land_style && effects.check_repair_posture_cleanup);
    assert!(state.deferred_pending);
    assert_eq!(state.deferred_height, 6.);
    assert_eq!(
        state.properties,
        JumpProperties {
            evaluated: 0,
            staged: 4
        }
    );
    assert_eq!(state.staged_land_style, 0.);
    // Pending remains consumable immediately after the stationary repair.
    let mut delivered = state;
    let attempt = begin_eligible(&mut delivered);
    assert_eq!(attempt.height(), 6.);
    delivered.complete_jump_start_impulse(attempt, true);
    assert_eq!(delivered.properties.staged, 4);
    // Without delivery, subsequent untracked pending takes explicit reset.
    let effects = state.evaluate_reset(supported_phase(0.25)).unwrap();
    assert_eq!(effects.decision, ResetDecision::ExplicitDeferredReset);
    assert!(effects.physics_reset_callback);
    assert!(!state.deferred_pending);
    assert_eq!(state.deferred_height, -1.);
    assert_eq!(state.properties, JumpProperties::default());
    assert!(state.check_jump_start_impulse(eligible_event()).is_none());
}

#[test]
fn disabled_tracking_selects_explicit_reset_before_pending_event() {
    let mut state = JumpLifecycle::default();
    state.request_deferred_height(4.).unwrap();
    let effects = state
        .evaluate_reset(LifecycleInputs {
            animation_triggered_jump: false,
            ..supported_phase(0.1)
        })
        .unwrap();
    assert_eq!(effects.decision, ResetDecision::ExplicitDeferredReset);
    assert_eq!(state.deferred_height, -1.);
    assert!(!state.deferred_pending);
    assert_eq!(state.properties, JumpProperties::default());
    assert!(state.check_jump_start_impulse(eligible_event()).is_none());
}

#[test]
fn legacy_lifecycle_defaults_land_style_and_roundtrip_keeps_pending_request() {
    let mut state = JumpLifecycle::default();
    state.request_deferred_height(7.).unwrap();
    let mut encoded = serde_json::to_value(state).unwrap();
    encoded.as_object_mut().unwrap().remove("staged_land_style");
    let restored: JumpLifecycle = serde_json::from_value(encoded).unwrap();
    assert_eq!(restored, state);
    assert_eq!(restored.staged_land_style, 0.);
}
