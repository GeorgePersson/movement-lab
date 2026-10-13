//! Supplied landing callback phases, not a native style selector or graph runner.
//! Budgets and physical effects belong to Player; these tests keep callback graph
//! writes separate from the outer landing/refill and deferred impulse phases.

use movement_lab::jump_lifecycle::{
    AirSample, JumpLifecycle, JumpProperties, JumpStartImpulseContext, LandingAction,
    LandingCleanup, LandingCleanupGate, LifecycleInputs, ResetDecision, ResetState,
    landing_callback_eligible,
};

fn state() -> JumpLifecycle {
    JumpLifecycle {
        reset: ResetState {
            cached_air: true,
            suppress_once: true,
            tracked_seconds: 0.75,
        },
        properties: JumpProperties {
            evaluated: 2,
            staged: 3,
        },
        deferred_pending: false,
        deferred_height: 8.,
        property_four_seconds: 1.25,
        staged_land_style: 6.,
        staged_dodge: 9.,
    }
}

fn styled() -> LandingCleanupGate {
    LandingCleanupGate {
        actor_flag_four: false,
        inventory_blocks_styled_landing: false,
    }
}

fn cleanup(action: LandingAction) -> LandingCleanup {
    match action {
        LandingAction::Cleanup(packet) => packet,
        LandingAction::JumpStartImpulse => panic!("expected nonpending cleanup"),
    }
}

#[test]
fn caller_gate_distinguishes_evaluated_staged_and_animation_samples() {
    for evaluated in [1, 2, 3] {
        for staged in [0, 2, 3, 5, 255] {
            assert!(landing_callback_eligible(evaluated, staged, false));
            assert!(!landing_callback_eligible(evaluated, staged, true));
        }
        for staged in [1, 4] {
            assert!(!landing_callback_eligible(evaluated, staged, false));
        }
    }
    for evaluated in [0, 4, 5, 255] {
        for staged in [0, 1, 2, 3, 4, 255] {
            assert!(!landing_callback_eligible(evaluated, staged, false));
        }
    }
}

#[test]
fn caller_uses_staged_after_projection_but_before_animation_callback() {
    let mut lifecycle = state();
    lifecycle.properties.staged = 1;
    let evaluated_before_projection = lifecycle.properties.evaluated;
    // Supplied projection callback changes both current graph channels.
    lifecycle.properties = JumpProperties {
        evaluated: 0,
        staged: 2,
    };
    let staged_after_projection = lifecycle.properties.staged;
    // Anim.v150 may mutate staged after its getter has already returned.
    lifecycle.properties.staged = 4;
    assert!(landing_callback_eligible(
        evaluated_before_projection,
        staged_after_projection,
        false,
    ));
    assert!(!landing_callback_eligible(
        lifecycle.properties.evaluated,
        lifecycle.properties.staged,
        false,
    ));
    assert_eq!(lifecycle.properties.staged, 4, "gate itself cannot stage");
}

#[test]
fn callback_route_reads_current_pending_and_has_no_effects_itself() {
    for pending in [false, true] {
        for style in [0, 1, 2, 3, 4, 255] {
            let mut lifecycle = state();
            lifecycle.deferred_pending = pending;
            let before = lifecycle;
            let action = lifecycle.landing_callback_action(style);
            if pending {
                assert_eq!(action, LandingAction::JumpStartImpulse);
            } else {
                assert_eq!(
                    action,
                    LandingAction::Cleanup(LandingCleanup { style, subtype: 1 })
                );
            }
            assert_eq!(lifecycle, before);
        }
    }
}

#[test]
fn selector_phase_can_create_pending_after_caller_admission() {
    let mut lifecycle = state();
    assert!(landing_callback_eligible(2, 3, false));
    // The supplied selector callback queues a request before v1168 samples it.
    lifecycle.request_deferred_height(5.).unwrap();
    let before = lifecycle;
    assert_eq!(
        lifecycle.landing_callback_action(4),
        LandingAction::JumpStartImpulse
    );
    assert_eq!(lifecycle, before, "routing neither consumes nor launches");
}

#[test]
fn rejected_pending_event_does_not_fall_through_to_cleanup() {
    let mut lifecycle = state();
    lifecycle.deferred_pending = true;
    let before = lifecycle;
    assert_eq!(
        lifecycle.landing_callback_action(255),
        LandingAction::JumpStartImpulse
    );
    // The event handler may reject later, after the pending callback branch was
    // selected. There is no cleanup packet on that branch to apply afterward.
    assert!(
        lifecycle
            .check_jump_start_impulse(JumpStartImpulseContext {
                own_sender: true,
                is_jump_start_impulse: true,
                animation_triggered_jump: true,
                physics_available: false,
            })
            .is_none()
    );
    assert_eq!(lifecycle, before);
}

#[test]
fn cleanup_preserves_request_created_during_the_posture_phase() {
    let mut lifecycle = state();
    let packet = cleanup(lifecycle.landing_callback_action(3));
    // Nonpending was already selected. Posture/notification callbacks can now
    // request a jump; cleanup must not reselect the branch or consume that request.
    lifecycle.request_deferred_height(11.).unwrap();
    let before = lifecycle;
    lifecycle.apply_landing_cleanup(packet, styled());
    let mut expected = before;
    expected.staged_land_style = 3.;
    expected.properties.staged = 4;
    assert_eq!(lifecycle, expected);
    assert!(lifecycle.deferred_pending);
    assert_eq!(lifecycle.deferred_height, 11.);
    assert!(lifecycle.reset.suppress_once);
}

#[test]
fn styled_raw_byte_is_signed_for_land_style_and_does_not_clear_other_state() {
    for (raw, expected_style) in [
        (0, 0.),
        (1, 1.),
        (3, 3.),
        (4, 4.),
        (127, 127.),
        (128, -128.),
        (255, -1.),
    ] {
        let mut lifecycle = state();
        let before = lifecycle;
        let packet = cleanup(lifecycle.landing_callback_action(raw));
        lifecycle.apply_landing_cleanup(packet, styled());
        let mut expected = before;
        expected.staged_land_style = expected_style;
        expected.properties.staged = 4;
        assert_eq!(lifecycle, expected, "raw style {raw}");
    }
}

#[test]
fn type_two_alone_writes_unsigned_dodge_subtype_on_styled_route() {
    for subtype in [0, 1, 127, 128, 255] {
        let mut lifecycle = state();
        let before = lifecycle;
        let mut packet = cleanup(lifecycle.landing_callback_action(2));
        packet.subtype = subtype; // supplied directional classification
        lifecycle.apply_landing_cleanup(packet, styled());
        let mut expected = before;
        expected.staged_land_style = 2.;
        expected.staged_dodge = f32::from(subtype);
        expected.properties.staged = 4;
        assert_eq!(lifecycle, expected);
    }
    for raw in [0, 1, 3, 4, 255] {
        let mut lifecycle = state();
        lifecycle.apply_landing_cleanup(
            LandingCleanup {
                style: raw,
                subtype: 255,
            },
            styled(),
        );
        assert_eq!(lifecycle.staged_dodge, 9., "non-type-two preserves dodge");
    }
}

#[test]
fn either_later_cleanup_gate_selects_clear_style_without_dodge_or_request_clear() {
    for (actor_flag_four, inventory_blocks_styled_landing) in
        [(true, false), (false, true), (true, true)]
    {
        for raw in [2, 255] {
            let mut lifecycle = state();
            let packet = cleanup(lifecycle.landing_callback_action(raw));
            // This field may change between route selection and the later gate.
            lifecycle.deferred_pending = true;
            let before = lifecycle;
            lifecycle.apply_landing_cleanup(
                packet,
                LandingCleanupGate {
                    actor_flag_four,
                    inventory_blocks_styled_landing,
                },
            );
            let mut expected = before;
            expected.staged_land_style = 0.;
            expected.properties.staged = 0;
            assert_eq!(lifecycle, expected);
        }
    }
}

#[test]
fn outer_landing_refill_request_is_separate_from_callback_eligibility_and_cleanup() {
    let mut lifecycle = state();
    lifecycle.reset.suppress_once = false;
    // Staged one rejects the inner callback while evaluated two still requests
    // additive projection. Outer landing must retain its refill request.
    lifecycle.properties.staged = 1;
    let effects = lifecycle
        .evaluate_reset(LifecycleInputs {
            air: AirSample::SuppliedRolePredicate(false),
            repair_enabled: true,
            animation_triggered_jump: true,
            shared_remaining: 0,
            effective_shared_maximum: 2,
            dt: 0.25,
        })
        .unwrap();
    assert_eq!(effects.decision, ResetDecision::Landing);
    assert!(effects.refill_budgets && effects.landing_callback);
    assert!(effects.remove_vertical_additive_motion);
    assert!(!effects.physics_reset_callback);
    assert!(!landing_callback_eligible(2, 1, false));
    assert_eq!(lifecycle.properties.staged, 1);
    assert_eq!(lifecycle.staged_land_style, 6.);
    assert_eq!(lifecycle.reset.tracked_seconds, 0.75);
    assert!(!lifecycle.reset.cached_air);
}

#[test]
fn legacy_lifecycle_defaults_only_the_new_dodge_channel() {
    let mut lifecycle = state();
    lifecycle.deferred_pending = true;
    let mut encoded = serde_json::to_value(lifecycle).unwrap();
    encoded.as_object_mut().unwrap().remove("staged_dodge");
    let restored: JumpLifecycle = serde_json::from_value(encoded).unwrap();
    let mut expected = lifecycle;
    expected.staged_dodge = 0.;
    assert_eq!(restored, expected);
    let roundtrip: JumpLifecycle =
        serde_json::from_value(serde_json::to_value(lifecycle).unwrap()).unwrap();
    assert_eq!(roundtrip, lifecycle);
}
