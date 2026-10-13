use movement_lab::{
    jump_animation::{GraphInputs, JumpEvent, JumpNode, JumpStyle, LabJumpAnimation},
    jump_graph::{JumpBinding, JumpValue, WriteMode},
    jump_lifecycle::{JumpLifecycle, JumpProperties, JumpStartImpulseContext},
};

fn inputs(style: JumpStyle) -> GraphInputs {
    GraphInputs {
        style,
        active_clip: true,
        ..GraphInputs::default()
    }
}

#[test]
fn normal_route_changes_properties_before_separate_impulse_completion() {
    let mut lifecycle = JumpLifecycle::default();
    lifecycle.request_deferred_height(2.5).unwrap();
    let mut binding = JumpBinding::default();
    binding.promote(&mut lifecycle.properties).unwrap();
    let mut graph = LabJumpAnimation::default();
    let events = graph.evaluate(
        &mut lifecycle.properties,
        &mut binding,
        inputs(JumpStyle::Normal),
    );
    assert_eq!(
        graph.transition_trace(),
        &[
            JumpNode::Initial,
            JumpNode::Jumping,
            JumpNode::NormalJump,
            JumpNode::TriggerJump,
            JumpNode::Falling
        ]
    );
    assert_eq!(
        events,
        [JumpEvent::Start, JumpEvent::Impulse, JumpEvent::Wait]
    );
    assert_eq!(
        lifecycle.properties,
        JumpProperties {
            staged: 3,
            evaluated: 3
        }
    );
    assert!(lifecycle.deferred_pending);
    assert_eq!(lifecycle.deferred_height, 2.5);
    assert!(!lifecycle.reset.suppress_once);

    let context = JumpStartImpulseContext {
        own_sender: true,
        is_jump_start_impulse: true,
        animation_triggered_jump: true,
        physics_available: true,
    };
    let mut launches = 0;
    for event in events {
        if event == JumpEvent::Impulse {
            if let Some(eligible) = lifecycle.check_jump_start_impulse(context) {
                let attempt = lifecycle.begin_jump_start_impulse(eligible);
                assert_eq!(attempt.height(), 2.5);
                launches += 1;
                lifecycle.complete_jump_start_impulse(attempt, true);
            }
        }
    }
    assert_eq!(launches, 1);
    assert!(!lifecycle.deferred_pending);
    assert_eq!(lifecycle.deferred_height, -1.);
    assert_eq!(lifecycle.properties.staged, 3);
    assert!(lifecycle.reset.suppress_once);
    assert!(
        graph
            .evaluate(
                &mut lifecycle.properties,
                &mut binding,
                inputs(JumpStyle::Normal)
            )
            .is_empty()
    );
}

#[test]
fn bullet_holds_without_clock_or_velocity_and_aim_precedes_active() {
    let mut graph = LabJumpAnimation::default();
    let mut binding = JumpBinding::default();
    let mut p = JumpProperties {
        staged: 1,
        evaluated: 1,
    };
    let mut input = inputs(JumpStyle::Bullet);
    assert!(graph.evaluate(&mut p, &mut binding, input).is_empty());
    assert_eq!(graph.node(), JumpNode::Jumping);
    for _ in 0..100 {
        assert!(graph.evaluate(&mut p, &mut binding, input).is_empty());
        assert_eq!(graph.transition_trace(), &[JumpNode::Jumping]);
    }
    input.aiming = true;
    assert_eq!(
        graph.evaluate(&mut p, &mut binding, input),
        [JumpEvent::Wait]
    );
    assert_eq!(
        p,
        JumpProperties {
            staged: 3,
            evaluated: 3
        }
    );
    input.aiming = false;
    assert!(graph.evaluate(&mut p, &mut binding, input).is_empty());
    assert_eq!(graph.node(), JumpNode::Falling);
}

#[test]
fn double_and_retrigger_preserve_ordered_persistent_nodes() {
    let mut graph = LabJumpAnimation::default();
    let mut binding = JumpBinding::default();
    let mut p = JumpProperties {
        staged: 1,
        evaluated: 1,
    };
    let input = inputs(JumpStyle::Double);
    graph.evaluate(&mut p, &mut binding, input);
    assert_eq!(
        graph.transition_trace(),
        &[JumpNode::Initial, JumpNode::Jumping, JumpNode::JumpingB]
    );
    graph.restart_for_lab_request();
    binding.write(&mut p, JumpValue::Start, WriteMode::StageOnly);
    binding.promote(&mut p).unwrap();
    assert!(graph.evaluate(&mut p, &mut binding, input).is_empty());
    assert_eq!(
        graph.transition_trace(),
        &[JumpNode::JumpingB, JumpNode::Jumping, JumpNode::JumpingB]
    );
    assert_eq!(p.evaluated, 2);
}

#[test]
fn normal_style_priority_beats_aim_and_active() {
    let mut graph = LabJumpAnimation::default();
    let mut binding = JumpBinding::default();
    let mut p = JumpProperties {
        staged: 1,
        evaluated: 1,
    };
    let mut input = inputs(JumpStyle::Normal);
    input.aiming = true;
    assert_eq!(
        graph.evaluate(&mut p, &mut binding, input),
        [JumpEvent::Start, JumpEvent::Impulse, JumpEvent::Wait]
    );
}

#[test]
fn inactive_immediate_path_queues_impulse_without_inventing_pending_request() {
    let mut graph = LabJumpAnimation::default();
    let mut binding = JumpBinding::default();
    let mut lifecycle = JumpLifecycle::default();
    lifecycle.immediate_lab_launch(false, true);
    binding.promote(&mut lifecycle.properties).unwrap();
    let mut input = inputs(JumpStyle::Bullet);
    input.active_clip = false;
    let events = graph.evaluate(&mut lifecycle.properties, &mut binding, input);
    assert_eq!(events, [JumpEvent::Impulse, JumpEvent::Wait]);
    assert!(!lifecycle.deferred_pending);
    assert!(
        lifecycle
            .check_jump_start_impulse(JumpStartImpulseContext {
                own_sender: true,
                is_jump_start_impulse: true,
                animation_triggered_jump: true,
                physics_available: true,
            })
            .is_none()
    );
    assert!(lifecycle.reset.suppress_once);
}

#[test]
fn nonloop_active_blocks_landing_rule_until_supplied_false() {
    let mut graph = LabJumpAnimation::default();
    let mut binding = JumpBinding::default();
    let mut p = JumpProperties {
        staged: 1,
        evaluated: 1,
    };
    let mut input = inputs(JumpStyle::Bullet);
    graph.evaluate(&mut p, &mut binding, input);
    binding.write(&mut p, JumpValue::Landing, WriteMode::RespectBinding);
    assert!(graph.evaluate(&mut p, &mut binding, input).is_empty());
    assert_eq!(graph.node(), JumpNode::Jumping);
    input.active_clip = false;
    assert!(graph.evaluate(&mut p, &mut binding, input).is_empty());
    assert_eq!(graph.node(), JumpNode::Terminate);
    assert_eq!(p.staged, 4);
}

#[test]
fn falling_landings_keep_property_four_and_explicit_clip_hold() {
    for sliding in [false, true] {
        let mut graph = LabJumpAnimation::default();
        let mut binding = JumpBinding::default();
        let mut p = JumpProperties {
            staged: 3,
            evaluated: 3,
        };
        let mut input = inputs(JumpStyle::Bullet);
        input.sliding = sliding;
        assert_eq!(
            graph.evaluate(&mut p, &mut binding, input),
            [JumpEvent::Wait]
        );
        binding.write(&mut p, JumpValue::Landing, WriteMode::RespectBinding);
        assert!(graph.evaluate(&mut p, &mut binding, input).is_empty());
        assert_eq!(
            graph.transition_trace(),
            &[
                JumpNode::Falling,
                JumpNode::LandingCheck,
                if sliding {
                    JumpNode::SlideLanding
                } else {
                    JumpNode::Landing
                }
            ]
        );
        assert_eq!(p.staged, 4);
        input.active_clip = false;
        graph.evaluate(&mut p, &mut binding, input);
        assert_eq!(graph.node(), JumpNode::Terminate);
        assert_eq!(p.staged, 4);
    }
}

#[test]
fn leaving_landing_can_retrigger_without_termination_restart() {
    let mut graph = LabJumpAnimation::default();
    let mut binding = JumpBinding::default();
    let mut p = JumpProperties {
        staged: 3,
        evaluated: 3,
    };
    let input = inputs(JumpStyle::Bullet);
    graph.evaluate(&mut p, &mut binding, input);
    binding.write(&mut p, JumpValue::Landing, WriteMode::RespectBinding);
    graph.evaluate(&mut p, &mut binding, input);
    binding.write(&mut p, JumpValue::Start, WriteMode::RespectBinding);
    graph.evaluate(&mut p, &mut binding, input);
    assert_eq!(
        graph.transition_trace(),
        &[JumpNode::Landing, JumpNode::Initial, JumpNode::Jumping]
    );
    assert_eq!(p.staged, 2);
}

#[test]
fn termination_requires_explicit_lab_reactivation() {
    let mut graph = LabJumpAnimation::default();
    let mut binding = JumpBinding::default();
    let mut p = JumpProperties::default();
    graph.evaluate(&mut p, &mut binding, inputs(JumpStyle::Bullet));
    assert_eq!(graph.node(), JumpNode::Terminate);
    binding.write(&mut p, JumpValue::Start, WriteMode::RespectBinding);
    assert!(
        graph
            .evaluate(&mut p, &mut binding, inputs(JumpStyle::Bullet))
            .is_empty()
    );
    assert_eq!(graph.node(), JumpNode::Terminate);
    graph.restart_for_lab_request();
    graph.evaluate(&mut p, &mut binding, inputs(JumpStyle::Bullet));
    assert_eq!(graph.node(), JumpNode::Jumping);
}

#[test]
fn external_lock_retains_evaluated_value_and_bounds_transition_loop() {
    let mut graph = LabJumpAnimation::default();
    let mut binding = JumpBinding::default();
    let mut p = JumpProperties {
        staged: 1,
        evaluated: 1,
    };
    binding.acquire(&p).unwrap();
    let events = graph.evaluate(&mut p, &mut binding, inputs(JumpStyle::Normal));
    assert_eq!(p.evaluated, 1);
    assert_eq!(p.staged, 3);
    assert_eq!(binding.lock_count(), 1);
    assert_eq!(binding.captured_value(), Some(JumpValue::Start));
    assert_eq!(graph.transition_trace().len(), 11);
    assert!(graph.transition_limit_reached());
    assert_eq!(
        events,
        [
            JumpEvent::Start,
            JumpEvent::Impulse,
            JumpEvent::Wait,
            JumpEvent::Start,
            JumpEvent::Impulse,
            JumpEvent::Wait
        ]
    );
}

#[test]
fn evaluation_does_not_promote_and_unsupported_evaluated_value_is_inert() {
    let mut graph = LabJumpAnimation::default();
    let mut binding = JumpBinding::default();
    let mut p = JumpProperties {
        staged: 1,
        evaluated: 255,
    };
    assert!(
        graph
            .evaluate(&mut p, &mut binding, inputs(JumpStyle::Normal))
            .is_empty()
    );
    assert_eq!(
        p,
        JumpProperties {
            staged: 1,
            evaluated: 255
        }
    );
    assert_eq!(graph.node(), JumpNode::Initial);
    assert!(!graph.transition_limit_reached());
    p.evaluated = 0;
    graph.evaluate(&mut p, &mut binding, inputs(JumpStyle::Normal));
    assert_eq!(graph.node(), JumpNode::Terminate);
    assert_eq!(p.staged, 1);
}

#[test]
fn serde_retains_persistent_node_but_discards_transient_trace() {
    let mut graph = LabJumpAnimation::default();
    let mut binding = JumpBinding::default();
    let mut p = JumpProperties {
        staged: 1,
        evaluated: 1,
    };
    graph.evaluate(&mut p, &mut binding, inputs(JumpStyle::Double));
    let encoded = serde_json::to_string(&graph).unwrap();
    let loaded: LabJumpAnimation = serde_json::from_str(&encoded).unwrap();
    assert_eq!(loaded.node(), JumpNode::JumpingB);
    assert!(loaded.transition_trace().is_empty());
    let missing: LabJumpAnimation = serde_json::from_str("{}").unwrap();
    assert_eq!(missing.node(), JumpNode::Initial);
}
