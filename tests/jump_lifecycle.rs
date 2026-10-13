//! Multi-phase regression cases for the recovered reset contract. These are
//! original simulations, not executions of native code or gameplay measurements.
use movement_lab::jump_lifecycle::{
    InvalidTime, JumpProperties, ResetDecision as D, ResetInputs, ResetState,
};

fn idle() -> ResetInputs {
    ResetInputs {
        current_air: false,
        repair_enabled: true,
        animation_triggered_jump: true,
        deferred_pending: false,
        properties: JumpProperties::default(),
        shared_remaining: 1,
        effective_shared_maximum: 2,
        dt: 0.25,
    }
}

#[test]
fn suppressed_landing_loses_edge_without_refilling_partial_budget() {
    let mut state = ResetState {
        cached_air: true,
        suppress_once: true,
        tracked_seconds: 0.,
    };
    assert_eq!(state.evaluate(idle()), Ok(D::Suppressed));
    assert!(
        state.cached_air,
        "reset evaluation must not commit air early"
    );
    state.commit_air_sample(false);
    assert_eq!(state.evaluate(idle()), Ok(D::None));
    assert!(!D::Suppressed.refills_budgets());

    let mut empty = idle();
    empty.shared_remaining = 0;
    assert_eq!(state.evaluate(empty), Ok(D::StationaryRepair));
    assert!(D::StationaryRepair.refills_budgets());
}

#[test]
fn pending_property_blocks_idle_repair_before_graph_promotion() {
    let mut state = ResetState::default();
    let mut request = idle();
    request.shared_remaining = 0;
    request.deferred_pending = true;
    request.properties.staged = 1;
    assert_eq!(state.evaluate(request), Ok(D::None));
    assert_eq!(state.tracked_seconds, 0.25);

    // Simulate explicit promotion, then stage a reset without promotion.
    request.properties = JumpProperties {
        evaluated: 1,
        staged: 0,
    };
    assert_eq!(state.evaluate(request), Ok(D::None));
    request.properties.evaluated = 0;
    assert_eq!(state.evaluate(request), Ok(D::ExplicitDeferredReset));
    assert_eq!(state.tracked_seconds, 0.);
}

#[test]
fn tracked_request_waits_until_two_seconds_and_resets_tracking() {
    let mut state = ResetState::default();
    let mut input = idle();
    input.deferred_pending = true;
    input.properties.staged = 2;
    input.dt = 0.5;
    for _ in 0..3 {
        assert_eq!(state.evaluate(input), Ok(D::None));
        state.commit_air_sample(false);
    }
    assert_eq!(state.evaluate(input), Ok(D::StationaryRepair));
    assert_eq!(state.tracked_seconds, 0.);
    // A pending staged reset alone cannot erase an evaluated tracked value.
    input.properties = JumpProperties {
        evaluated: 2,
        staged: 4,
    };
    assert_eq!(state.evaluate(input), Ok(D::None));
}

#[test]
fn real_landing_bypasses_role_repair_and_zero_shared_maximum() {
    let mut state = ResetState::default();
    let mut input = idle();
    input.current_air = true;
    input.repair_enabled = false;
    input.effective_shared_maximum = 0;
    assert_eq!(state.evaluate(input), Ok(D::None));
    state.commit_air_sample(true);
    input.current_air = false;
    assert_eq!(state.evaluate(input), Ok(D::Landing));
    assert!(D::Landing.refills_budgets());
    state.commit_air_sample(false);
    assert_eq!(state.evaluate(input), Ok(D::None));
}

#[test]
fn evaluated_dodge_property_repairs_but_staging_it_is_not_immediate() {
    let mut state = ResetState::default();
    let mut input = idle();
    input.properties.staged = 3;
    assert_eq!(state.evaluate(input), Ok(D::None));
    input.properties.evaluated = 3;
    input.repair_enabled = false;
    assert_eq!(state.evaluate(input), Ok(D::None));
    input.repair_enabled = true;
    assert_eq!(state.evaluate(input), Ok(D::StationaryRepair));
}

#[test]
fn invalid_time_is_rejected_without_consuming_suppression() {
    let initial = ResetState {
        cached_air: true,
        suppress_once: true,
        tracked_seconds: 0.5,
    };
    for dt in [f32::NAN, f32::INFINITY, -0.25] {
        let mut state = initial;
        let mut input = idle();
        input.dt = dt;
        assert_eq!(state.evaluate(input), Err(InvalidTime));
        assert_eq!(state, initial);
    }
}
