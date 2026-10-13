//! Supplied finite preparation and integrated LAB-entry regressions.
use glam::Vec3;
use movement_lab::wall_entry::{WallEntryInput, WallEntryTuning, prepare_wall_entry};
use movement_lab::{Input, MovementState, Player, Solid, Tuning, World};

const DT: f32 = 1. / 240.;
fn signals(velocity: Vec3, movement: Vec3) -> WallEntryInput {
    WallEntryInput {
        base_velocity: velocity,
        movement,
        wall_normal: -Vec3::X,
        service_rejects: false,
        geometry_available: true,
        physics_available: true,
    }
}
fn accepted(velocity: Vec3, movement: Vec3) -> bool {
    prepare_wall_entry(signals(velocity, movement), WallEntryTuning::default())
        .unwrap()
        .direction
        .is_some()
}
fn wall() -> World {
    World {
        terrain: movement_lab::world::WorldTerrain::Boxes, // Explicit box-only backend.
        solids: vec![Solid::new(Vec3::new(2., 0., -10.), Vec3::new(3., 30., 10.))],
    }
}

#[test]
fn inward_velocity_admits_and_stationary_outward_or_parallel_without_input_reject() {
    assert!(accepted(Vec3::X * 4., Vec3::ZERO));
    assert!(!accepted(Vec3::ZERO, Vec3::ZERO));
    assert!(!accepted(-Vec3::X * 4., Vec3::ZERO));
    assert!(!accepted(Vec3::Z * 4., Vec3::ZERO));
    // The native robust fallback preserves a small nonzero direction.
    assert!(accepted(Vec3::X * 1e-30, Vec3::ZERO));
}

#[test]
fn lateral_requires_velocity_alignment_positive_input_projection_and_no_outward_input() {
    assert!(accepted(Vec3::Z * 4., Vec3::Z));
    assert!(!accepted(-Vec3::Z * 4., Vec3::Z));
    assert!(!accepted(Vec3::Z * 4., Vec3::Z - Vec3::X));
    let tuning = WallEntryTuning {
        parallel_angle_degrees: 180.,
        ..Default::default()
    };
    // Relaxing tangent alignment does not bypass the strict positive input dot.
    let perpendicular = prepare_wall_entry(signals(Vec3::X, Vec3::Z), tuning).unwrap();
    assert_eq!(perpendicular.direction, None);
    assert!(perpendicular.remove_bullet_gravity);
}

#[test]
fn side_threshold_equality_uses_head_on_and_head_on_half_is_strict() {
    assert!(!accepted(Vec3::Z, Vec3::Z * 0.3));
    assert!(accepted(
        Vec3::Z,
        Vec3::Z * f32::from_bits(0.3_f32.to_bits() + 1)
    ));
    // Length is exactly1: head-on dot=.5, so equality must reject.
    let boundary = Vec3::new(0.5, 0., (0.75_f32).sqrt());
    assert_eq!(boundary.length(), 1.);
    assert!(!accepted(boundary, Vec3::ZERO));
    assert!(accepted(
        Vec3::new(0.501, 0., (1. - 0.501_f32.powi(2)).sqrt()),
        Vec3::ZERO
    ));
}

#[test]
fn tangent_alignment_equality_is_accepted_without_replacing_other_gates() {
    let tuning = WallEntryTuning {
        parallel_angle_degrees: 0.,
        ..Default::default()
    };
    let out = prepare_wall_entry(signals(Vec3::Z, Vec3::Z), tuning).unwrap();
    assert_eq!(out.direction, Some(Vec3::Z));
    assert!(out.remove_bullet_gravity);
    assert!(
        prepare_wall_entry(signals(Vec3::new(0.01, 0., 1.), Vec3::Z), tuning)
            .unwrap()
            .direction
            .is_none()
    );
}

#[test]
fn early_guards_preserve_gravity_but_later_direction_failure_requests_removal() {
    let original = signals(Vec3::ZERO, Vec3::ZERO);
    for input in [
        WallEntryInput {
            service_rejects: true,
            ..original
        },
        WallEntryInput {
            geometry_available: false,
            ..original
        },
        WallEntryInput {
            physics_available: false,
            ..original
        },
    ] {
        let out = prepare_wall_entry(input, WallEntryTuning::default()).unwrap();
        assert_eq!(out.direction, None);
        assert!(!out.remove_bullet_gravity);
    }
    let disabled = prepare_wall_entry(
        original,
        WallEntryTuning {
            enabled: false,
            ..Default::default()
        },
    )
    .unwrap();
    assert!(!disabled.remove_bullet_gravity);
    let rejected = prepare_wall_entry(original, WallEntryTuning::default()).unwrap();
    assert!(rejected.remove_bullet_gravity);
    assert_eq!(rejected.direction, None);
}

#[test]
fn invalid_inputs_produce_no_effect_request_and_legacy_tuning_loads_defaults() {
    let input = signals(Vec3::X, Vec3::ZERO);
    for bad in [
        WallEntryInput {
            base_velocity: Vec3::NAN,
            ..input
        },
        WallEntryInput {
            movement: Vec3::INFINITY,
            ..input
        },
        WallEntryInput {
            wall_normal: Vec3::NAN,
            ..input
        },
    ] {
        assert_eq!(prepare_wall_entry(bad, WallEntryTuning::default()), None);
    }
    for angle in [f32::NAN, -1., 181.] {
        assert_eq!(
            prepare_wall_entry(
                input,
                WallEntryTuning {
                    parallel_angle_degrees: angle,
                    ..Default::default()
                }
            ),
            None
        );
    }
    let mut json = serde_json::to_value(Tuning::default()).unwrap();
    json.as_object_mut().unwrap().remove("wall_entry");
    let restored: Tuning = serde_json::from_value(json).unwrap();
    assert!(restored.wall_entry.enabled);
    assert_eq!(restored.wall_entry.parallel_angle_degrees, 60.);
    assert!(restored.is_valid());
}

#[test]
fn huge_finite_length_and_subnormal_reciprocal_overflow_reject_but_keep_early_effect() {
    for magnitude in [1e20, f32::MAX, f32::from_bits(1)] {
        let out = prepare_wall_entry(
            signals(Vec3::X * magnitude, Vec3::ZERO),
            WallEntryTuning::default(),
        )
        .unwrap();
        assert!(out.remove_bullet_gravity);
        assert_eq!(out.direction, None);
    }
    // A usable tiny value still takes the robust reciprocal-rescale path.
    let out = prepare_wall_entry(
        signals(Vec3::X * 1e-30, Vec3::ZERO),
        WallEntryTuning::default(),
    )
    .unwrap();
    assert!(out.remove_bullet_gravity);
    assert_eq!(out.direction, Some(Vec3::Y));
}

#[test]
fn integrated_new_entry_rejects_away_but_accepts_incoming_and_keeps_continuation() {
    let input = Input {
        aim: true,
        ..Default::default()
    };
    let t = Tuning::default();
    let w = wall();
    let mut rejected = Player::new(Vec3::new(1.699, 3., 0.));
    rejected.velocity.x = -4.;
    rejected.step(&input, DT, &w, &t);
    assert_eq!(rejected.state, MovementState::AimGlide);
    assert!(rejected.position.x < 1.699);
    let mut accepted = Player::new(Vec3::new(1.699, 3., 0.));
    accepted.velocity.x = 4.;
    let before = accepted.position;
    accepted.step(&input, DT, &w, &t);
    assert_eq!(accepted.state, MovementState::WallLatch);
    assert_eq!(accepted.position, before);
    assert_eq!(accepted.velocity, Vec3::ZERO);
    for _ in 0..60 {
        accepted.step(&input, DT, &w, &t);
        assert_eq!(accepted.state, MovementState::WallLatch);
        assert_eq!(accepted.position, before);
    }
    accepted.step(&Input::default(), DT, &w, &t);
    assert_ne!(accepted.state, MovementState::WallLatch);
}

#[test]
fn integrated_scalar_only_request_rejects_and_repeated_rejection_preserves_budgets() {
    let t = Tuning::default();
    let w = wall();
    let input = Input {
        aim: true,
        ..Default::default()
    };
    let mut p = Player::new(Vec3::new(1.699, 10., 0.));
    p.velocity.y = 20.;
    let budgets = (p.jumps_remaining, p.bullet_jumps_left, p.air_rolls_left);
    for _ in 0..10 {
        p.step(&input, DT, &w, &t);
        assert_eq!(p.state, MovementState::AimGlide);
        assert_eq!(
            (p.jumps_remaining, p.bullet_jumps_left, p.air_rolls_left),
            budgets
        );
    }
}
