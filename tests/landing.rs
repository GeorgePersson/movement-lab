//! Finite supplied-sample selector tests; no live RNG, asset or contact claims.

use glam::Vec3;
use movement_lab::landing::{LandingInputs, LandingTuning, select_landing_style};

fn inputs() -> LandingInputs {
    LandingInputs {
        actor_allowed: true,
        physics_available: true,
        cached_vertical: 7.,
        interrupted: false,
        flag4: false,
        movement: Vec3::ZERO,
        evaluated_jump: 1,
    }
}

#[test]
fn early_guards_and_interruption_skip_rng_at_distinct_selection_points() {
    let tuning = LandingTuning::default();
    for input in [
        LandingInputs {
            actor_allowed: false,
            movement: Vec3::X,
            ..inputs()
        },
        LandingInputs {
            physics_available: false,
            movement: Vec3::X,
            ..inputs()
        },
    ] {
        let mut rng = u64::MAX;
        assert_eq!(select_landing_style(input, &tuning, &mut rng).unwrap(), 0);
        assert_eq!(rng, u64::MAX);
    }
    let mut rng = u64::MAX;
    assert_eq!(
        select_landing_style(
            LandingInputs {
                interrupted: true,
                movement: Vec3::X,
                ..inputs()
            },
            &tuning,
            &mut rng
        )
        .unwrap(),
        3
    );
    assert_eq!(
        rng,
        u64::MAX,
        "interruption skips candidate but still selects moving style"
    );
}

#[test]
fn threshold_includes_equality_and_uses_absolute_cached_scalar() {
    let tuning = LandingTuning::default();
    let below = f32::from_bits(7f32.to_bits() - 1);
    for cached_vertical in [below, -below] {
        let mut rng = 0;
        assert_eq!(
            select_landing_style(
                LandingInputs {
                    cached_vertical,
                    ..inputs()
                },
                &tuning,
                &mut rng
            )
            .unwrap(),
            0
        );
        assert_eq!(rng, 0);
    }
    for cached_vertical in [7., -7., 8., -8.] {
        let mut rng = 0;
        assert_eq!(
            select_landing_style(
                LandingInputs {
                    cached_vertical,
                    ..inputs()
                },
                &tuning,
                &mut rng
            )
            .unwrap(),
            1
        );
        assert_eq!(rng, 0x1405_7b7e_f767_814f);
    }
}

#[test]
fn supplied_rng_wraps_and_extracts_the_declared_24_bits() {
    // Independent literal fixtures for one LCG step and its resulting f32.
    for (seed, next, random_bits) in [
        (0, 0x1405_7b7e_f767_814f, 0x3ea0_2bda),
        (u64::MAX, 0xbbb3_8751_aad2_0222, 0x3f6e_ce1d),
        (0x1234_5678_9abc_def0, 0xa1e0_95c2_dee3_c17f, 0x3f07_8257),
    ] {
        let random = f32::from_bits(random_bits);
        for (resist_chance, expected) in [
            (f32::from_bits(random_bits - 1), 1),
            (random, 0),
            (f32::from_bits(random_bits + 1), 0),
        ] {
            let tuning = LandingTuning {
                resist_chance,
                ..Default::default()
            };
            let mut rng = seed;
            assert_eq!(
                select_landing_style(inputs(), &tuning, &mut rng).unwrap(),
                expected
            );
            assert_eq!(rng, next, "probability rejection still consumes RNG");
        }
    }
}

#[test]
fn candidate_with_clear_flag_precedes_movement_and_fresh_graph_selection() {
    let tuning = LandingTuning::default();
    for evaluated_jump in 0..=u8::MAX {
        for movement in [Vec3::ZERO, Vec3::X, -Vec3::Y] {
            let mut rng = 0;
            assert_eq!(
                select_landing_style(
                    LandingInputs {
                        evaluated_jump,
                        movement,
                        ..inputs()
                    },
                    &tuning,
                    &mut rng
                )
                .unwrap(),
                1
            );
        }
    }
}

#[test]
fn flag_four_uses_candidate_and_fresh_graph_instead_of_universal_heavy_style() {
    let tuning = LandingTuning::default();
    for evaluated_jump in 0..=u8::MAX {
        for moving in [false, true] {
            let mut rng = 0;
            let expected = match (matches!(evaluated_jump, 1 | 2), moving) {
                (true, false) => 1,
                (true, true) => 3,
                (false, false) => 0,
                (false, true) => 4,
            };
            assert_eq!(
                select_landing_style(
                    LandingInputs {
                        evaluated_jump,
                        flag4: true,
                        movement: if moving { Vec3::Z } else { Vec3::ZERO },
                        ..inputs()
                    },
                    &tuning,
                    &mut rng
                )
                .unwrap(),
                expected
            );
            assert_eq!(rng, 0x1405_7b7e_f767_814f);
        }
    }
}

#[test]
fn absent_candidate_selects_only_zero_three_or_four_for_all_properties() {
    let tuning = LandingTuning {
        resist_chance: 1.,
        ..Default::default()
    };
    for evaluated_jump in 0..=u8::MAX {
        for moving in [false, true] {
            let mut rng = 0;
            let style = select_landing_style(
                LandingInputs {
                    evaluated_jump,
                    movement: if moving { Vec3::X } else { Vec3::ZERO },
                    ..inputs()
                },
                &tuning,
                &mut rng,
            )
            .unwrap();
            assert_eq!(
                style,
                if !moving {
                    0
                } else if matches!(evaluated_jump, 1 | 2) {
                    3
                } else {
                    4
                }
            );
            assert!(
                matches!(style, 0 | 3 | 4),
                "selector never directly emits type two"
            );
            assert_eq!(rng, 0x1405_7b7e_f767_814f);
        }
    }
}

#[test]
fn movement_uses_strict_xyz_epsilon_including_vertical_and_negative_input() {
    let tuning = LandingTuning::default();
    let above = f32::from_bits(1e-6f32.to_bits() + 1);
    for evaluated_jump in [1, 2, 0, 3, 4, 255] {
        for axis in [Vec3::X, Vec3::Y, Vec3::Z] {
            for sign in [-1., 1.] {
                for (amount, moving) in [(0., false), (1e-6, false), (above, true)] {
                    let mut rng = 0x1234;
                    let style = select_landing_style(
                        LandingInputs {
                            interrupted: true,
                            evaluated_jump,
                            movement: axis * amount * sign,
                            ..inputs()
                        },
                        &tuning,
                        &mut rng,
                    )
                    .unwrap();
                    assert_eq!(
                        style,
                        if !moving {
                            0
                        } else if matches!(evaluated_jump, 1 | 2) {
                            3
                        } else {
                            4
                        }
                    );
                    assert_eq!(rng, 0x1234);
                }
            }
        }
    }
}

#[test]
fn invalid_supplied_values_are_rejected_without_advancing_rng() {
    for tuning in [
        LandingTuning {
            heavy_threshold: -1.,
            ..Default::default()
        },
        LandingTuning {
            heavy_threshold: f32::NAN,
            ..Default::default()
        },
        LandingTuning {
            heavy_threshold: f32::INFINITY,
            ..Default::default()
        },
        LandingTuning {
            resist_chance: -0.01,
            ..Default::default()
        },
        LandingTuning {
            resist_chance: 1.01,
            ..Default::default()
        },
        LandingTuning {
            resist_chance: f32::NAN,
            ..Default::default()
        },
        LandingTuning {
            resist_chance: f32::INFINITY,
            ..Default::default()
        },
    ] {
        let mut rng = 0x1234;
        assert!(!tuning.is_valid());
        assert!(select_landing_style(inputs(), &tuning, &mut rng).is_err());
        assert_eq!(rng, 0x1234);
    }
    for invalid in [f32::NAN, f32::INFINITY, f32::NEG_INFINITY] {
        for input in [
            LandingInputs {
                cached_vertical: invalid,
                ..inputs()
            },
            LandingInputs {
                movement: Vec3::new(invalid, 0., 0.),
                ..inputs()
            },
            LandingInputs {
                movement: Vec3::new(0., invalid, 0.),
                ..inputs()
            },
            LandingInputs {
                movement: Vec3::new(0., 0., invalid),
                ..inputs()
            },
        ] {
            let mut rng = 0x1234;
            assert!(select_landing_style(input, &LandingTuning::default(), &mut rng).is_err());
            assert_eq!(rng, 0x1234);
        }
    }
}

#[test]
fn legacy_tuning_defaults_are_explicit_and_seed_does_not_reset_supplied_rng() {
    let tuning: LandingTuning = serde_json::from_str("{}").unwrap();
    assert_eq!(tuning, LandingTuning::default());
    assert_eq!(tuning.heavy_threshold, 7.);
    assert_eq!(tuning.resist_chance, 0.);
    assert_eq!(tuning.rng_seed, 0);
    assert!(tuning.is_valid());
    let partial: LandingTuning = serde_json::from_str("{\"heavy_threshold\":4}").unwrap();
    assert_eq!(partial.heavy_threshold, 4.);
    assert_eq!(partial.resist_chance, 0.);
    let different_seed = LandingTuning {
        rng_seed: u64::MAX,
        ..tuning
    };
    let mut a = 0x1234_5678_9abc_def0;
    let mut b = a;
    assert_eq!(
        select_landing_style(inputs(), &tuning, &mut a),
        select_landing_style(inputs(), &different_seed, &mut b)
    );
    assert_eq!(a, b);
    assert_eq!(a, 0xa1e0_95c2_dee3_c17f);
}
