//! Convenience combo checks against the original LAB controller, not native
//! animation timing or measured Warframe behavior.
use glam::Vec3;
use movement_lab::{Input, MovementState, Player, Solid, Tuning, World, combo::Combo};

fn floor() -> World {
    World {
        terrain: movement_lab::world::WorldTerrain::Boxes, // Explicit box-only backend.
        solids: vec![Solid::new(
            Vec3::new(-500., -2., -500.),
            Vec3::new(500., 0., 500.),
        )],
    }
}

fn settled(world: &World, tuning: &Tuning) -> Player {
    let mut player = Player::new(Vec3::Y * 0.05);
    for _ in 0..120 {
        player.step(&Input::default(), 1. / 120., world, tuning);
    }
    assert!(player.grounded);
    player
}

#[test]
fn open_floor_combo_sprints_slides_bullets_doubles_and_glides_at_three_rates() {
    for hz in [60., 120., 240.] {
        let dt = 1. / hz;
        let world = floor();
        let tuning = Tuning::default();
        let mut player = settled(&world, &tuning);
        let mut combo = Combo::default();
        combo.start();
        let forward = Vec3::new(0., 0.4, -0.916_515_1);
        let mut saw_sprint = false;
        let mut saw_slide = false;
        let mut saw_glide = false;
        let mut jumps = 0;
        let mut bullet_time = 0.;
        let mut double_time = 0.;
        let mut pending_budget=None;
        for frame in 0..(5. * hz) as usize {
            let Some(input) = combo.input(dt, player.grounded, forward) else {
                break;
            };
            assert_eq!(input.aim_target, None);
            assert_eq!(input.camera_forward, forward);
            assert_eq!(input.movement, Vec3::NEG_Z);
            let before_budget = player.jumps_remaining;
            player.step(&input, dt, &world, &tuning);
            if let Some(expected)=pending_budget.take() {
                assert_eq!(player.jumps_remaining,expected);
                assert!(player.wall_traversal().ticket.is_none());
                assert!(player.wall_traversal().outcomes.iter().any(|o|o.kind==movement_lab::wall_traversal::OutcomeKind::InitialFallback && o.result==movement_lab::wall_traversal::ResultKind::Accepted));
            }
            saw_sprint |= player.state == MovementState::Sprint;
            saw_slide |= player.state == MovementState::Slide;
            if input.jump {
                jumps += 1;
                if jumps == 1 {
                    assert!(saw_sprint && saw_slide, "launch follows both ground stages");
                    assert!(input.crouch);
                    assert_eq!(player.state, MovementState::BulletJump);
                    assert_eq!(player.bullet_jumps_left, 0);
                    assert_eq!(player.jumps_remaining, before_budget - 1);
                    assert!(!player.grounded);
                    bullet_time = frame as f32 * dt;
                } else {
                    assert_eq!(jumps, 2, "no repeated jump pulses");
                    assert!(!input.crouch);
                    if hz==240. {
                        assert_eq!(player.jumps_remaining,before_budget);
                        assert_eq!(player.wall_traversal().ticket.as_ref().unwrap().kind,movement_lab::wall_traversal::Kind::Initial);
                        pending_budget=Some(before_budget-1);
                    } else {
                        assert_eq!(player.jumps_remaining, before_budget - 1);
                        assert_eq!(player.jumps_remaining, 0);
                    }
                    double_time = frame as f32 * dt;
                }
            }
            if input.aim {
                assert_eq!(jumps, 2);
                saw_glide |= player.state == MovementState::AimGlide;
            }
        }
        assert!(pending_budget.is_none());
        assert_eq!(player.jumps_remaining,0);
        assert!(!combo.is_active(), "sequence ends at {hz}Hz");
        assert_eq!(jumps, 2);
        assert!(saw_sprint && saw_slide && saw_glide, "stages at {hz}Hz");
        assert!((0.70..=0.70 + 3. * dt).contains(&bullet_time));
        assert!((0.30..=0.30 + 3. * dt).contains(&(double_time - bullet_time)));
    }
}

#[test]
fn cancel_emits_no_input_and_restart_restarts_sprint_without_jump() {
    let mut combo = Combo::default();
    assert!(!combo.is_active());
    assert_eq!(combo.stage_label(), "Ready");
    assert!(combo.input(0.01, true, Vec3::NEG_Z).is_none());
    combo.start();
    for _ in 0..80 {
        let _ = combo.input(1. / 120., true, Vec3::NEG_Z);
    }
    combo.cancel();
    for _ in 0..10 {
        assert!(combo.input(1. / 120., true, Vec3::NEG_Z).is_none());
    }
    combo.start();
    let input = combo.input(1. / 120., true, Vec3::NEG_Z).unwrap();
    assert!(input.sprint);
    assert!(!input.crouch && !input.jump && !input.aim);
    assert_eq!(combo.stage_label(), "Sprint");
}

#[test]
fn losing_support_before_launch_cancels_but_grounded_bullet_pulse_is_allowed() {
    let mut combo = Combo::default();
    combo.start();
    assert!(combo.input(0.01, false, Vec3::NEG_Z).is_none());
    assert!(!combo.is_active());
    combo.start();
    combo.input(0.55, true, Vec3::NEG_Z).unwrap();
    assert_eq!(combo.stage_label(), "Slide");
    assert!(combo.input(0.01, false, Vec3::NEG_Z).is_none());
    combo.start();
    combo.input(0.55, true, Vec3::NEG_Z).unwrap();
    combo.input(0.15, true, Vec3::NEG_Z).unwrap();
    assert_eq!(combo.stage_label(), "Bullet jump");
    let pulse = combo.input(1. / 120., true, Vec3::NEG_Z).unwrap();
    assert!(pulse.jump && pulse.crouch);
    assert!(combo.is_active());
    // A later grounded sample ends the sequence instead of manufacturing flight.
    assert!(combo.input(1. / 120., true, Vec3::NEG_Z).is_none());
    assert!(!combo.is_active());
}

#[test]
fn invalid_samples_preserve_progress_and_large_steps_cannot_repeat_or_merge_pulses() {
    let mut combo = Combo::default();
    combo.start();
    for dt in [0., -1., f32::NAN, f32::INFINITY] {
        assert!(combo.input(dt, true, Vec3::NEG_Z).is_none());
        assert_eq!(combo.stage_label(), "Sprint");
    }
    assert!(combo.input(0.1, true, Vec3::NAN).is_none());
    let mut jumps = 0;
    let mut launched = false;
    let mut calls = 0;
    while combo.is_active() {
        calls += 1;
        assert!(calls <= 7);
        let input = combo.input(f32::MAX, !launched, Vec3::NEG_Z).unwrap();
        if input.jump {
            jumps += 1;
            launched = true;
        }
    }
    assert_eq!(calls, 7);
    assert_eq!(jumps, 2);
}

#[test]
fn current_camera_drives_horizontal_movement_without_changing_bullet_pitch() {
    let mut combo = Combo::default();
    combo.start();
    let first = combo.input(0.1, true, Vec3::new(3., 4., 0.)).unwrap();
    assert_eq!(first.movement, Vec3::X);
    assert_eq!(first.camera_forward, Vec3::new(3., 4., 0.));
    let second = combo.input(0.1, true, Vec3::new(0., -4., 3.)).unwrap();
    assert_eq!(second.movement, Vec3::Z);
    assert_eq!(second.camera_forward, Vec3::new(0., -4., 3.));
    let vertical = combo.input(0.1, true, Vec3::Y).unwrap();
    assert_eq!(vertical.movement, Vec3::ZERO);
    assert!(vertical.movement.is_finite());
}
