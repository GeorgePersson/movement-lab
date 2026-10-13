//! Independent graybox-controller checks. These test the laboratory's declared
//! collision and transition policies, not fidelity to measured game behavior.

use glam::Vec3;
use movement_lab::{Input, Player, Solid, Tuning, World};

fn floor() -> Solid {
    Solid::new(Vec3::new(-50.0, -2.0, -50.0), Vec3::new(50.0, 0.0, 50.0))
}

fn grounded_player(world: &World, tuning: &Tuning) -> Player {
    let mut player = Player::new(Vec3::new(0.0, 0.05, 0.0));
    for _ in 0..120 {
        player.step(&Input::default(), 1.0 / 120.0, world, tuning);
    }
    assert!(player.grounded, "fixture must settle on the floor");
    player
}

#[test]
fn nearby_aim_target_changes_bullet_launch_without_spending_an_extra_jump() {
    let tuning = Tuning::default();
    let world = World {
        terrain: movement_lab::world::WorldTerrain::Boxes, // Explicit box-only backend.
        solids: vec![floor()],
    };
    let mut plain = grounded_player(&world, &tuning);
    let mut targeted = grounded_player(&world, &tuning);
    let input = Input {
        camera_forward: Vec3::new(0., 0.5, -0.8660254),
        crouch: true,
        jump: true,
        ..Default::default()
    };
    plain.step(&input, 1. / 120., &world, &tuning);
    targeted.step(
        &Input {
            aim_target: Some(Vec3::new(0., 2., -4.)),
            ..input
        },
        1. / 120.,
        &world,
        &tuning,
    );
    assert!(targeted.velocity.y > plain.velocity.y);
    assert!(targeted.velocity.z.abs() < plain.velocity.z.abs());
    assert_eq!(targeted.jumps_remaining, plain.jumps_remaining);
    assert_eq!(targeted.bullet_jumps_left, 0);
}

#[test]
fn thin_wall_blocks_a_fast_move_and_long_frame() {
    let tuning = Tuning::default();
    let world = World {
        terrain: movement_lab::world::WorldTerrain::Boxes, // Explicit box-only backend.
        solids: vec![
            floor(),
            Solid::new(Vec3::new(1.0, 0.0, -10.0), Vec3::new(1.04, 5.0, 10.0)),
        ],
    };
    let mut player = grounded_player(&world, &tuning);
    player.velocity = Vec3::X * 120.0;
    player.step(
        &Input {
            movement: Vec3::X,
            ..Default::default()
        },
        0.25,
        &world,
        &tuning,
    );
    assert!(player.position.x <= 1.0 - tuning.radius + 0.002);
    assert!(world.can_occupy(player.position, tuning.radius, player.height(&tuning)));
    assert!(player.position.is_finite() && player.velocity.is_finite());
}

#[test]
fn oblique_wall_contact_preserves_tangential_travel() {
    let tuning = Tuning::default();
    let world = World {
        terrain: movement_lab::world::WorldTerrain::Boxes, // Explicit box-only backend.
        solids: vec![
            floor(),
            Solid::new(Vec3::new(1.0, 0.0, -10.0), Vec3::new(1.1, 5.0, 10.0)),
        ],
    };
    let mut player = grounded_player(&world, &tuning);
    let input = Input {
        movement: Vec3::new(1.0, 0.0, 1.0).normalize(),
        ..Default::default()
    };
    for _ in 0..120 {
        player.step(&input, 1.0 / 120.0, &world, &tuning);
        assert!(world.can_occupy(player.position, tuning.radius, player.height(&tuning)));
    }
    assert!(player.position.x <= 1.0 - tuning.radius + 0.002);
    assert!(
        player.position.z > 0.5,
        "a wall must not freeze its tangent axis: {:?} {:?}",
        player.position,
        player.velocity
    );
}

#[test]
fn corner_contact_does_not_penetrate_either_wall() {
    let tuning = Tuning::default();
    let world = World {
        terrain: movement_lab::world::WorldTerrain::Boxes, // Explicit box-only backend.
        solids: vec![
            floor(),
            Solid::new(Vec3::new(1.0, 0.0, -10.0), Vec3::new(1.1, 5.0, 10.0)),
            Solid::new(Vec3::new(-10.0, 0.0, 1.0), Vec3::new(10.0, 5.0, 1.1)),
        ],
    };
    let mut player = grounded_player(&world, &tuning);
    let input = Input {
        movement: Vec3::new(1.0, 0.0, 1.0).normalize(),
        ..Default::default()
    };
    for _ in 0..240 {
        player.step(&input, 1.0 / 120.0, &world, &tuning);
        assert!(player.position.is_finite() && player.velocity.is_finite());
        assert!(world.can_occupy(player.position, tuning.radius, player.height(&tuning)));
    }
    assert!(player.position.x <= 1.0 - tuning.radius + 0.002);
    assert!(player.position.z <= 1.0 - tuning.radius + 0.002);
}

#[test]
fn one_jump_edge_is_not_replayed_by_internal_substeps() {
    let tuning = Tuning::default();
    let world = World {
        terrain: movement_lab::world::WorldTerrain::Boxes, // Explicit box-only backend.
        solids: vec![floor()],
    };
    let mut player = grounded_player(&world, &tuning);
    let initial_air_jumps = player.air_jumps_left;
    let input = Input {
        jump: true,
        ..Default::default()
    };
    player.step(&input, 0.1, &world, &tuning);
    assert!(!player.grounded && player.velocity.y > 0.0);
    assert_eq!(
        player.air_jumps_left, initial_air_jumps,
        "substeps must not repeat the jump edge"
    );
    for _ in 0..1200 {
        player.step(&Input::default(), 1.0 / 120.0, &world, &tuning);
    }
    assert!(player.grounded, "a single jump edge must allow landing");
    let landed = player.position;
    for _ in 0..120 {
        player.step(&Input::default(), 1.0 / 120.0, &world, &tuning);
        assert!(player.grounded);
        assert!((player.position.y - landed.y).abs() < 0.002);
    }
}

#[test]
fn an_air_jump_is_consumed_once_and_landing_restores_it() {
    let tuning = Tuning::default();
    let world = World {
        terrain: movement_lab::world::WorldTerrain::Boxes, // Explicit box-only backend.
        solids: vec![floor()],
    };
    let mut player = grounded_player(&world, &tuning);
    let initial_allowance = player.air_jumps_left;
    let pressed = Input {
        jump: true,
        ..Default::default()
    };
    player.step(&pressed, 1.0 / 120.0, &world, &tuning);
    player.step(&Input::default(), 1.0 / 120.0, &world, &tuning);
    player.step(&pressed, 1.0 / 120.0, &world, &tuning);
    assert!(player.air_jumps_left < initial_allowance);
    let consumed = player.air_jumps_left;
    for _ in 0..12 {
        player.step(&pressed, 1.0 / 120.0, &world, &tuning);
        assert_eq!(player.air_jumps_left, consumed);
    }
    for _ in 0..1200 {
        player.step(&Input::default(), 1.0 / 120.0, &world, &tuning);
    }
    assert!(player.grounded);
    assert_eq!(player.air_jumps_left, initial_allowance);
}

#[test]
fn equal_elapsed_time_at_common_frame_rates_stays_close() {
    let tuning = Tuning::default();
    let world = World {
        terrain: movement_lab::world::WorldTerrain::Boxes, // Explicit box-only backend.
        solids: vec![floor()],
    };
    let input = Input {
        movement: Vec3::X,
        ..Default::default()
    };
    let simulate = |hz: u32| {
        let mut player = grounded_player(&world, &tuning);
        for _ in 0..hz * 2 {
            player.step(&input, 1.0 / hz as f32, &world, &tuning);
        }
        player
    };
    let reference = simulate(120);
    for hz in [30, 60] {
        let player = simulate(hz);
        assert!(
            player.position.distance(reference.position) < 0.02,
            "position drift at {hz}Hz"
        );
        assert!(
            player.velocity.distance(reference.velocity) < 0.02,
            "velocity drift at {hz}Hz"
        );
        assert_eq!(player.grounded, reference.grounded);
    }
}

#[test]
fn releasing_crouch_under_a_low_ceiling_preserves_a_fitting_hull() {
    let tuning = Tuning::default();
    let ceiling = (tuning.crouch_height + tuning.standing_height) * 0.5;
    let world = World {
        terrain: movement_lab::world::WorldTerrain::Boxes, // Explicit box-only backend.
        solids: vec![
            floor(),
            Solid::new(
                Vec3::new(-2.0, ceiling, -2.0),
                Vec3::new(2.0, ceiling + 0.2, 2.0),
            ),
        ],
    };
    let mut player = Player::new(Vec3::new(0.0, 0.01, 0.0));
    player.crouched = true;
    let crouch = Input {
        crouch: true,
        ..Default::default()
    };
    for _ in 0..120 {
        player.step(&crouch, 1.0 / 120.0, &world, &tuning);
    }
    for _ in 0..120 {
        player.step(&Input::default(), 1.0 / 120.0, &world, &tuning);
        assert!(player.crouched, "standing under the roof would overlap it");
        assert!(world.can_occupy(player.position, tuning.radius, player.height(&tuning)));
    }
}

#[test]
fn released_crouch_jump_uses_retained_posture_and_respects_bullet_budget() {
    let tuning = Tuning::default();
    let ceiling = (tuning.crouch_height + tuning.standing_height) * 0.5;
    let covered = World {
        terrain: movement_lab::world::WorldTerrain::Boxes, // Explicit box-only backend.
        solids: vec![
            floor(),
            Solid::new(
                Vec3::new(-2.0, ceiling, -2.0),
                Vec3::new(2.0, ceiling + 0.2, 2.0),
            ),
        ],
    };
    let clear = World {
        terrain: movement_lab::world::WorldTerrain::Boxes, // Explicit box-only backend.
        solids: vec![floor()],
    };
    for (world, bullet_budget, expected_crouched, expected_bullet) in [
        (&covered, tuning.max_bullet_jumps, true, true),
        (&clear, tuning.max_bullet_jumps, false, false),
        (&covered, 0, true, false),
    ] {
        let mut player = Player::new(Vec3::new(0.0, 0.01, 0.0));
        player.crouched = true;
        for _ in 0..120 {
            player.step(
                &Input {
                    crouch: true,
                    ..Default::default()
                },
                1.0 / 120.0,
                world,
                &tuning,
            );
        }
        assert!(player.grounded);
        player.step(&Input::default(), 1.0 / 240.0, world, &tuning);
        assert_eq!(player.crouched, expected_crouched);
        player.bullet_jumps_left = bullet_budget;
        let jumps_before = player.jumps_remaining;
        player.step(
            &Input {
                jump: true,
                camera_forward: Vec3::X,
                ..Default::default()
            },
            1.0 / 240.0,
            world,
            &tuning,
        );
        assert_eq!(player.crouched, expected_crouched);
        assert_eq!(player.jumps_remaining, jumps_before - 1);
        assert_eq!(
            player.bullet_jumps_left,
            bullet_budget - u8::from(expected_bullet),
            "retained posture selects a bullet only when its budget permits"
        );
        if expected_bullet {
            assert!(player.velocity.x > tuning.bullet_jump_speed * 0.9);
        } else {
            assert!(player.velocity.x.abs() < 0.001);
        }
        assert!(!player.grounded);
        assert!(player.velocity.is_finite());
        assert!(world.can_occupy(player.position, tuning.radius, player.height(&tuning)));
    }
}
