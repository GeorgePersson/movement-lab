//! Public controller regressions for the explicitly selected LAB jump graph.
//! Graph ordinals and queue order do not establish native clip timing or fit.

use glam::Vec3;
use movement_lab::combo::Combo;
use movement_lab::jump_lifecycle::{JumpProperties, ResetDecision};
use movement_lab::{Input, Player, Solid, Tuning, World};

const DT: f32 = 1. / 240.;

fn fixture() -> (World, Tuning) {
    let world = World {
        terrain: movement_lab::world::WorldTerrain::Boxes, // Explicit box-only backend.
        solids: vec![Solid::new(
            Vec3::new(-500., -2., -500.),
            Vec3::new(500., 0., 500.),
        )],
    };
    let tuning: Tuning =
        serde_json::from_str(include_str!("../presets/static-tenno-bullet.json")).unwrap();
    assert!(tuning.is_valid());
    (world, tuning)
}

fn settled(world: &World, tuning: &Tuning) -> Player {
    let mut player = Player::new(Vec3::Y * 0.05);
    for _ in 0..120 {
        player.step(&Input::default(), DT, world, tuning);
    }
    assert!(player.grounded);
    player
}

fn graph(player: &Player, value: u8) {
    assert_eq!(
        player.jump_lifecycle().properties,
        JumpProperties {
            evaluated: value,
            staged: value,
        },
        "unbound LAB graph entry publishes both channels"
    );
}

fn reach_first_contact(player: &mut Player, world: &World, tuning: &Tuning) {
    assert!(!player.grounded);
    for _ in 0..4800 {
        player.step(&Input::default(), DT, world, tuning);
        if player.grounded {
            return;
        }
    }
    panic!("fixture never reached the floor");
}

#[test]
fn ordinary_graph_is_falling_while_the_once_completed_height_impulse_rises() {
    let (world, tuning) = fixture();
    let mut player = settled(&world, &tuning);
    player.step(
        &Input {
            jump: true,
            ..Default::default()
        },
        DT,
        &world,
        &tuning,
    );
    graph(&player, 3);
    let target = (2. * tuning.launch_gravity.unwrap_or(tuning.gravity) * tuning.jump_height).sqrt();
    assert!((player.velocity.y - target).abs() < 1e-4);
    assert!(player.velocity.y > 0. && !player.grounded);
    assert_eq!(player.jumps_remaining, tuning.max_jumps - 1);
    assert!(!player.jump_lifecycle().deferred_pending);
    assert_eq!(player.jump_lifecycle().deferred_height, -1.);
    assert!(player.jump_lifecycle().reset.suppress_once);

    // Input.jump is a supplied edge; holding unrelated movement must not replay it.
    let input = Input {
        movement: Vec3::NEG_Z,
        sprint: true,
        ..Default::default()
    };
    player.step(&input, DT, &world, &tuning);
    assert_eq!(player.last_jump_reset(), ResetDecision::Suppressed);
    assert!(!player.jump_lifecycle().reset.suppress_once);
    let mut prior_y = player.velocity.y;
    for _ in 0..30 {
        player.step(&input, DT, &world, &tuning);
        assert!(player.velocity.y < prior_y, "no queued impulse repeats");
        prior_y = player.velocity.y;
        assert_eq!(player.jumps_remaining, tuning.max_jumps - 1);
        assert!(!player.jump_lifecycle().deferred_pending);
    }
}

#[test]
fn ordinary_land_cleans_up_after_publication_and_can_launch_again() {
    let (world, tuning) = fixture();
    let mut player = settled(&world, &tuning);
    for _ in 0..2 {
        player.step(
            &Input {
                jump: true,
                ..Default::default()
            },
            DT,
            &world,
            &tuning,
        );
        graph(&player, 3);
        reach_first_contact(&mut player, &world, &tuning);
        assert_eq!(player.jumps_remaining, tuning.max_jumps - 1);
        assert_ne!(player.jump_lifecycle().properties.staged, 1);
        player.step(&Input::default(), DT, &world, &tuning);
        assert_eq!(player.last_jump_reset(), ResetDecision::Landing);
        graph(&player, 4);
        assert_eq!(player.jumps_remaining, tuning.max_jumps);
        assert_eq!(player.bullet_jumps_left, tuning.max_bullet_jumps);
        assert_eq!(player.air_rolls_left, tuning.max_air_rolls);
        assert!(!player.jump_lifecycle().deferred_pending);
    }
}

#[test]
fn bullet_double_and_aim_graph_changes_do_not_manufacture_height_requests() {
    let (world, mut tuning) = fixture();
    // Make an accidental ordinary impulse much stronger than the intended bullet.
    tuning.jump_height = 100.;
    let mut player = settled(&world, &tuning);
    let bullet = Input {
        camera_forward: Vec3::new(0., 0.5, -0.866_025_4),
        crouch: true,
        jump: true,
        ..Default::default()
    };
    player.step(&bullet, DT, &world, &tuning);
    graph(&player, 2);
    assert!((player.velocity.y - tuning.bullet_jump_speed * 0.5).abs() < 1e-4);
    assert_eq!(player.bullet_jumps_left, 0);
    assert_eq!(player.jumps_remaining, 1);
    assert!(!player.jump_lifecycle().deferred_pending);
    let held = Input {
        jump: false,
        ..bullet
    };
    for _ in 0..4 {
        player.step(&held, DT, &world, &tuning);
        graph(&player, 2);
        assert_eq!(player.jumps_remaining, 1);
    }
    player.step(
        &Input {
            jump: true,
            ..Default::default()
        },
        DT,
        &world,
        &tuning,
    );
    graph(&player, 2);
    assert_eq!(player.jumps_remaining,1);
    assert_eq!(player.wall_traversal().ticket.as_ref().unwrap().kind,movement_lab::wall_traversal::Kind::Initial);
    assert!(!player.jump_lifecycle().deferred_pending);
    player.step(&Input::default(),DT,&world,&tuning);
    assert!(player.wall_traversal().ticket.is_none());
    assert!(player.wall_traversal().outcomes.iter().any(|o|o.kind==movement_lab::wall_traversal::OutcomeKind::InitialFallback && o.result==movement_lab::wall_traversal::ResultKind::Accepted));
    graph(&player,2);
    assert_eq!(player.jumps_remaining, 0);
    assert_eq!(player.bullet_jumps_left, 0);
    assert!(!player.jump_lifecycle().deferred_pending);
    let double_target =
        (2. * tuning.launch_gravity.unwrap_or(tuning.gravity) * tuning.double_jump_height).sqrt();
    assert!((player.velocity.y - double_target).abs() < 1e-4);
    let before_aim = player.velocity.y;
    player.step(
        &Input {
            aim: true,
            ..Default::default()
        },
        DT,
        &world,
        &tuning,
    );
    graph(&player, 3);
    assert!(player.velocity.y <= before_aim * tuning.aim_entry_vertical_scale + 1e-4);
    assert_eq!(player.jumps_remaining, 0);
    assert_eq!(player.bullet_jumps_left, 0);
    assert!(!player.jump_lifecycle().deferred_pending);
    let mut prior_y = player.velocity.y;
    for _ in 0..8 {
        player.step(
            &Input {
                aim: true,
                ..Default::default()
            },
            DT,
            &world,
            &tuning,
        );
        graph(&player, 3);
        assert!(player.velocity.y <= prior_y + 1e-5);
        assert!(
            player.velocity.y > prior_y * 0.8,
            "held aim does not rescale every step"
        );
        prior_y = player.velocity.y;
        assert_eq!(player.jumps_remaining, 0);
    }
}

#[test]
fn downward_bullet_can_be_jumping_graph_state_while_gravity_still_applies() {
    let (world, mut tuning) = fixture();
    tuning.bullet_jump_gravity_scale = 1.;
    let mut player = Player::new(Vec3::Y * 10.);
    player.step(&Input::default(), DT, &world, &tuning);
    player.step(
        &Input {
            crouch: true,
            jump: true,
            camera_forward: Vec3::NEG_Y,
            ..Default::default()
        },
        DT,
        &world,
        &tuning,
    );
    graph(&player, 2);
    assert!(player.velocity.y < -tuning.bullet_jump_speed);
    // A real negative scalar write sets reset suppression, but does not get the
    // distinct positive-launch preparation gravity skip.
    assert!(player.jump_lifecycle().reset.suppress_once);
    assert_eq!(player.bullet_jumps_left, 0);
    assert_eq!(player.jumps_remaining, tuning.max_jumps - 1);
    assert!(!player.jump_lifecycle().deferred_pending);
}

#[test]
fn completed_combo_lands_refills_and_reactivates_for_a_second_sequence() {
    let (world, tuning) = fixture();
    let mut player = settled(&world, &tuning);
    let mut combo = Combo::default();
    let forward = Vec3::new(0., 0.4, -0.916_515_1);
    for _ in 0..2 {
        combo.start();
        let mut jumps = 0;
        let mut saw_glide = false;
        for _ in 0..1200 {
            let Some(input) = combo.input(DT, player.grounded, forward) else {
                break;
            };
            player.step(&input, DT, &world, &tuning);
            jumps += usize::from(input.jump);
            saw_glide |= input.aim;
            if !combo.is_active() {
                break;
            }
        }
        assert!(!combo.is_active());
        assert_eq!(jumps, 2, "reactivation must accept both edges each run");
        assert!(saw_glide);
        assert_eq!(player.bullet_jumps_left, 0);
        assert_eq!(player.jumps_remaining, 0);
        reach_first_contact(&mut player, &world, &tuning);
        assert_eq!(player.jumps_remaining, 0);
        player.step(&Input::default(), DT, &world, &tuning);
        assert_eq!(player.last_jump_reset(), ResetDecision::Landing);
        graph(&player, 4);
        assert_eq!(player.jumps_remaining, tuning.max_jumps);
        assert_eq!(player.bullet_jumps_left, tuning.max_bullet_jumps);
        assert_eq!(player.air_rolls_left, tuning.max_air_rolls);
    }
}
