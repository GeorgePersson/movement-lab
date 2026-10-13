//! Product regressions for the viewer's advertised one-button sequence.
//! These exercise the static preset and real Combo/Player/world-query APIs.
//! Camera presentation is sampled once at each simulation boundary (alpha=1),
//! not a claim about Bevy render scheduling or measured game trajectories.
use glam::Vec3;
use movement_lab::combo::Combo;
use movement_lab::jump_lifecycle::ResetDecision;
use movement_lab::{Input, MovementState, Player, Solid, Tuning, World};

const SPAWN: Vec3 = Vec3::new(0.0, 0.05, 8.0);
const PITCH: f32 = 0.32;

fn tuning() -> Tuning {
    serde_json::from_str(include_str!("../presets/static-tenno-bullet.json")).unwrap()
}
fn forward() -> Vec3 {
    Vec3::new(0.0, PITCH.sin(), -PITCH.cos())
}

// Same supplied LAB camera geometry as main::presentation, with current feet
// instead of a render-time interpolation. The query itself calls World directly.
fn camera_origin(player: &Player, world: &World, tuning: &Tuning) -> Vec3 {
    let target = player.position + Vec3::Y * (player.height(tuning) * 0.7);
    let desired = target - forward() * 5.5;
    let delta = desired - target;
    let mut fraction: f32 = 1.0;
    for solid in &world.solids {
        let min = solid.min - Vec3::splat(0.15);
        let max = solid.max + Vec3::splat(0.15);
        let mut enter: f32 = 0.0;
        let mut leave: f32 = 1.0;
        let mut hit = true;
        for axis in 0..3 {
            if delta[axis].abs() < 1e-6 {
                if target[axis] < min[axis] || target[axis] > max[axis] {
                    hit = false;
                    break;
                }
            } else {
                let a = (min[axis] - target[axis]) / delta[axis];
                let b = (max[axis] - target[axis]) / delta[axis];
                enter = enter.max(a.min(b));
                leave = leave.min(a.max(b));
                if enter > leave {
                    hit = false;
                    break;
                }
            }
        }
        if hit && enter > 0.0 && enter <= 1.0 {
            fraction = fraction.min((enter - 0.025).max(0.08));
        }
    }
    target + delta * fraction
}
fn input_for(
    combo: &mut Combo,
    player: &Player,
    world: &World,
    tuning: &Tuning,
    dt: f32,
) -> Option<Input> {
    let mut input = combo.input(dt, player.grounded, forward())?;
    input.aim_target = world.aim_target(
        camera_origin(player, world, tuning),
        input.camera_forward,
        5000.0,
    );
    Some(input)
}
fn valid_pose(player: &Player, world: &World, tuning: &Tuning) {
    assert!(player.position.is_finite() && player.combined_velocity().is_finite());
    assert!(
        world.can_occupy(player.position, tuning.radius, player.height(tuning)),
        "penetrating pose {:?}",
        player.position
    );
}
fn settle(world: &World, tuning: &Tuning, dt: f32) -> Player {
    let mut p = Player::new(SPAWN);
    for _ in 0..(2.0 / dt) as usize {
        p.step(&Input::default(), dt, world, tuning);
        valid_pose(&p, world, tuning);
    }
    assert!(p.grounded, "spawn did not settle");
    p
}
#[derive(Debug)]
struct Metrics {
    bullet_time: f32,
    double_time: f32,
    glide_time: f32,
    apex: f32,
    end: Vec3,
    targeted_launch: bool,
}
fn sequence(player: &mut Player, world: &World, tuning: &Tuning, hz: u32) -> Metrics {
    let dt = 1.0 / hz as f32;
    let mut combo = Combo::default();
    combo.start();
    let mut saw_sprint = false;
    let mut saw_slide = false;
    let mut saw_glide = false;
    let mut jumps = 0;
    let mut pending_budget=None;
    let mut result = Metrics {
        bullet_time: 0.0,
        double_time: 0.0,
        glide_time: 0.0,
        apex: player.position.y,
        end: player.position,
        targeted_launch: false,
    };
    for frame in 0..hz * 8 {
        let Some(input) = input_for(&mut combo, player, world, tuning, dt) else {
            break;
        };
        let before = player.jumps_remaining;
        player.step(&input, dt, world, tuning);
        valid_pose(player, world, tuning);
        if let Some(expected)=pending_budget.take() {
            assert_eq!(player.jumps_remaining,expected);
            assert!(player.wall_traversal().ticket.is_none());
            assert!(player.wall_traversal().outcomes.iter().any(|o|o.kind==movement_lab::wall_traversal::OutcomeKind::InitialFallback && o.result==movement_lab::wall_traversal::ResultKind::Accepted));
        }
        if input.jump {
            jumps += 1;
            if hz==240 && jumps==2 {
                assert_eq!(player.jumps_remaining,before);
                assert_eq!(player.wall_traversal().ticket.as_ref().unwrap().kind,movement_lab::wall_traversal::Kind::Initial);
                pending_budget=Some(before-1);
            } else {
                assert_eq!(player.jumps_remaining,before-1,"one allowance per admitted press at {hz}Hz");
            }
            if jumps == 1 {
                assert!(
                    saw_sprint && saw_slide,
                    "bullet must follow sprint and slide"
                );
                assert!(input.crouch);
                assert_eq!(player.state, MovementState::BulletJump);
                result.bullet_time = frame as f32 * dt;
                result.targeted_launch = input.aim_target.is_some();
            } else {
                assert_eq!(jumps, 2, "only two jump edges");
                assert!(!input.crouch);
                result.double_time = frame as f32 * dt;
            }
        }
        saw_sprint |= player.state == MovementState::Sprint;
        saw_slide |= player.state == MovementState::Slide;
        if input.aim {
            assert_eq!(jumps, 2, "glide follows both launches");
            if !saw_glide {
                result.glide_time = frame as f32 * dt;
            }
            saw_glide |= player.state == MovementState::AimGlide;
        }
        if jumps > 0 {
            assert!(
                !player.grounded,
                "sequence landed/cancelled before finishing at {hz}Hz, frame{frame}"
            );
            assert_eq!(
                player.jumps_remaining,
                tuning.max_jumps - jumps + u8::from(pending_budget.is_some()),
                "premature allowance restoration at {hz}Hz"
            );
            assert_eq!(player.bullet_jumps_left, tuning.max_bullet_jumps - 1);
            assert!(
                !matches!(
                    player.last_jump_reset(),
                    ResetDecision::Landing
                        | ResetDecision::StationaryRepair
                        | ResetDecision::ExplicitDeferredReset
                ),
                "premature reset at {hz}Hz"
            );
        }
        result.apex = result.apex.max(player.position.y);
        result.end = player.position;
        if !combo.is_active() {
            break;
        }
    }
    assert!(pending_budget.is_none());
    assert!(!combo.is_active());
    assert_eq!(jumps, 2);
    assert!(saw_sprint && saw_slide && saw_glide);
    assert!(result.bullet_time < result.double_time && result.double_time < result.glide_time);
    result
}
fn land_and_refill(p: &mut Player, world: &World, tuning: &Tuning, hz: u32) {
    let dt = 1.0 / hz as f32;
    let mut contacted = false;
    let mut refilled = false;
    for _ in 0..hz * 15 {
        let before = p.jumps_remaining;
        let was_grounded = p.grounded;
        p.step(&Input::default(), dt, world, tuning);
        valid_pose(p, world, tuning);
        if !was_grounded && !p.grounded {
            assert_eq!(p.jumps_remaining, before, "airborne refill before contact");
        }
        contacted |= p.grounded;
        if contacted
            && p.grounded
            && p.jumps_remaining == tuning.max_jumps
            && p.bullet_jumps_left == tuning.max_bullet_jumps
        {
            refilled = true;
            break;
        }
    }
    assert!(
        contacted && refilled,
        "completed sequence must land and rearm"
    );
}
#[test]
fn viewer_preset_combo_repeats_without_airborne_refill_at_three_rates() {
    let world = World {
        terrain: movement_lab::world::WorldTerrain::Boxes, // Explicit box-only backend.
        solids: vec![Solid::new(
            Vec3::new(-500.0, -2.0, -500.0),
            Vec3::new(500.0, 0.0, 500.0),
        )],
    };
    let tuning = tuning();
    let mut summaries = Vec::new();
    for hz in [60, 120, 240] {
        let dt = 1.0 / hz as f32;
        let mut p = settle(&world, &tuning, dt);
        let first = sequence(&mut p, &world, &tuning, hz);
        land_and_refill(&mut p, &world, &tuning, hz);
        let second = sequence(&mut p, &world, &tuning, hz);
        land_and_refill(&mut p, &world, &tuning, hz);
        eprintln!("COMBO_METRICS open-floor {hz}Hz first={first:?} second={second:?}");
        summaries.push((hz, first));
    }
    let reference = &summaries[1].1;
    for (hz, m) in &summaries {
        eprintln!(
            "COMBO_RATE_DELTA {hz}Hz apex={} end_distance={} bullet_time={} double_time={}",
            m.apex - reference.apex,
            m.end.distance(reference.end),
            m.bullet_time - reference.bullet_time,
            m.double_time - reference.double_time
        );
    }
}
#[test]
fn viewer_graybox_spawn_combo_uses_world_target_and_completes_at_three_rates() {
    let world = World::graybox();
    let tuning = tuning();
    for hz in [60, 120, 240] {
        let mut p = settle(&world, &tuning, 1.0 / hz as f32);
        let m = sequence(&mut p, &world, &tuning, hz);
        assert!(
            m.targeted_launch,
            "graybox fixture should exercise the viewer target query"
        );
        land_and_refill(&mut p, &world, &tuning, hz);
        eprintln!("COMBO_METRICS graybox {hz}Hz {m:?}");
    }
}
