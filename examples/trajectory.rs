//! Reproducible simulated trajectories; never a source of gameplay measurements.
use glam::Vec3;
use movement_lab::jump_lifecycle::{JumpProperties, ResetDecision};
use movement_lab::{Input, Player, Solid, Tuning, World};
use serde::Serialize;
use std::{
    collections::BTreeSet,
    fs,
    io::{BufWriter, Write},
    path::PathBuf,
};

#[derive(Serialize)]
struct Sample {
    time: f64,
    position: [f32; 3],
    /// Composed channel velocity; collision displacement may clip it.
    velocity: [f32; 3],
    base_velocity: [f32; 3],
    slide_additive_velocity: [f32; 3],
    roll_additive_velocity: [f32; 3],
    state: String,
    grounded: bool,
    jumps_remaining: u8,
    air_jumps: u8,
    bullet_jumps: u8,
    air_rolls: u8,
    aim_resource: f32,
    wall_jump_subtype: u8,
    jump_properties: JumpProperties,
    last_jump_reset: ResetDecision,
}
#[derive(Serialize)]
struct Trace {
    provenance: &'static str,
    scenario: String,
    hz: u32,
    duration: f64,
    tuning: Tuning,
    spawn: [f32; 3],
    initial_velocity: [f32; 3],
    input_event_times: Vec<f64>,
    states_visited: BTreeSet<String>,
    max_height: f32,
    max_speed: f32,
    nonpenetration: bool,
    samples: Vec<Sample>,
}
fn schedule(name: &str, t: f64, edge: bool) -> Input {
    let mut i = Input::default();
    match name {
        "wall-latch-enter" | "wall-latch-reject" => {
            i.aim = t < 0.5;
        }
        "wall-facing" | "wall-along" | "wall-chain" | "wall-double" => {
            i.camera_forward = Vec3::new(-0.8, 0.6, 0.);
            let first = edge && (t - 0.1).abs() < 1e-8;
            if (0.1..0.15).contains(&t) {
                i.movement = if name == "wall-along" {
                    Vec3::NEG_Z
                } else {
                    Vec3::X
                };
            }
            i.crouch = name == "wall-chain" && (0.3..0.4).contains(&t);
            i.jump = first
                || (edge
                    && (t - 0.35).abs() < 1e-8
                    && matches!(name, "wall-chain" | "wall-double"));
        }
        "jump-up" | "double-up" => {
            i.jump = edge
                && ((t - 0.1).abs() < 1e-8 || (name == "double-up" && (t - 0.35).abs() < 1e-8));
        }
        "bullet-level" | "bullet-up" | "bullet-down" | "bullet-target" => {
            let angle: f32 = match name {
                "bullet-up" => 45.,
                "bullet-down" => -45.,
                "bullet-target" => 30.,
                _ => 0.,
            };
            i.camera_forward = Vec3::new(0., angle.to_radians().sin(), -angle.to_radians().cos());
            if name == "bullet-target" {
                i.aim_target = Some(Vec3::new(0., 2., -4.));
            }
            i.crouch = t < 0.2;
            i.jump = edge && (t - 0.1).abs() < 1e-8;
        }
        "air-dive" => {
            i.camera_forward = Vec3::new(0., -0.8, -0.6);
            i.crouch = t < 0.2;
            i.jump = edge && (t - 0.1).abs() < 1e-8;
        }
        "chain" => {
            i.movement = Vec3::NEG_Z;
            i.camera_forward = Vec3::new(0., 0.6, -0.8);
            i.sprint = true;
            i.crouch = (0.4..0.65).contains(&t);
            i.jump = edge && ((t - 0.5).abs() < 1e-8 || (t - 0.85).abs() < 1e-8);
            i.aim = (1.0..2.0).contains(&t);
            i.roll = edge && (t - 2.1).abs() < 1e-8;
        }
        "roll-coast" | "roll-opposed" | "roll-orthogonal" => {
            i.camera_forward = match name {
                "roll-opposed" => Vec3::NEG_X,
                "roll-orthogonal" => Vec3::NEG_Z,
                _ => Vec3::X,
            };
            i.roll = edge && (t - 0.1).abs() < 1e-8;
        }
        "slide-coast" => {
            i.movement = if t < 0.6 { Vec3::NEG_Z } else { Vec3::ZERO };
            i.sprint = t < 0.6;
            i.crouch = t >= 0.4;
        }
        "aim-rise" | "aim-fall" => {
            i.aim = (0.1..1.0).contains(&t);
        }
        _ => unreachable!(),
    }
    i
}
fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut args = std::env::args().skip(1);
    let scenario = args.next().unwrap_or_else(|| "chain".into());
    if scenario == "--help" {
        println!(
            "cargo run --example trajectory -- [wall-latch-enter|wall-latch-reject|wall-facing|wall-along|wall-chain|wall-double|chain|jump-up|double-up|slide-coast|roll-coast|roll-opposed|roll-orthogonal|aim-rise|aim-fall|bullet-level|bullet-up|bullet-down|bullet-target|air-dive] [Hz] [output.json] [tuning.json]"
        );
        return Ok(());
    }
    if ![
        "wall-latch-enter",
        "wall-latch-reject",
        "wall-facing",
        "wall-along",
        "wall-chain",
        "wall-double",
        "chain",
        "jump-up",
        "double-up",
        "slide-coast",
        "roll-coast",
        "roll-opposed",
        "roll-orthogonal",
        "aim-rise",
        "aim-fall",
        "bullet-level",
        "bullet-up",
        "bullet-down",
        "bullet-target",
        "air-dive",
    ]
    .contains(&scenario.as_str())
    {
        return Err("unknown scenario".into());
    }
    let hz: u32 = args.next().unwrap_or_else(|| "120".into()).parse()?;
    if !(15..=1000).contains(&hz) {
        return Err("Hz must be 15..1000".into());
    }
    let output: PathBuf = args
        .next()
        .unwrap_or_else(|| "trajectory.json".into())
        .into();
    let tuning = if let Some(path) = args.next() {
        serde_json::from_str::<Tuning>(&fs::read_to_string(path)?)?
    } else {
        Tuning::default()
    };
    if !tuning.is_valid() {
        return Err("invalid tuning".into());
    }
    let mut world = World {
        terrain: Default::default(),
        solids: vec![Solid::new(
            Vec3::new(-500., -2., -500.),
            Vec3::new(500., 0., 500.),
        )],
    };
    let wall_trial = scenario.starts_with("wall-");
    if wall_trial {
        world.solids.push(Solid::new(
            Vec3::new(2., 0., -100.),
            Vec3::new(3., 30., 100.),
        ));
    }
    let starts_airborne =
        scenario == "air-dive" || scenario.starts_with("aim-") || scenario.starts_with("roll-");
    let latch_trial = scenario.starts_with("wall-latch-");
    let spawn = if latch_trial {
        Vec3::new(2. - tuning.radius - 0.01, 3., 0.)
    } else if wall_trial {
        Vec3::new(1.69, 3., 0.)
    } else {
        Vec3::new(0., if starts_airborne { 12. } else { 0.001 }, 0.)
    };
    let mut player = Player::new(spawn);
    // The first step initializes these from tuning. Match that configuration in
    // the time-zero snapshot too, without advancing an airborne trial early.
    player.jumps_remaining = tuning.max_jumps;
    player.air_jumps_left = tuning.max_jumps;
    player.bullet_jumps_left = tuning.max_bullet_jumps;
    player.air_rolls_left = tuning.max_air_rolls;
    if latch_trial {
        player.velocity.x = if scenario == "wall-latch-enter" {
            4.
        } else {
            -4.
        };
    } else if scenario.starts_with("roll-") {
        player.velocity.x = 8.;
    } else if scenario == "air-dive" {
        player.velocity.y = -20.;
    } else if scenario.starts_with("aim-") {
        player.velocity.y = if scenario == "aim-rise" { 8. } else { -8. };
    } else if !wall_trial {
        player.step(&Input::default(), 1. / 120., &world, &tuning);
    }
    let initial_velocity = player.combined_velocity().to_array();
    let duration = 5.;
    let events = if latch_trial {
        vec![0., 0.5, 5.]
    } else if wall_trial {
        vec![0., 0.1, 0.15, 0.3, 0.35, 0.4, 5.]
    } else if scenario == "jump-up" || scenario == "double-up" {
        vec![0., 0.1, 0.35, 5.]
    } else if scenario == "chain" {
        vec![0., 0.4, 0.5, 0.65, 0.85, 1., 2., 2.1, 5.]
    } else if scenario == "slide-coast" {
        vec![0., 0.4, 0.6, 5.]
    } else if scenario.starts_with("aim-") {
        vec![0., 0.1, 1., 5.]
    } else {
        vec![0., 0.1, 0.2, 5.]
    };
    // Split frame intervals at exact input changes so comparisons do not silently
    // change event timing between frame rates. Outer frame partitions still vary.
    let mut boundaries = events.clone();
    boundaries.extend((0..=(duration * hz as f64) as u32).map(|n| n as f64 / hz as f64));
    boundaries.sort_by(f64::total_cmp);
    boundaries.dedup_by(|a, b| (*a - *b).abs() < 1e-9);
    let mut trace = Trace {
        provenance: "simulated-prototype-unmeasured",
        scenario: scenario.clone(),
        hz,
        duration,
        tuning,
        spawn: spawn.to_array(),
        initial_velocity,
        input_event_times: events.clone(),
        states_visited: BTreeSet::new(),
        max_height: 0.,
        max_speed: 0.,
        nonpenetration: true,
        samples: vec![],
    };
    let record = |trace: &mut Trace, player: &Player, time| {
        let state = format!("{:?}", player.state);
        trace.states_visited.insert(state.clone());
        trace.max_height = trace.max_height.max(player.position.y);
        trace.max_speed = trace.max_speed.max(player.combined_velocity().length());
        trace.nonpenetration &=
            world.can_occupy(player.position, tuning.radius, player.height(&tuning));
        assert!(
            player.position.is_finite() && player.combined_velocity().is_finite(),
            "nonfinite simulation"
        );
        trace.samples.push(Sample {
            time,
            position: player.position.to_array(),
            velocity: player.combined_velocity().to_array(),
            base_velocity: player.velocity.to_array(),
            slide_additive_velocity: player.slide_velocity().to_array(),
            roll_additive_velocity: player.roll_velocity().to_array(),
            state,
            grounded: player.grounded,
            jumps_remaining: player.jumps_remaining,
            air_jumps: player.air_jumps_left,
            bullet_jumps: player.bullet_jumps_left,
            air_rolls: player.air_rolls_left,
            aim_resource: player.aim_resource,
            wall_jump_subtype: player.wall_jump_subtype,
            jump_properties: player.jump_lifecycle().properties,
            last_jump_reset: player.last_jump_reset(),
        });
    };
    record(&mut trace, &player, 0.);
    for times in boundaries.windows(2) {
        let t = times[0];
        let dt = (times[1] - t) as f32;
        let edge = events.iter().any(|event| (t - event).abs() < 1e-8);
        player.step(&schedule(&scenario, t + 1e-10, edge), dt, &world, &tuning);
        record(&mut trace, &player, times[1]);
    }
    if let Some(parent) = output.parent().filter(|p| !p.as_os_str().is_empty()) {
        fs::create_dir_all(parent)?;
    }
    let mut writer = BufWriter::new(fs::File::create(&output)?);
    serde_json::to_writer_pretty(&mut writer, &trace)?;
    writer.write_all(b"\n")?;
    println!(
        "SIMULATED {} at {}Hz: {} samples, apex {:.3}, states {:?}, nonpenetration {} -> {}",
        scenario,
        hz,
        trace.samples.len(),
        trace.max_height,
        trace.states_visited,
        trace.nonpenetration,
        output.display()
    );
    if !trace.nonpenetration {
        return Err("trajectory penetrates world geometry".into());
    }
    Ok(())
}
