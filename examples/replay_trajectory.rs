//! Supplied input replay in the original LAB controller; never gameplay data.
//! No camera calibration, input fitting, live configuration or meters are inferred.

use glam::Vec3;
use movement_lab::{Input, Player, Tuning, World};
use serde::{Deserialize, Serialize};
use std::{
    fs::{self, File, OpenOptions},
    io::{Read, Write},
    path::{Path, PathBuf},
};

const MAX_INPUT_BYTES: u64 = 8 * 1024 * 1024;
const MAX_EVENTS: usize = 10_000;
const MAX_SOLIDS: usize = 256;
const MAX_STEPS: usize = 150_000;

#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
struct Controls {
    movement: Vec3,
    camera_forward: Vec3,
    aim_target: Option<Vec3>,
    jump: bool,
    crouch: bool,
    sprint: bool,
    aim: bool,
    roll: bool,
}

impl From<Input> for Controls {
    fn from(input: Input) -> Self {
        // Schema1 is the legacy pulse adapter, not ordered held-action replay.
        assert!(!input.jump_held && !input.jump_actions.authoritative && input.jump_actions.len==0 && input.jump_edge_id.is_none(), "schema1 cannot encode held/action input");
        Self {
            movement: input.movement,
            camera_forward: input.camera_forward,
            aim_target: input.aim_target,
            jump: input.jump,
            crouch: input.crouch,
            sprint: input.sprint,
            aim: input.aim,
            roll: input.roll,
        }
    }
}

impl From<&Controls> for Input {
    fn from(input: &Controls) -> Self {
        Self {
            movement: input.movement,
            camera_forward: input.camera_forward,
            aim_target: input.aim_target,
            jump: input.jump,
            crouch: input.crouch,
            sprint: input.sprint,
            aim: input.aim,
            roll: input.roll,
            // Existing replay schema explicitly uses the legacy pulse adapter.
            ..Default::default()
        }
    }
}

#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
struct Event {
    time: f64,
    input: Controls,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(try_from = "ReplayWire", into = "ReplayWire")]
struct Replay {
    schema_version: u32,
    scenario: String,
    duration: f64,
    hz: u32,
    /// Position/velocity initialize Player before this explicit neutral preroll.
    /// Replay time zero is the resulting state, not the constructor's state.
    neutral_preroll_seconds: f64,
    initial_position: Vec3,
    initial_velocity: Vec3,
    initial_player: Option<Player>,
    tuning: Tuning,
    world: World,
    /// Complete snapshots; jump/roll true each denote one new pulse, not a hold.
    events: Vec<Event>,
}

/// Schema1 keeps its original shape. Schema2 has one explicit initializer;
/// mixed old/new fields are errors, even if otherwise well typed.
#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case", deny_unknown_fields)]
enum Initialization {
    Spawn {
        position: Vec3,
        velocity: Vec3,
        neutral_preroll_seconds: f64,
    },
    Resume {
        #[serde(deserialize_with = "resume_player")]
        player: Player,
    },
}
#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
struct ReplayWire {
    schema_version: u32,
    scenario: String,
    duration: f64,
    hz: u32,
    #[serde(
        default,
        skip_serializing_if = "Option::is_none",
        deserialize_with = "present_option"
    )]
    neutral_preroll_seconds: Option<f64>,
    #[serde(
        default,
        skip_serializing_if = "Option::is_none",
        deserialize_with = "present_option"
    )]
    initial_position: Option<Vec3>,
    #[serde(
        default,
        skip_serializing_if = "Option::is_none",
        deserialize_with = "present_option"
    )]
    initial_velocity: Option<Vec3>,
    #[serde(
        default,
        skip_serializing_if = "Option::is_none",
        deserialize_with = "present_option"
    )]
    initialization: Option<Initialization>,
    tuning: Tuning,
    world: World,
    events: Vec<Event>,
}
fn require_full_snapshot(
    raw: &serde_json::Value,
    canonical: &serde_json::Value,
    path: &str,
) -> Result<(), String> {
    use serde_json::Value;
    match (raw, canonical) {
        (Value::Object(a), Value::Object(b)) => {
            if a.len() != b.len() || a.keys().any(|key| !b.contains_key(key)) {
                return Err(format!(
                    "resume requires exact retained field presence at {path}"
                ));
            }
            for (key, value) in b {
                require_full_snapshot(
                    a.get(key)
                        .ok_or_else(|| format!("resume missing {path}.{key}"))?,
                    value,
                    &format!("{path}.{key}"),
                )?;
            }
        }
        (Value::Array(a), Value::Array(b)) => {
            if a.len() != b.len() {
                return Err(format!("resume array cardinality differs at {path}"));
            }
            for (i, (a, b)) in a.iter().zip(b).enumerate() {
                require_full_snapshot(a, b, &format!("{path}[{i}]"))?;
            }
        }
        (Value::Null, Value::Null)
        | (Value::Bool(_), Value::Bool(_))
        | (Value::String(_), Value::String(_))
        | (Value::Number(_), Value::Number(_)) => {}
        _ => return Err(format!("resume transport type differs at {path}")),
    }
    Ok(())
}
fn resume_player<'de, D: serde::Deserializer<'de>>(deserializer: D) -> Result<Player, D::Error> {
    let raw = serde_json::Value::deserialize(deserializer)?;
    let player: Player = serde_json::from_value(raw.clone()).map_err(serde::de::Error::custom)?;
    let canonical = serde_json::to_value(&player).map_err(serde::de::Error::custom)?;
    require_full_snapshot(&raw, &canonical, "player").map_err(serde::de::Error::custom)?;
    Ok(player)
}
fn present_option<'de, D, T>(deserializer: D) -> Result<Option<T>, D::Error>
where
    D: serde::Deserializer<'de>,
    T: Deserialize<'de>,
{
    T::deserialize(deserializer).map(Some)
}
impl TryFrom<ReplayWire> for Replay {
    type Error = String;
    fn try_from(w: ReplayWire) -> Result<Self, String> {
        let (position, velocity, preroll, player) = match w.schema_version {
            1 => {
                if w.initialization.is_some() {
                    return Err("schema1 does not accept initialization".into());
                }
                (
                    w.initial_position
                        .ok_or("schema1 requires initial_position")?,
                    w.initial_velocity
                        .ok_or("schema1 requires initial_velocity")?,
                    w.neutral_preroll_seconds
                        .ok_or("schema1 requires neutral_preroll_seconds")?,
                    None,
                )
            }
            2 => {
                if w.initial_position.is_some()
                    || w.initial_velocity.is_some()
                    || w.neutral_preroll_seconds.is_some()
                {
                    return Err("schema2 accepts only the tagged initialization; legacy initializer fields are forbidden".into());
                }
                match w.initialization.ok_or("schema2 requires initialization")? {
                    Initialization::Spawn {
                        position,
                        velocity,
                        neutral_preroll_seconds,
                    } => (position, velocity, neutral_preroll_seconds, None),
                    Initialization::Resume { player } => {
                        (player.position, player.velocity, 0., Some(player))
                    }
                }
            }
            _ => return Err("schema_version must be1 or2".into()),
        };
        Ok(Self {
            schema_version: w.schema_version,
            scenario: w.scenario,
            duration: w.duration,
            hz: w.hz,
            neutral_preroll_seconds: preroll,
            initial_position: position,
            initial_velocity: velocity,
            initial_player: player,
            tuning: w.tuning,
            world: w.world,
            events: w.events,
        })
    }
}
impl From<Replay> for ReplayWire {
    fn from(r: Replay) -> Self {
        let initialization = if r.schema_version == 2 {
            Some(match r.initial_player {
                Some(player) => Initialization::Resume { player },
                None => Initialization::Spawn {
                    position: r.initial_position,
                    velocity: r.initial_velocity,
                    neutral_preroll_seconds: r.neutral_preroll_seconds,
                },
            })
        } else {
            None
        };
        Self {
            schema_version: r.schema_version,
            scenario: r.scenario,
            duration: r.duration,
            hz: r.hz,
            neutral_preroll_seconds: (r.schema_version == 1).then_some(r.neutral_preroll_seconds),
            initial_position: (r.schema_version == 1).then_some(r.initial_position),
            initial_velocity: (r.schema_version == 1).then_some(r.initial_velocity),
            initialization,
            tuning: r.tuning,
            world: r.world,
            events: r.events,
        }
    }
}

fn grid(duration: f64, hz: u32) -> Vec<f64> {
    let mut times = vec![0.];
    for frame in 1..=(duration * f64::from(hz)).ceil() as usize {
        let time = frame as f64 / f64::from(hz);
        if time < duration {
            times.push(time);
        }
    }
    times.push(duration);
    times
}

fn solver_dt(start: f64, end: f64) -> Result<f32, String> {
    let dt = (end - start) as f32;
    if !dt.is_normal() || dt <= 0. || dt > 2. {
        return Err("time interval cannot be represented as a positive solver dt".into());
    }
    Ok(dt)
}

fn boundaries(replay: &Replay) -> Vec<f64> {
    let mut times = grid(replay.duration, replay.hz);
    times.extend(replay.events.iter().map(|event| event.time));
    times.sort_by(f64::total_cmp);
    // Exact equality only: nearby input edges must remain separate.
    times.dedup_by(|left, right| *left == *right);
    times
}

fn validate(replay: &Replay) -> Result<Vec<f64>, String> {
    if !matches!(replay.schema_version, 1 | 2)
        || replay.scenario.trim().is_empty()
        || replay.scenario.len() > 128
    {
        return Err(
            "require schema_version1 or2 and a nonempty scenario of at most 128 bytes".into(),
        );
    }
    if !replay.duration.is_finite()
        || !(0. ..=120.).contains(&replay.duration)
        || replay.duration == 0.
    {
        return Err("duration must be finite and in (0,120] seconds".into());
    }
    // Sample every public physics step, rather than conceal internal long-step
    // support transitions. This is an original LAB rate, not a native tick rate.
    if !(if replay.schema_version == 1 {
        240..=1000
    } else {
        120..=1000
    })
    .contains(&replay.hz)
    {
        return Err("hz must be240..=1000 in schema1 or120..=1000 in schema2".into());
    }
    if !replay.neutral_preroll_seconds.is_finite()
        || !(0. ..=30.).contains(&replay.neutral_preroll_seconds)
    {
        return Err("neutral_preroll_seconds must be finite and in [0,30]".into());
    }
    if !replay.initial_position.is_finite()
        || !replay.initial_velocity.is_finite()
        || !replay.tuning.is_valid()
    {
        return Err("initial vectors and valid tuning must be finite".into());
    }
    if replay.world.mesh_terrain().is_some() && replay.tuning.collision.overlap_recovery {
        return Err("selected prepared mesh requires collision.overlap_recovery=false".into());
    }
    replay
        .world
        .validate()
        .map_err(|e| format!("invalid selected world: {e:?}"))?;
    if replay.world.solids.len() > MAX_SOLIDS
        || replay.world.solids.iter().any(|solid| {
            !solid.min.is_finite() || !solid.max.is_finite() || !solid.min.cmple(solid.max).all()
        })
    {
        return Err("world requires at most 256 finite, ordered boxes".into());
    }
    if replay.events.is_empty() || replay.events.len() > MAX_EVENTS || replay.events[0].time != 0. {
        return Err("require 1..=10000 complete snapshots, starting at time zero".into());
    }
    let mut previous = None;
    for event in &replay.events {
        if !event.time.is_finite()
            || event.time < 0.
            || event.time >= replay.duration
            || previous.is_some_and(|time| event.time <= time)
        {
            return Err("event times must strictly increase in [0,duration); terminal pulses cannot execute".into());
        }
        if !event.input.movement.is_finite()
            || !event.input.camera_forward.is_finite()
            || event
                .input
                .aim_target
                .is_some_and(|target| !target.is_finite())
        {
            return Err(
                "all supplied movement, camera and aim-target vectors must be finite".into(),
            );
        }
        previous = Some(event.time);
    }
    let times = boundaries(replay);
    let preroll_steps = (replay.neutral_preroll_seconds * f64::from(replay.hz)).ceil() as usize;
    if times.len() - 1 + preroll_steps > MAX_STEPS {
        return Err("replay and preroll exceed the 150000-step work limit".into());
    }
    for interval in times.windows(2) {
        solver_dt(interval[0], interval[1])?;
    }
    if replay.neutral_preroll_seconds > 0. {
        for interval in grid(replay.neutral_preroll_seconds, replay.hz).windows(2) {
            solver_dt(interval[0], interval[1])?;
        }
    }
    if !replay.world.can_occupy(
        replay.initial_position,
        replay.tuning.radius,
        replay
            .initial_player
            .as_ref()
            .map_or(replay.tuning.standing_height, |p| p.height(&replay.tuning)),
    ) {
        return Err("initial capsule hull overlaps supplied world geometry".into());
    }
    Ok(times)
}

#[derive(Debug, Serialize)]
struct Sample {
    time: f64,
    position: Vec3,
    velocity: Vec3,
    base_velocity: Vec3,
    /// Stored owners after this interval; request transport is separately named.
    slide_velocity: Vec3,
    roll_velocity: Vec3,
    vertical_scalar: f32,
    last_substep_request_base: Vec3,
    last_substep_request_scalar: f32,
    last_substep_request_dt: f32,
    backend_failed: bool,
    /// Feet displacement across the sampled interval, distinct from requests.
    actual_displacement: Option<Vec3>,
    state: movement_lab::MovementState,
    grounded: bool,
    jumps_remaining: u8,
    bullet_jumps: u8,
    air_rolls: u8,
    jump_properties: movement_lab::jump_lifecycle::JumpProperties,
    /// None at time zero; thereafter records the input used for this interval.
    applied_input: Option<Input>,
    interval_start: Option<f64>,
}

fn sample(
    player: &Player,
    time: f64,
    input: Option<Input>,
    start: Option<f64>,
) -> Result<Sample, String> {
    if !player.position.is_finite()
        || !player.velocity.is_finite()
        || !player.combined_velocity().is_finite()
    {
        return Err("simulation produced nonfinite motion; no trace is written".into());
    }
    Ok(Sample {
        time,
        position: player.position,
        velocity: player.combined_velocity(),
        base_velocity: player.velocity,
        slide_velocity: player.slide_velocity(),
        roll_velocity: player.roll_velocity(),
        vertical_scalar: player.velocity.y,
        last_substep_request_base: player.engine_contact_runtime().last_request_base,
        last_substep_request_scalar: player.engine_contact_runtime().last_request_scalar,
        last_substep_request_dt: player.engine_contact_runtime().last_request_dt,
        backend_failed: player.last_world_move_failed(),
        actual_displacement: None,
        state: player.state,
        grounded: player.grounded,
        jumps_remaining: player.jumps_remaining,
        bullet_jumps: player.bullet_jumps_left,
        air_rolls: player.air_rolls_left,
        jump_properties: player.jump_lifecycle().properties,
        applied_input: input,
        interval_start: start,
    })
}

#[derive(Clone, Copy, Debug, Serialize)]
struct Bracket {
    start: f64,
    end: f64,
}

#[derive(Debug, Serialize)]
struct SupportChange {
    bracket: Bracket,
    from_supported: bool,
    to_supported: bool,
}

#[derive(Debug, Serialize)]
struct FlightInterval {
    takeoff: Bracket,
    contact: Bracket,
    minimum_seconds: f64,
    maximum_seconds: f64,
}

#[derive(Debug, Serialize)]
struct Trace {
    provenance: &'static str,
    scenario: String,
    coordinates: &'static str,
    units: &'static str,
    timing_scope: &'static str,
    replay: Replay,
    input_path: Option<PathBuf>,
    initial_player: Player,
    final_player: Player,
    samples: Vec<Sample>,
    simulated_support_changes: Vec<SupportChange>,
    simulated_flight_intervals: Vec<FlightInterval>,
}

fn run(replay: Replay) -> Result<Trace, String> {
    let times = validate(&replay)?;
    let mut player = if let Some(player) = &replay.initial_player {
        replay.world.validate_player_runtime(
            player.engine_contact_runtime(),
            player.position,
            replay.tuning.radius,
            player.height(&replay.tuning),
        )?;
        // Required numeric Player fields cannot deserialize null produced by
        // a nonfinite value; this also covers private retained timer transport.
        let value = serde_json::to_value(player).map_err(|e| e.to_string())?;
        let restored: Player = serde_json::from_value(value)
            .map_err(|e| format!("invalid resumed Player transport: {e}"))?;
        restored
    } else {
        let mut player = Player::new(replay.initial_position);
        player.velocity = replay.initial_velocity;
        player
    };
    if replay.neutral_preroll_seconds > 0. {
        for interval in grid(replay.neutral_preroll_seconds, replay.hz).windows(2) {
            player.step(
                &Input::default(),
                solver_dt(interval[0], interval[1])?,
                &replay.world,
                &replay.tuning,
            );
            if player.last_world_move_failed() {
                return Err("World backend rejected preroll movement".into());
            }
            sample(&player, 0., None, None)?;
        }
    }
    if !replay.world.can_occupy(
        player.position,
        replay.tuning.radius,
        player.height(&replay.tuning),
    ) {
        return Err("resulting preroll hull overlaps supplied world geometry".into());
    }
    let initial_player = player.clone();
    let mut samples = vec![sample(&player, 0., None, None)?];
    let mut changes = Vec::new();
    let mut flights = Vec::new();
    let mut takeoff = None;
    let mut input = Input::default();
    let mut next_event = 0;
    for interval in times.windows(2) {
        let start = interval[0];
        let end = interval[1];
        if next_event < replay.events.len() && replay.events[next_event].time == start {
            input = Input::from(&replay.events[next_event].input);
            next_event += 1;
        }
        let supported = player.grounded;
        let previous_position = player.position;
        player.step(
            &input,
            solver_dt(start, end)?,
            &replay.world,
            &replay.tuning,
        );
        if player.last_world_move_failed() {
            return Err(
                "World backend rejected the staged movement; no physical-miss trace is written"
                    .into(),
            );
        }
        let mut current = sample(&player, end, Some(input), Some(start))?;
        current.actual_displacement = Some(player.position - previous_position);
        samples.push(current);
        if !replay.world.can_occupy(
            player.position,
            replay.tuning.radius,
            player.height(&replay.tuning),
        ) {
            return Err("replay hull penetrates supplied world; no trace is written".into());
        }
        if supported != player.grounded {
            let bracket = Bracket { start, end };
            changes.push(SupportChange {
                bracket,
                from_supported: supported,
                to_supported: player.grounded,
            });
            if supported {
                takeoff = Some(bracket);
            } else if let Some(previous) = takeoff.take() {
                flights.push(FlightInterval {
                    takeoff: previous,
                    contact: bracket,
                    minimum_seconds: bracket.start - previous.end,
                    maximum_seconds: bracket.end - previous.start,
                });
            }
        }
        // Complete snapshots retain held controls, but edges execute just once.
        input.jump = false;
        input.roll = false;
    }
    Ok(Trace {
        provenance: "simulated-prototype-unmeasured",
        scenario: replay.scenario.clone(),
        coordinates: "x-right_y-up_z-back",
        units: "unmeasured LAB world units; no meters conversion supplied or inferred",
        timing_scope: "Nominal f64 replay boundaries; solver dt rounded to f32. Support brackets concern LAB physics publication, not rendered gameplay or native event timestamps. No reference fit.",
        replay,
        input_path: None,
        initial_player,
        final_player: player,
        samples,
        simulated_support_changes: changes,
        simulated_flight_intervals: flights,
    })
}

fn load(path: &Path) -> Result<Replay, Box<dyn std::error::Error>> {
    let mut bytes = Vec::new();
    File::open(path)?
        .take(MAX_INPUT_BYTES + 1)
        .read_to_end(&mut bytes)?;
    if bytes.len() as u64 > MAX_INPUT_BYTES {
        return Err("input exceeds the 8 MiB limit".into());
    }
    Ok(serde_json::from_slice(&bytes)?)
}

fn write_trace(path: &Path, trace: &Trace) -> Result<(), Box<dyn std::error::Error>> {
    // Encode before creating a file; validation/serialization cannot truncate
    // prior output or the supplied input, including path aliases and symlinks.
    let encoded = serde_json::to_vec_pretty(trace)?;
    let mut file = OpenOptions::new().write(true).create_new(true).open(path)?;
    file.write_all(&encoded)?;
    file.write_all(b"\n")?;
    Ok(())
}

fn mesh_template() -> Replay {
    let world = World::mesh(movement_lab::static_terrain::StaticTerrain::course())
        .expect("checked mesh course");
    let mut tuning = Tuning::default();
    tuning.collision.overlap_recovery = false;
    Replay {
        schema_version: 2,
        scenario: "supplied-triangle-course".into(),
        duration: 3.,
        hz: 120,
        neutral_preroll_seconds: 0.1,
        initial_position: world.spawn(),
        initial_velocity: Vec3::ZERO,
        initial_player: None,
        tuning,
        world,
        events: vec![Event {
            time: 0.,
            input: Input::default().into(),
        }],
    }
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args: Vec<_> = std::env::args_os().skip(1).collect();
    if args.len() == 2 && args[0] == "--write-mesh-template" {
        let encoded = serde_json::to_vec_pretty(&mesh_template())?;
        let mut file = OpenOptions::new()
            .write(true)
            .create_new(true)
            .open(&args[1])?;
        file.write_all(&encoded)?;
        file.write_all(b"\n")?;
        return Ok(());
    }
    if args.len() == 1 && args[0] == "--help" {
        println!(
            "cargo run --example replay_trajectory -- INPUT.json NEW_TRACE.json\n--write-mesh-template NEW_INPUT.json writes the same checked mesh-course selected by the viewer.\nSchema1: scenario,duration,hz(240..1000),neutral_preroll_seconds,initial_position,initial_velocity,tuning,world,events[{{time,input}}]. Events start at0; complete snapshots, jump/roll pulses. Schema2 replaces legacy initializer fields with initialization:{{kind:spawn,position,velocity,neutral_preroll_seconds}} or {{kind:resume,player}}. Original LAB simulation only; no reference fit."
        );
        return Ok(());
    }
    if args.len() != 2 {
        return Err(
            "usage: replay_trajectory INPUT.json NEW_TRACE.json; output must not exist".into(),
        );
    }
    let input_path = Path::new(&args[0]);
    let mut trace = run(load(input_path)?)?;
    trace.input_path = Some(fs::canonicalize(input_path)?);
    write_trace(Path::new(&args[1]), &trace)?;
    println!(
        "SIMULATED {}: {} samples, {} support changes, {} bounded flight intervals; reference fit NOT VALIDATED",
        trace.scenario,
        trace.samples.len(),
        trace.simulated_support_changes.len(),
        trace.simulated_flight_intervals.len()
    );
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use movement_lab::Solid;

    fn fixture() -> Replay {
        Replay {
            schema_version: 1,
            scenario: "synthetic-replay-contract".into(),
            duration: 0.2,
            hz: 240,
            neutral_preroll_seconds: 0.1,
            initial_position: Vec3::Y * 0.05,
            initial_velocity: Vec3::ZERO,
            initial_player: None,
            tuning: Tuning::default(),
            world: World {
                terrain: Default::default(),
                solids: vec![Solid::new(
                    Vec3::new(-500., -2., -500.),
                    Vec3::new(500., 0., 500.),
                )],
            },
            events: vec![Event {
                time: 0.,
                input: Input::default().into(),
            }],
        }
    }

    fn pulse(time: f64) -> Event {
        Event {
            time,
            input: Input {
                jump: true,
                movement: Vec3::NEG_Z,
                sprint: true,
                ..Default::default()
            }
            .into(),
        }
    }

    #[test]
    fn off_grid_pulse_is_once_and_other_controls_remain_held() {
        let mut replay = fixture();
        replay.events.push(pulse(0.0037));
        let trace = run(replay).unwrap();
        assert!(trace.initial_player.grounded);
        let pulses: Vec<_> = trace
            .samples
            .iter()
            .filter(|sample| sample.applied_input.is_some_and(|input| input.jump))
            .collect();
        assert_eq!(pulses.len(), 1);
        assert_eq!(pulses[0].interval_start, Some(0.0037));
        assert!(trace.samples.iter().any(|sample| sample.time == 0.0037));
        for sample in trace
            .samples
            .iter()
            .filter(|sample| sample.interval_start.is_some_and(|time| time >= 0.0037))
        {
            let input = sample.applied_input.unwrap();
            assert!(input.sprint && input.movement == Vec3::NEG_Z);
        }
    }

    #[test]
    fn time_zero_edges_execute_and_two_close_edges_are_not_merged() {
        let mut zero = fixture();
        zero.events = vec![pulse(0.)];
        let trace = run(zero).unwrap();
        assert!(trace.samples[1].applied_input.unwrap().jump);
        assert_eq!(trace.samples[1].interval_start, Some(0.));
        let mut replay = fixture();
        replay.events.extend([pulse(0.1), pulse(0.101)]);
        let trace = run(replay).unwrap();
        assert_eq!(
            trace
                .samples
                .iter()
                .filter(|sample| sample.applied_input.is_some_and(|input| input.jump))
                .count(),
            2
        );
        assert_eq!(trace.final_player.jumps_remaining, 0);
        assert_eq!(
            trace
                .simulated_support_changes
                .iter()
                .filter(|change| change.from_supported)
                .count(),
            1
        );
    }

    #[test]
    fn support_intervals_are_bracketed_without_claiming_video_events() {
        let mut replay = fixture();
        replay.duration = 2.;
        replay.events.push(pulse(0.1));
        let trace = run(replay).unwrap();
        assert_eq!(trace.provenance, "simulated-prototype-unmeasured");
        let interval = &trace.simulated_flight_intervals[0];
        assert_eq!(interval.takeoff.start, 0.1);
        assert!(interval.minimum_seconds > 0.);
        assert!(interval.maximum_seconds > interval.minimum_seconds);
        assert!((interval.maximum_seconds - interval.minimum_seconds) <= 2. / 240. + 1e-12);
        assert!(trace.final_player.grounded);
    }

    #[test]
    fn rejects_invalid_schedule_geometry_vectors_and_work_bounds() {
        let mut cases = Vec::new();
        for time in [-1., 0.2, 0.3, f64::NAN] {
            let mut replay = fixture();
            replay.events.push(pulse(time));
            cases.push(replay);
        }
        let mut replay = fixture();
        replay.events.extend([pulse(0.1), pulse(0.1)]);
        cases.push(replay);
        let mut replay = fixture();
        replay.events.extend([pulse(0.1), pulse(0.05)]);
        cases.push(replay);
        let mut replay = fixture();
        replay.events[0].time = 0.01;
        cases.push(replay);
        let mut replay = fixture();
        replay.events[0].input.camera_forward = Vec3::NAN;
        cases.push(replay);
        let mut replay = fixture();
        replay.events[0].input.aim_target = Some(Vec3::INFINITY);
        cases.push(replay);
        let mut replay = fixture();
        replay.world.solids[0].min.x = 1000.;
        cases.push(replay);
        let mut replay = fixture();
        replay.duration = 1e99;
        cases.push(replay);
        let mut replay = fixture();
        replay.hz = u32::MAX;
        cases.push(replay);
        let mut replay = fixture();
        replay.neutral_preroll_seconds = f64::INFINITY;
        cases.push(replay);
        let mut replay = fixture();
        replay.tuning.gravity = f32::NAN;
        cases.push(replay);
        let mut replay = fixture();
        replay.events.push(pulse(1e-300));
        cases.push(replay);
        let mut replay = fixture();
        replay.world.solids = vec![replay.world.solids[0]; MAX_SOLIDS + 1];
        cases.push(replay);
        let mut replay = fixture();
        replay.events = vec![pulse(0.); MAX_EVENTS + 1];
        cases.push(replay);
        let mut replay = fixture();
        replay.duration = 120.;
        replay.neutral_preroll_seconds = 30.;
        replay.hz = 1000;
        // Exactly 150000 grid steps, then one distinct off-grid boundary.
        replay.events.push(pulse(0.0005));
        cases.push(replay);
        for replay in cases {
            assert!(validate(&replay).is_err());
        }
    }

    #[test]
    fn complete_controls_reject_unknown_fields_and_missing_held_values() {
        let mut value = serde_json::to_value(fixture()).unwrap();
        value["events"][0]["input"]["spint"] = serde_json::json!(true);
        assert!(serde_json::from_value::<Replay>(value).is_err());
        let mut value = serde_json::to_value(fixture()).unwrap();
        value["events"][0]["input"]
            .as_object_mut()
            .unwrap()
            .remove("crouch");
        assert!(serde_json::from_value::<Replay>(value).is_err());
    }

    #[test]
    fn writing_preserves_existing_output_and_input_alias() {
        let name = format!(
            "movement-replay-test-{}-{}",
            std::process::id(),
            std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap()
                .as_nanos()
        );
        let directory = std::env::temp_dir().join(name);
        fs::create_dir(&directory).unwrap();
        let input_path = directory.join("input.json");
        let input_bytes = serde_json::to_vec(&fixture()).unwrap();
        fs::write(&input_path, &input_bytes).unwrap();
        let trace = run(load(&input_path).unwrap()).unwrap();
        assert!(write_trace(&input_path, &trace).is_err());
        assert_eq!(fs::read(&input_path).unwrap(), input_bytes);
        let output = directory.join("trace.json");
        write_trace(&output, &trace).unwrap();
        let first = fs::read(&output).unwrap();
        assert!(write_trace(&output, &trace).is_err());
        assert_eq!(fs::read(&output).unwrap(), first);
        fs::remove_dir_all(directory).unwrap();
    }
    #[test]
    fn tagged_initialization_is_strict_and_schema1_round_trips() {
        let original = serde_json::to_value(fixture()).unwrap();
        let parsed: Replay = serde_json::from_value(original.clone()).unwrap();
        assert_eq!(serde_json::to_value(parsed).unwrap(), original);
        let mut mesh = serde_json::to_value(mesh_template()).unwrap();
        assert!(mesh.get("initial_position").is_none());
        assert_eq!(mesh["initialization"]["kind"], "spawn");
        mesh["initial_position"] = serde_json::Value::Null;
        assert!(serde_json::from_value::<Replay>(mesh).is_err());
        let mut mesh = serde_json::to_value(mesh_template()).unwrap();
        mesh["initialization"]["extra"] = serde_json::json!(true);
        assert!(serde_json::from_value::<Replay>(mesh).is_err());
    }
    #[test]
    fn triangle_replay_resume_matches_uninterrupted_runtime_at_viewer_cadence() {
        let mut initial = mesh_template();
        initial.duration = 0.25;
        let first = run(initial.clone()).unwrap();
        assert!(first.final_player.grounded);
        let mut resume = initial.clone();
        resume.neutral_preroll_seconds = 0.;
        resume.initial_position = first.final_player.position;
        resume.initial_velocity = first.final_player.velocity;
        resume.initial_player = Some(first.final_player.clone());
        let resumed =
            run(serde_json::from_slice(&serde_json::to_vec(&resume).unwrap()).unwrap()).unwrap();
        let mut entire = initial;
        entire.duration = 0.5;
        let full = run(entire).unwrap();
        assert_eq!(
            serde_json::to_vec(&resumed.final_player).unwrap(),
            serde_json::to_vec(&full.final_player).unwrap()
        );
    }
    #[test]
    fn triangle_resume_rejects_stale_geometry_and_prepared_domain_options() {
        let mut initial = mesh_template();
        initial.duration = 0.1;
        let first = run(initial.clone()).unwrap();
        initial.initial_player = Some(first.final_player.clone());
        initial.neutral_preroll_seconds = 0.;
        initial.initial_position = first.final_player.position;
        initial.initial_velocity = first.final_player.velocity;
        let mut value = serde_json::to_value(initial.clone()).unwrap();
        value["world"]["terrain"]["asset"]["revision"] = serde_json::json!(60);
        assert!(run(serde_json::from_value(value).unwrap()).is_err());
        initial.tuning.collision.overlap_recovery = true;
        assert!(run(initial).is_err());
    }
    #[test]
    fn resumed_player_requires_every_retained_field_and_finite_report_bytes() {
        let mut input = mesh_template();
        input.duration = 0.1;
        let player = run(input.clone()).unwrap().final_player;
        input.initial_position = player.position;
        input.initial_velocity = player.velocity;
        input.initial_player = Some(player);
        input.neutral_preroll_seconds = 0.;
        let valid = serde_json::to_value(&input).unwrap();
        for field in ["initialized", "mesh", "reports", "last_request_dt"] {
            let mut value = valid.clone();
            value["initialization"]["player"]["engine_contact"]
                .as_object_mut()
                .unwrap()
                .remove(field);
            assert!(
                serde_json::from_value::<Replay>(value).is_err(),
                "missing {field}"
            );
        }
        for group in ["common", "state"] {
            let mut value = valid.clone();
            let mesh = &mut value["initialization"]["player"]["engine_contact"]["mesh"];
            let handles = if group == "common" {
                &mut mesh["common"]["handles"]
            } else {
                &mut mesh["state"]["mesh"]["handles"]
            };
            handles.as_object_mut().unwrap().clear();
            assert!(serde_json::from_value::<Replay>(value).is_err());
        }
        let mut value = valid.clone();
        value["initialization"]["player"]["engine_contact"]["unknown"] = serde_json::json!(true);
        assert!(serde_json::from_value::<Replay>(value).is_err());
        let mut value = valid;
        let bytes = f32::NAN.to_le_bytes();
        for (i, byte) in bytes.into_iter().enumerate() {
            value["initialization"]["player"]["engine_contact"]["reports"][i] =
                serde_json::json!(byte);
        }
        assert!(run(serde_json::from_value(value).unwrap()).is_err());
    }
    #[test]
    fn crouched_snapshot_resumes_below_course_ceiling() {
        let mut input = mesh_template();
        input.duration = 0.1;
        input.neutral_preroll_seconds = 0.;
        input.initial_position = Vec3::new(0., 0.04, 16.);
        let mut player = Player::new(input.initial_position);
        player.crouched = true;
        input.initial_player = Some(player);
        input.events[0].input.crouch = true;
        assert!(run(serde_json::from_slice(&serde_json::to_vec(&input).unwrap()).unwrap()).is_ok());
    }
}
