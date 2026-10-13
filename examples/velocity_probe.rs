//! JSON-lines adapter for independent comparisons of the velocity kernel.
//! Inputs and reference results are supplied externally; no game data is bundled.

use glam::Vec3;
use movement_lab::velocity::{VelocityTuning, accelerate_velocity};
use serde::{Deserialize, Serialize};
use std::io::{self, BufRead, Write};

#[derive(Deserialize)]
struct Case {
    id: String,
    velocity: [f32; 3],
    target: [f32; 3],
    dt: f32,
    acceleration: f32,
    turn_rate: f32,
    deceleration: f32,
    max_speed: f32,
    angle_cutoff_degrees: f32,
}

#[derive(Serialize)]
struct ResultRow {
    id: String,
    velocity: [f32; 3],
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut output = io::BufWriter::new(io::stdout().lock());
    for line in io::stdin().lock().lines() {
        let case: Case = serde_json::from_str(&line?)?;
        let velocity = accelerate_velocity(
            Vec3::from_array(case.velocity),
            Vec3::from_array(case.target),
            case.dt,
            VelocityTuning {
                acceleration: case.acceleration,
                turn_rate: case.turn_rate,
                deceleration: case.deceleration,
                max_speed: case.max_speed,
                angle_cutoff_degrees: case.angle_cutoff_degrees,
            },
        );
        serde_json::to_writer(
            &mut output,
            &ResultRow {
                id: case.id,
                velocity: velocity.to_array(),
            },
        )?;
        output.write_all(b"\n")?;
    }
    output.flush()?;
    Ok(())
}
