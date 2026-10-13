//! Supplied-base adapter for isolated bullet-direction comparisons.
//! This deliberately excludes the lab camera-forward normalization policy.

use glam::Vec3;
use movement_lab::bullet::bullet_direction_from_base;
use serde::{Deserialize, Serialize};
use std::io::{self, BufRead, Write};

#[derive(Deserialize)]
struct Case {
    id: String,
    forward: [f32; 3],
    anchor: [f32; 3],
    target: [f32; 3],
    airborne: bool,
}

#[derive(Serialize)]
struct ResultRow {
    id: String,
    direction: [f32; 3],
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut output = io::BufWriter::new(io::stdout().lock());
    for line in io::stdin().lock().lines() {
        let case: Case = serde_json::from_str(&line?)?;
        let direction = bullet_direction_from_base(
            Vec3::from_array(case.forward),
            Vec3::from_array(case.anchor),
            Vec3::from_array(case.target),
            case.airborne,
        );
        serde_json::to_writer(
            &mut output,
            &ResultRow {
                id: case.id,
                direction: direction.to_array(),
            },
        )?;
        output.write_all(b"\n")?;
    }
    output.flush()?;
    Ok(())
}
