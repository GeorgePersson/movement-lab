//! JSON-lines adapter for independent bullet-direction comparisons.
//! No game data or reference results are bundled.

use glam::Vec3;
use movement_lab::bullet::bullet_direction;
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
        let direction = bullet_direction(
            Vec3::from_array(case.forward),
            Vec3::from_array(case.anchor),
            Some(Vec3::from_array(case.target)),
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
