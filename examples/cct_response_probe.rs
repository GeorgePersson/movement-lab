//! JSONL response probe with exact u64 f64 transport and u32 f32 transport.
use movement_lab::cct_response::response;
use serde::{Deserialize, Serialize};
use std::io::{self, BufRead};
#[derive(Deserialize)]
struct Input {
    target_bits: [u64; 3],
    current_bits: [u64; 3],
    direction_bits: [u32; 3],
    normal_bits: [u32; 3],
    bump_bits: u32,
    tangent_bits: u32,
    normalize_components: bool,
}
#[derive(Serialize)]
struct Output {
    target_bits: [u64; 3],
}
fn main() -> Result<(), Box<dyn std::error::Error>> {
    for line in io::stdin().lock().lines() {
        let row: Input = serde_json::from_str(&line?)?;
        let target = row.target_bits.map(f64::from_bits);
        let current = row.current_bits.map(f64::from_bits);
        let direction = row.direction_bits.map(f32::from_bits);
        let normal = row.normal_bits.map(f32::from_bits);
        let bump = f32::from_bits(row.bump_bits);
        let tangent = f32::from_bits(row.tangent_bits);
        if !target.into_iter().chain(current).all(f64::is_finite)
            || !direction
                .into_iter()
                .chain(normal)
                .chain([bump, tangent])
                .all(f32::is_finite)
        {
            return Err("finite supplied inputs required".into());
        }
        let out = response(
            target,
            current,
            direction,
            normal,
            bump,
            tangent,
            row.normalize_components,
        );
        println!(
            "{}",
            serde_json::to_string(&Output {
                target_bits: out.map(f64::to_bits)
            })?
        );
    }
    Ok(())
}
