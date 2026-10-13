//! Supplied-pose sweep probe, independently compared to original Common API.
use movement_lab::{
    cct_scene::query_endpoints,
    precise_sweep::{self, SweepInput},
};
use serde::Deserialize;
use serde_json::json;
use std::io::{self, BufRead};

#[derive(Deserialize)]
struct Input {
    capsule_q: [f32; 4],
    capsule_p: [f32; 3],
    radius: f32,
    half_height: f32,
    box_q: [f32; 4],
    box_p: [f32; 3],
    extents: [f32; 3],
    direction: [f32; 3],
    distance: f32,
}
fn hex(bytes: &[u8]) -> String {
    bytes.iter().map(|x| format!("{x:02x}")).collect()
}
fn main() -> Result<(), Box<dyn std::error::Error>> {
    for line in io::stdin().lock().lines() {
        let c: Input = serde_json::from_str(&line?)?;
        let q = c.capsule_q;
        let q_squared = ((q[0] * q[0] + q[1] * q[1]) + q[2] * q[2]) + q[3] * q[3];
        if !c
            .capsule_q
            .into_iter()
            .chain(c.capsule_p)
            .chain([c.half_height])
            .all(f32::is_finite)
            || c.half_height < 0.
            || (q_squared - 1.).abs() > 0.001
        {
            return Err("invalid capsule pose".into());
        }
        let (a, b) = query_endpoints(c.capsule_q, c.capsule_p, c.half_height);
        let hit = precise_sweep::sweep(&SweepInput {
            a,
            b,
            radius: c.radius,
            box_q: c.box_q,
            box_p: c.box_p,
            extents: c.extents,
            direction: c.direction,
            distance: c.distance,
            flags: 0x503,
        })
        .map_err(|e| format!("{e:?}"))?;
        let mut record = [0x55_u8; 64];
        record[52..56].copy_from_slice(&hit.distance.to_le_bytes());
        if let Some(flags) = hit.flags {
            record[24..26].copy_from_slice(&flags.to_le_bytes());
        }
        for (start, vector) in [(28, hit.point), (40, hit.normal)] {
            if let Some(v) = vector {
                for (i, x) in v.iter().enumerate() {
                    record[start + i * 4..start + i * 4 + 4].copy_from_slice(&x.to_le_bytes());
                }
            }
        }
        let segment: Vec<u8> = a
            .into_iter()
            .chain(b)
            .chain([c.radius])
            .flat_map(f32::to_le_bytes)
            .collect();
        println!(
            "{}",
            json!({"hit":hit.hit,"record_hex":hex(&record),"segment_hex":hex(&segment)})
        );
    }
    Ok(())
}
