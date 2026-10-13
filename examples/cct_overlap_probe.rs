//! Bit-transport probe for supplied overlap-recovery scans.
use movement_lab::cct_overlap::{self, BoxRecord, Penetration};
use serde::Deserialize;
use serde_json::json;
use std::io::{self, BufRead};
#[derive(Deserialize)]
struct Record {
    record_type: u32,
    actor_type: u16,
    actor_flags: u8,
    extents: [u32; 3],
    actor_position: [u32; 3],
    local_position: [u32; 3],
}
#[derive(Deserialize)]
struct Response {
    hit: bool,
    normal: [u32; 3],
    depth: u32,
}
#[derive(Deserialize)]
struct Input {
    center: [u32; 3],
    capsule_q: [u32; 4],
    radius: u32,
    height: u32,
    offset: u32,
    records: Vec<Record>,
    responses: Vec<Response>,
    max_scans: usize,
}
fn main() -> Result<(), Box<dyn std::error::Error>> {
    for line in io::stdin().lock().lines() {
        let c: Input = serde_json::from_str(&line?)?;
        let records: Vec<_> = c
            .records
            .iter()
            .map(|r| BoxRecord {
                record_type: r.record_type,
                actor_type: r.actor_type,
                actor_flags: r.actor_flags,
                extents: r.extents.map(f32::from_bits),
                actor_position: r.actor_position.map(f32::from_bits),
                local_position: r.local_position.map(f32::from_bits),
            })
            .collect();
        let mut count = 0;
        let result = cct_overlap::recover(
            c.center.map(f32::from_bits),
            c.capsule_q.map(f32::from_bits),
            f32::from_bits(c.radius),
            f32::from_bits(c.height),
            f32::from_bits(c.offset),
            &records,
            c.max_scans,
            |_| {
                if c.responses.is_empty() {
                    return Err(cct_overlap::Error::QueryFailed);
                }
                let r = &c.responses[count % c.responses.len()];
                count += 1;
                Ok(r.hit.then(|| Penetration {
                    normal: r.normal.map(f32::from_bits),
                    depth: f32::from_bits(r.depth),
                }))
            },
        );
        let output = match result {
            Ok(out) => {
                json!({"position":out.position.map(f32::to_bits),"successes":out.successes,"scans":out.scans,"queries":out.queries.iter().map(|q|json!({"record":q.record,"capsule_geometry":[2,q.radius.to_bits(),q.half_height.to_bits()],"capsule_pose":[q.capsule_q[0].to_bits(),q.capsule_q[1].to_bits(),q.capsule_q[2].to_bits(),q.capsule_q[3].to_bits(),q.center[0].to_bits(),q.center[1].to_bits(),q.center[2].to_bits()],"box_geometry":[3,q.box_extents[0].to_bits(),q.box_extents[1].to_bits(),q.box_extents[2].to_bits()],"box_pose":[0,0,0,1.0f32.to_bits(),q.box_center[0].to_bits(),q.box_center[1].to_bits(),q.box_center[2].to_bits()]})).collect::<Vec<_>>() })
            }
            Err(error) => json!({"error":error}),
        };
        println!("{}", serde_json::to_string(&output)?);
    }
    Ok(())
}
