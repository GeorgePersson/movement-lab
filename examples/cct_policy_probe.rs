//! Scripted-boundary CCT probe. Integer float transport avoids decimal f64 loss.
use movement_lab::cct_policy::{self, BoxHit, Error, Pass, Position, Scene, Settings, Vector};
use serde::Deserialize;
use serde_json::{Value, json};
use std::io::{self, BufRead};

#[derive(Clone, Deserialize)]
struct HitRule {
    distance_bits: Option<u32>,
    fraction_bits: Option<u32>,
    normal_bits: [u32; 3],
    point_bits: Option<[u64; 3]>,
    behavior: u8,
}
#[derive(Deserialize)]
struct Input {
    mode: String,
    center_bits: [u64; 3],
    displacement_bits: [u32; 3],
    side_bits: [u32; 3],
    up_bits: [u32; 3],
    contact_offset_bits: u32,
    step_offset_bits: u32,
    min_distance_bits: u32,
    normalize_components: bool,
    prevent_ceiling_slide: bool,
    overlap_recovery: bool,
    pass_id: u8,
    budget: u32,
    hits: [Vec<Option<HitRule>>; 3],
    repeat: bool,
    #[serde(default)]
    custom_response: bool,
    #[serde(default)]
    walk_experiment: bool,
    #[serde(default)]
    nonwalkable_mode: u32,
    #[serde(default = "default_slope_bits")]
    slope_limit_bits: u32,
}
fn default_slope_bits() -> u32 {
    0.7_f32.to_bits()
}
fn hex(bytes: impl Iterator<Item = u8>) -> String {
    bytes.map(|b| format!("{b:02x}")).collect()
}
fn f32hex(v: impl IntoIterator<Item = f32>) -> String {
    hex(v.into_iter().flat_map(f32::to_le_bytes))
}
fn f64hex(v: Position) -> String {
    hex(v.into_iter().flat_map(f64::to_le_bytes))
}
struct Script {
    hits: [Vec<Option<HitRule>>; 3],
    repeat: bool,
    overlap_recovery: bool,
    consumed: [usize; 3],
    current_behavior: u8,
    trace: Vec<Value>,
}
impl Scene for Script {
    fn prepare(&mut self, center: Position, delta: Vector, side: Vector) -> Result<(), Error> {
        self.trace
            .push(json!({"kind":"prepare", "center":f64hex(center),
            "delta":f32hex(delta), "side":f32hex(side)}));
        Ok(())
    }
    fn query(
        &mut self,
        center: Position,
        direction: Vector,
        limit: f32,
        pass: Pass,
    ) -> Result<Option<BoxHit>, Error> {
        self.trace
            .push(json!({"kind":"query", "center":f64hex(center),
            "direction":f32hex(direction), "limit":f32hex([limit]), "pass_id":pass as u8}));
        let slot = pass as usize;
        let index = self.consumed[slot];
        self.consumed[slot] += 1;
        let row = self.hits[slot].get(index).or_else(|| {
            if self.repeat {
                self.hits[slot].last()
            } else {
                None
            }
        });
        let Some(Some(row)) = row else {
            return Ok(None);
        };
        let distance = row
            .distance_bits
            .map(f32::from_bits)
            .unwrap_or_else(|| limit * row.fraction_bits.map(f32::from_bits).unwrap_or(0.5));
        if distance >= limit || (distance == 0. && !self.overlap_recovery) {
            return Ok(None);
        }
        self.current_behavior = row.behavior;
        Ok(Some(BoxHit {
            distance,
            normal: row.normal_bits.map(f32::from_bits),
            point: Some(
                row.point_bits
                    .map(|p| p.map(f64::from_bits))
                    .unwrap_or([0.; 3]),
            ),
            shape_id: index as u32,
        }))
    }
    fn report(
        &mut self,
        _: &BoxHit,
        direction: Vector,
        distance: f32,
        pass: Pass,
    ) -> Result<u8, Error> {
        self.trace.push(json!({"kind":"report", "direction":f32hex(direction),
            "distance":f32hex([distance]), "pass_id":pass as u8, "behavior":self.current_behavior}));
        Ok(self.current_behavior)
    }
}
fn run(row: Input) -> Result<Value, Error> {
    let center = row.center_bits.map(f64::from_bits);
    let displacement = row.displacement_bits.map(f32::from_bits);
    let settings = Settings {
        up: row.up_bits.map(f32::from_bits),
        contact_offset: f32::from_bits(row.contact_offset_bits),
        step_offset: f32::from_bits(row.step_offset_bits),
        min_distance: f32::from_bits(row.min_distance_bits),
        overlap_recovery: row.overlap_recovery,
        prevent_ceiling_slide: row.prevent_ceiling_slide,
        normalize_components: row.normalize_components,
        custom_response: row.custom_response,
        walk_experiment: row.walk_experiment,
        standing_on_moving: false,
        nonwalkable_mode: row.nonwalkable_mode,
        slope_limit: f32::from_bits(row.slope_limit_bits),
    };
    let mut script = Script {
        hits: row.hits,
        repeat: row.repeat,
        overlap_recovery: row.overlap_recovery,
        consumed: [0; 3],
        current_behavior: 0,
        trace: Vec::new(),
    };
    let (position, flags, results, ids) = match row.mode.as_str() {
        "split" => {
            let out = cct_policy::move_with_scene(center, displacement, &settings, &mut script)?;
            (
                out.position,
                Some(out.collision_flags),
                out.passes,
                vec![0, 1, 2],
            )
        }
        "iterate" => {
            let pass = match row.pass_id {
                0 => Pass::Up,
                1 => Pass::Side,
                2 => Pass::Down,
                _ => return Err(Error::InvalidInput),
            };
            let out = cct_policy::iterate(
                center,
                displacement,
                row.side_bits.map(f32::from_bits),
                pass,
                row.budget,
                &settings,
                &mut script,
            )?;
            (out.position, None, vec![out], vec![row.pass_id])
        }
        _ => return Err(Error::InvalidInput),
    };
    let iterations: Vec<_> = results
        .iter()
        .zip(ids)
        .map(|(x, id)| {
            json!({
                "pass_id":id, "position_bits":x.position.map(f64::to_bits),
                "attempted":x.attempted, "collisions":x.collisions, "passes":x.iterations,
            })
        })
        .collect();
    Ok(
        json!({"position_bits":position.map(f64::to_bits), "collision_flags":flags,
        "iterations":iterations, "trace":script.trace}),
    )
}
fn main() -> Result<(), Box<dyn std::error::Error>> {
    for line in io::stdin().lock().lines() {
        let result = match serde_json::from_str::<Input>(&line?) {
            Ok(row) => run(row).unwrap_or_else(|e| json!({"error":e})),
            Err(_) => json!({"error":"InvalidJson"}),
        };
        println!("{}", serde_json::to_string(&result)?);
    }
    Ok(())
}
