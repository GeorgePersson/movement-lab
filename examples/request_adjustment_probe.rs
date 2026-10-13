//! JSONL protocol uses float bits so signed zero and exact inputs survive transport.
use movement_lab::request_adjustment::*;
use serde::Deserialize;
use serde_json::json;
use std::io::{self, BufRead};
#[derive(Deserialize)]
struct Input {
    request: [u32; 4],
    normal: [u32; 4],
    projection_mask: [u32; 4],
    dt: u32,
    value_3c8: u32,
    cached_lower: u32,
    upper: u32,
    split_vertical: u8,
}
fn main() {
    for line in io::stdin().lock().lines() {
        let line = line.expect("input");
        let out = match serde_json::from_str::<Input>(&line) {
            Err(e) => json!({"error":e.to_string()}),
            Ok(i) => match adjust(AdjustmentInput {
                request: i.request.map(f32::from_bits),
                normal: i.normal.map(f32::from_bits),
                projection_mask: i.projection_mask.map(f32::from_bits),
                dt: f32::from_bits(i.dt),
                value_3c8: f32::from_bits(i.value_3c8),
                cached_lower: f32::from_bits(i.cached_lower),
                upper: f32::from_bits(i.upper),
                split_vertical: i.split_vertical,
            }) {
                Ok(o) => {
                    json!({"request":o.request.map(f32::to_bits),"split_vertical":o.split_vertical})
                }
                Err(e) => json!({"error":format!("{e:?}")}),
            },
        };
        println!("{out}");
    }
}
